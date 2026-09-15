#include "VoxelNavLinkJumpProxy.h"
#include "VoxelTerrainActor.h"
#include "VoxelPathFollowingComponent.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"

void UVoxelNavLinkJumpProxy::ReceiveLinkReached_Implementation(AActor* Agent, const AVoxelTerrainActor* Terrain, FIntVector Destination)
{
	if (!Agent || !Terrain)
	{
		// 没得跳：跟基类一样直接返回，不放行。跟随组件那边的 LinkWaitTimeout 负责收尾
		Super::ReceiveLinkReached_Implementation(Agent, Terrain, Destination);
		return;
	}

	if (bJumping)
	{
		// 防御：同一个代理被重入（现行交接里每一跳都是 NewObject 出的新实例，不该发生）。
		// 静默撤掉上一次的驱动 —— 不调 CompleteJump，免得把「放行」错发给新的这一跳
		if (TickerHandle.IsValid())
		{
			FTSTicker::GetCoreTicker().RemoveTicker(TickerHandle);
			TickerHandle.Reset();
		}
		RestoreCharacterMovement();
		bJumping = false;
	}

	Follower = Agent->FindComponentByClass<UVoxelPathFollowingComponent>();
	TerrainRef = Terrain;
	JumpAgent = Agent;
	DestinationCoord = Destination;

	StartLocation = Agent->GetActorLocation();
	// 落点的水平口径跟着跟随组件走：宽体型的 footprint 中心不在格心上（见 GetCellStandLocation）
	FVector DestStand = Follower ? Follower->GetCellStandLocation(Destination) : Terrain->CoordToWorldLocation(Destination);

	// 落点的竖直口径：此刻 Agent 正站在起跳格上（StartLinkHop 只在到点后交接），
	// 「它现在的高度 + 两格格心的高差」才和平时走路落在同一个高度上 ——
	// 直接按格心放会把角色按进地板半个胶囊高（基类注释里说的同一个坑）
	if (Follower)
	{
		const FIntVector Standing = Follower->GetCurrentCoord();
		if (Standing.X != MAX_int32 && Standing.Y != MAX_int32 && Standing.Z != MAX_int32)
		{
			DestStand.Z = StartLocation.Z
				+ Terrain->CoordToWorldLocation(Destination).Z - Terrain->CoordToWorldLocation(Standing).Z;
		}
	}
	DestinationLocation = DestStand;

	Elapsed = 0.f;
	bJumping = true;

	// 选飞行路线：Character 优先挂引擎真实弹道（IsFalling/GetVelocity/Landed 全真，
	// 标准动画蓝图不用改），其余走代理自己的运动学弧线。细节见函数内注释
	BeginCharacterFlight(Agent);

	// 先广播起跳（让表现层挂音效/粒子），再起驱动 —— 顺序反过来会漏掉第一帧前的事件
	ReceiveJumpStarted(Agent);

	// 代理是纯 UObject，自己没有 Tick：借全局 ticker 逐帧驱动/监护。
	// CreateUObject 是弱绑定 —— 代理被 GC 后这条 ticker 会自动失效，不会悬空调用
	TickerHandle = FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateUObject(this, &ThisClass::TickJump), 0.f);
}

bool UVoxelNavLinkJumpProxy::TickJump(float DeltaTime)
{
	AActor* Moving = JumpAgent.Get();
	if (!bJumping || !Moving)
	{
		// Agent 没了（被销毁 / 切关卡）：没对象可跳，就地收场
		return false;
	}
	if (!Follower || !Follower->IsUsingLink())
	{
		// 移动在这期间被中止了（StopMovement / 超时 / 被新请求顶掉）：连接已经不是我们的，
		// 既不能继续拽位置也不能放行 —— 只把欠角色的移动模式还回去
		bJumping = false;
		RestoreCharacterMovement();
		return false;
	}

	Elapsed += DeltaTime;

	if (bPhysicsJump)
	{
		// 弹道模式：位移整个交给移动组件（真 Falling、真速度、真碰撞、真落地），我们只做监护。
		// 每帧把水平速度按起跳解兜住 —— 角色落到终端速度前移动组件会对水平分量刹车（空气阻力/
		// 终端下落那套会把 Δxy 短掉），重设后抛物线的落点仍对得上目标格
		if (UCharacterMovementComponent* Movement = CharMovement.Get())
		{
			FVector Velocity = Movement->Velocity;
			Velocity.X = LaunchXYVelocity.X;
			Velocity.Y = LaunchXYVelocity.Y;
			Movement->Velocity = Velocity;
		}

		FaceDestination(Moving, DeltaTime);

		UCharacterMovementComponent* Movement = CharMovement.Get();
		if (Movement && !Movement->IsFalling())
		{
			CompleteJump();		// 引擎自己完成了 Falling→落地切换，动画蓝图能收到 Landed
			return false;
		}
		if (Elapsed >= PhysicsFlightLimit)
		{
			// 撞到中间几何体 / 滑开了 / 迟迟不落：不等了，直接按目标格落位收尾
			UE_LOG(LogTemp, Warning, TEXT("[Voxel] %s 的跳跃 %.1f 秒仍未落地，按目标格落位收尾（检查跳跃路径上是否有障碍）"),
				*GetNameSafe(Moving), PhysicsFlightLimit);
			CompleteJump();
			return false;
		}
		return true;
	}

	// 运动学模式：按 t 解析出抛物线位置。4H·t(1−t) 两端为 0、t=0.5 处取到最高点 JumpHeight
	const float Duration = FMath::Max(JumpDuration, 0.01f);
	const float Alpha = FMath::Clamp(Elapsed / Duration, 0.f, 1.f);
	FVector Location = FMath::Lerp(StartLocation, DestinationLocation, static_cast<double>(Alpha));
	Location.Z += 4.0 * JumpHeight * Alpha * (1.0 - Alpha);

	FaceDestination(Moving, DeltaTime);
	Moving->SetActorLocation(Location, /*bSweep=*/false);

	// 把弧线的解析速度交给移动组件：CMC 每次 PerformMovement 都会把 Velocity 同步进
	// 根组件 ComponentVelocity（= AActor::GetVelocity() 的读数），「按速度挑跳跃段」的
	// 动画逻辑在运动学模式下也有真实反馈（导数：水平恒速 (End−Start)/T，竖直再叠 4H(1−2t)/T²）。
	// 物理每帧按它积分出的那点位移会被我们下一帧的解析位置覆盖，不影响落点
	if (UCharacterMovementComponent* Movement = CharMovement.Get())
	{
		FVector ArcVelocity = (DestinationLocation - StartLocation) / static_cast<double>(Duration);
		ArcVelocity.Z += 4.0 * JumpHeight * (1.0 - 2.0 * Alpha) / (static_cast<double>(Duration) * Duration);
		Movement->Velocity = ArcVelocity;
	}

	if (Alpha >= 1.f)
	{
		CompleteJump();
		return false;	// 落地，撤掉这条 ticker
	}
	return true;
}

void UVoxelNavLinkJumpProxy::CompleteJump()
{
	if (TickerHandle.IsValid())
	{
		// 从 ticker 回调里调它也安全：FTSTicker 对执行中移除做了延迟处理
		FTSTicker::GetCoreTicker().RemoveTicker(TickerHandle);
		TickerHandle.Reset();
	}
	if (!bJumping)
	{
		return;
	}
	bJumping = false;

	AActor* Moving = JumpAgent.Get();
	const bool bStillOurs = Follower && Follower->IsUsingLink();

	// 落点定稿：弹道模式落在的是「目标格附近物理实际停下的位置」（碰撞/摩擦总有几厘米误差），
	// 运动学模式差最后一帧没花完的 dt —— 都用 SnapToCell 按组件口径精确放好，
	// 保证「Actor 位置 ↔ 格坐标」一一对应
	if (Moving && bStillOurs)
	{
		Follower->SnapToCell(DestinationCoord);
	}

	// 先落位再还模式：恢复 Walking 后物理立刻做贴地检测，此时必须已经站在目标格上。
	// 还原里的 StopMovementImmediately 会顺带把速度读数清零（弹道与运动学两条路共用）
	RestoreCharacterMovement();

	if (Moving && bStillOurs)
	{
		ResumePathFollowing(Moving);
	}
}

void UVoxelNavLinkJumpProxy::InterruptJump()
{
	if (bJumping)
	{
		CompleteJump();		// 提前落到目标格并放行，跟随组件接着走剩下的路
	}
}

void UVoxelNavLinkJumpProxy::BeginCharacterFlight(AActor* Agent)
{
	const ACharacter* Character = Cast<ACharacter>(Agent);
	UCharacterMovementComponent* Movement = Character ? Character->GetCharacterMovement() : nullptr;
	if (!Movement)
	{
		return;		// 不是 Character：走纯运动学弧线 + 根组件速度上报（TickJump 里做）
	}

	CharMovement = Movement;
	SavedMovementMode = Movement->MovementMode;
	SavedCustomMode = Movement->CustomMovementMode;
	Movement->StopMovementImmediately();	// 两条路线都从静止起跳：先清掉走路的残余速度

	// 弹道解：顶点比高的一端再抬 JumpHeight → 起跳竖直速度 vz = √(2gH)，H = max(0,Δz) + JumpHeight；
	// 解 ½gT² − vzT + Δz = 0 取落地根得滞空 T；水平按 Δxy/T 匀速 —— 无阻碍时轨迹正好落在目标格站位
	const double DeltaZ = DestinationLocation.Z - StartLocation.Z;
	const double ApexRise = FMath::Max(0.0, DeltaZ) + static_cast<double>(JumpHeight);
	const double Gravity = FMath::Abs(static_cast<double>(Movement->GetGravityZ()));

	FVector Horizontal = DestinationLocation - StartLocation;
	Horizontal.Z = 0.0;

	// 没有控制器驱动时移动组件不跑物理（PerformMovement 整个跳过），弹道会吊死在半空 —— 那种走运动学
	const bool bPhysicsDriven = Character->GetController() != nullptr || Movement->bRunPhysicsWithNoController;
	if (bUseRealPhysicsTrajectory && bPhysicsDriven && Gravity > 1.0 && ApexRise > 1.0)
	{
		const double Vz = FMath::Sqrt(2.0 * Gravity * ApexRise);
		const double AirTime = (Vz + FMath::Sqrt(FMath::Max(Vz * Vz - 2.0 * Gravity * DeltaZ, 0.0))) / Gravity;

		LaunchXYVelocity = FVector::ZeroVector;
		if (const double PlaneDist = Horizontal.Size(); PlaneDist > 1.0 && AirTime > KINDA_SMALL_NUMBER)
		{
			LaunchXYVelocity = Horizontal * (1.0 / AirTime);
		}

		// 真的挂成 Falling：IsFalling() 为真、GetVelocity() 是真抛物线速度、落地引擎自己
		// 切回 Walking 并广播 Landed —— 标准动画蓝图（跳跃/下落状态机）不用改任何节点
		Movement->SetMovementMode(MOVE_Falling);
		Movement->Velocity = LaunchXYVelocity + FVector(0.0, 0.0, Vz);

		bPhysicsJump = true;
		PhysicsFlightLimit = static_cast<float>(AirTime) * 4.f + 1.f;	// 兜底：撞墙/滑开也不至于把连接永久挂住
		return;
	}

	// 退化路径（关了开关、没人驱动、没重力、高太小解不出弹道）：回到运动学弧线，
	// Flying 只是摘掉重力免得物理每帧把弧线拽歪
	Movement->SetMovementMode(MOVE_Flying);
}

void UVoxelNavLinkJumpProxy::RestoreCharacterMovement()
{
	if (UCharacterMovementComponent* Movement = CharMovement.Get())
	{
		// 弹道模式正常落地时引擎已自己切回 Walking（模式没变时 SetMovementMode 直接短路，不会重复广播）；
		// 被中止/超时打断时还挂在 Falling，这里把起跳前的模式还回去
		Movement->SetMovementMode(SavedMovementMode, SavedCustomMode);
		Movement->StopMovementImmediately();
	}
	CharMovement = nullptr;	// 置空保证幂等：BeginDestroy 再兜一次底也不会有副作用
}

void UVoxelNavLinkJumpProxy::FaceDestination(AActor* Moving, float DeltaTime) const
{
	if (!bFaceDestination)
	{
		return;
	}

	// 跳跃的水平方向是恒定的（起点 → 落点的直线），所以朝向转到位就不再变了
	FVector Direction = DestinationLocation - StartLocation;
	Direction.Z = 0.0;
	if (Direction.IsNearlyZero())
	{
		return;		// 原地竖直跳（上下格贴着叠）不改朝向
	}

	const float TargetYaw = Direction.Rotation().Yaw;
	const FRotator Current = Moving->GetActorRotation();
	const float NewYaw = TurnSpeed > 0.f
		? FMath::FixedTurn(Current.Yaw, TargetYaw, TurnSpeed * DeltaTime)	// FixedTurn 走最短的一边
		: TargetYaw;
	if (!FMath::IsNearlyEqual(NewYaw, Current.Yaw, 1e-3f))
	{
		Moving->SetActorRotation(FRotator(Current.Pitch, NewYaw, Current.Roll));	// 只动 Yaw
	}
}

void UVoxelNavLinkJumpProxy::ReceiveJumpStarted_Implementation(AActor* Agent)
{
	// 默认什么都不做，留给蓝图子类挂表现
}

void UVoxelNavLinkJumpProxy::BeginDestroy()
{
	if (bJumping)
	{
		// 移动被中止后地形会把我丢出登记表，之后 GC 走到这里：
		// ticker 是弱绑定会自己掉，但角色的移动模式得还回去，不然它会一直挂在 Falling/Flying
		if (TickerHandle.IsValid())
		{
			FTSTicker::GetCoreTicker().RemoveTicker(TickerHandle);
			TickerHandle.Reset();
		}
		bJumping = false;
		RestoreCharacterMovement();
	}

	Super::BeginDestroy();
}
