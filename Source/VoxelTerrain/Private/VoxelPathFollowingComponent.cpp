// 路径跟随：把「调用方算好的路径」变成逐帧位移。
//
// 职责边界：
//   - 组件不寻路：路径由调用方算好传进来（通常就是 AVoxelTerrainActor::FindPath 的结果）。
//   - 组件只负责「走」：终点合不合法、要不要换个终点，都是调用方的事。会拒收的输入只有两种 ——
//     空路径，以及「路径起点与 Agent 当前所在的格对不上」（起点对不上时直线走向 path[0] 可能切墙）。
//   - 平面跳（同层水平 4 邻，LinkClass 为空）由本组件插值走过去：烘焙保证了这种跳一定同层相邻，
//     所以直线插值不会穿墙。
//   - 非平面跳（台阶 / 手动连接，LinkClass 非空）必须由 UVoxelNavLinkProxy 驱动：占住连接、把
//     Agent 交出去，等它调 ResumePathFollowing 放行后接着走剩下的路。
//     同步放行的代理（默认实现就是）在 StartLinkHop 里就地收尾，异步的（跳跃弧线/爬梯动画）
//     靠 ResumeFromLink 置位、下一帧继续，两条路都不产生递归。
//
// 位移有两套驱动（EVoxelMoveDrive），公式一样：速度取 min(MoveSpeed, 剩余距离/dt)，
// 所以天然不过冲、恰好收敛到格心，区别只在下发方式（SetActorLocation vs 移动组件）。

#include "VoxelPathFollowingComponent.h"

#include "VoxelTerrainActor.h"
#include "VoxelChunk.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Actor.h"
#include "GameFramework/NavMovementInterface.h"

namespace
{
	/** 每帧至少靠近这么多厘米才算「有进展」，否则计入卡住计时 */
	constexpr float StallProgressEpsilon = 1.f;

	/** 平面跳：同层 + 水平 4 邻。烘焙只给这种跳不带代理类，所以组件才能安全地按直线插值过去 */
	bool IsPlanarHop(const FIntVector& From, const FIntVector& To)
	{
		const FIntVector Delta = To - From;
		return Delta.Z == 0 && (FMath::Abs(Delta.X) + FMath::Abs(Delta.Y)) == 1;
	}
}

UVoxelPathFollowingComponent::UVoxelPathFollowingComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;
}

/* ===================== 生命周期 ===================== */

void UVoxelPathFollowingComponent::BeginPlay()
{
	Super::BeginPlay();

	ResolveTerrain();
}

void UVoxelPathFollowingComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	// 静默收尾：连接与占地登记都要还回去，但不广播 —— 销毁流程里回调游戏代码容易踩到半死的对象
	ReleaseActiveLink();

	if (bClaimCells && IsValid(Terrain))
	{
		if (bHasClaim)
		{
			Terrain->ReleaseCoord(ClaimedCoord, GetOwner());
		}
	}
	bHasClaim = false;
	Status = EVoxelPathFollowingStatus::Idle;
	Path.Reset();

	Super::EndPlay(EndPlayReason);
}

void UVoxelPathFollowingComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	AdvanceFollowing(DeltaTime);
}

/* ===================== 查询工具 ===================== */

AVoxelTerrainActor* UVoxelPathFollowingComponent::FindTerrain() const
{
	if (IsValid(Terrain))
	{
		return Terrain;
	}

	// 没显式指定就取世界里的第一块（多块地形请用细节面板 / SetTerrain 指定）
	if (const UWorld* World = GetWorld())
	{
		for (TActorIterator<AVoxelTerrainActor> It(const_cast<UWorld*>(World)); It; ++It)
		{
			return *It;
		}
	}
	return nullptr;
}

AVoxelTerrainActor* UVoxelPathFollowingComponent::ResolveTerrain()
{
	AVoxelTerrainActor* Found = FindTerrain();
	Terrain = Found;
	return Found;
}

AVoxelTerrainActor* UVoxelPathFollowingComponent::GetTerrain() const
{
	return FindTerrain();
}

INavMovementInterface* UVoxelPathFollowingComponent::GetNavMovement() const
{
	return MovementInterface.Get();
}

EVoxelMoveDrive UVoxelPathFollowingComponent::GetActiveDrive() const
{
	return MovementInterface.IsValid() ? EVoxelMoveDrive::NavMovement : EVoxelMoveDrive::DirectLocation;
}

void UVoxelPathFollowingComponent::ResolveDrive()
{
	MovementInterface = nullptr;

	if (Drive == EVoxelMoveDrive::DirectLocation)
	{
		return;
	}

	if (const AActor* Owner = GetOwner())
	{
		// 非模板重载 + Cast：比 FindComponentByInterface<T>() 的模板推导更直白，也和引擎 AIController 的用法一致
		MovementInterface = Owner->FindComponentByInterface(UNavMovementInterface::StaticClass());
	}

	if (!MovementInterface.IsValid() && Drive == EVoxelMoveDrive::NavMovement)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Voxel] %s 上没有实现 INavMovementInterface 的移动组件，本次移动退回直接插值"),
			*GetNameSafe(GetOwner()));
	}
}

FVector UVoxelPathFollowingComponent::CellLocation(const FIntVector& Coord) const
{
	const AVoxelTerrainActor* Found = FindTerrain();
	const FVector Center = Found ? Found->CoordToWorldLocation(Coord) : FVector(Coord);
	return Center + GetCellOffset();
}

void UVoxelPathFollowingComponent::SnapToCell(const FIntVector& Coord)
{
	if (AActor* Owner = GetOwner())
	{
		Owner->SetActorLocation(CellLocation(Coord), /*bSweep=*/false);
	}
}

FVector UVoxelPathFollowingComponent::GetCellOffset() const
{
	// 水平分量一律忽略：角色必须落在格心上，「Actor 位置 ↔ 格坐标」才一一对应
	if (!FMath::IsNearlyZero(LocationOffset.Z))
	{
		return FVector(0.0, 0.0, LocationOffset.Z);		// 手配了就以手配为准
	}
	if (!bAutoLocationOffset || GetActiveDrive() != EVoxelMoveDrive::NavMovement)
	{
		return FVector::ZeroVector;
	}

	// 自动推算：让碰撞柱体的底正好踩在落点格的底面上 = 格心 +（半高 − 半个体素）。
	// 角色原点若停在格心高度是够不到的 —— 格心在地板里，差的就是半个胶囊高，表现是
	// 「一直差 40 厘米、最后被卡住保护按 Blocked 收尾」
	const INavMovementInterface* NavMove = GetNavMovement();
	if (!NavMove)
	{
		return FVector::ZeroVector;
	}

	float Radius = 0.f;
	float HalfHeight = 0.f;
	NavMove->GetSimpleCollisionCylinder(Radius, HalfHeight);
	const float VoxelExtent = GetVoxelExtent();
	if (HalfHeight <= KINDA_SMALL_NUMBER || HalfHeight > VoxelExtent * 1.5f)
	{
		// 没有碰撞柱体（浮空 Pawn 之类），或者体型比格子大得多（推出来的站位会跑到格子外面）：
		// 一律按 0 处理，让角色原点就停在格心上
		return FVector::ZeroVector;
	}

	return FVector(0.0, 0.0, static_cast<double>(HalfHeight) - VoxelExtent * 0.5);
}

void UVoxelPathFollowingComponent::LogCellOffsetOnce()
{
	// 自动竖直偏移第一次生效时说一句，免得「角色怎么比格心高了 40 厘米」变成一个谜
	if (!bLoggedAutoOffset && bAutoLocationOffset && FMath::IsNearlyZero(LocationOffset.Z)
		&& GetActiveDrive() == EVoxelMoveDrive::NavMovement)
	{
		const FVector Offset = GetCellOffset();
		if (!Offset.IsNearlyZero())
		{
			bLoggedAutoOffset = true;
			UE_LOG(LogTemp, Log,
				TEXT("[Voxel] %s 没配 LocationOffset，按碰撞柱体自动把站位竖直偏移取为 %.1f（半高 − 半个体素）：角色原点是碰撞柱体中心，站在格心高度上是够不到的"),
				*GetNameSafe(GetOwner()), Offset.Z);
		}
	}

	// 手配的偏移与按碰撞柱体算出来的差太多时提醒一句：多半会够不到目标点
	if (!FMath::IsNearlyZero(LocationOffset.Z) && GetActiveDrive() == EVoxelMoveDrive::NavMovement)
	{
		if (const INavMovementInterface* NavMove = GetNavMovement())
		{
			float Radius = 0.f;
			float HalfHeight = 0.f;
			NavMove->GetSimpleCollisionCylinder(Radius, HalfHeight);
			const double AutoZ = static_cast<double>(HalfHeight) - GetVoxelExtent() * 0.5;
			if (HalfHeight > KINDA_SMALL_NUMBER && FMath::Abs(LocationOffset.Z - AutoZ) > GetVoxelExtent() * 0.25)
			{
				UE_LOG(LogTemp, Warning,
					TEXT("[Voxel] LocationOffset.Z=%.1f 与按碰撞柱体算出来的 %.1f 差得较多，角色可能够不到目标点（够不到会被卡住保护按 Blocked 收尾）"),
					LocationOffset.Z, AutoZ);
			}
		}
	}
}

FIntVector UVoxelPathFollowingComponent::GetCurrentCoord() const
{
	const AVoxelTerrainActor* Found = FindTerrain();
	const AActor* Owner = GetOwner();
	if (!Found || !Owner)
	{
		return InvalidCoord();
	}

	// 减掉偏移再反推：偏移是「角色原点相对格心」的，不能让它把坐标算到隔壁格。
	// 只减 Z：水平分量被忽略（见 GetCellOffset），否则「Actor 位置 ↔ 格坐标」就不再一一对应
	return Found->WorldLocationToCoord(Owner->GetActorLocation() - GetCellOffset());
}

float UVoxelPathFollowingComponent::GetVoxelExtent() const
{
	if (const AVoxelTerrainActor* Found = FindTerrain())
	{
		const FVector& Size = Found->GetVoxelSize();
		// 取绝对值：体素边长理论上为正，但配错了也不能让容差/刹车距离跟着变成负数
		return FMath::Abs(static_cast<float>(FMath::Min(Size.X, FMath::Min(Size.Y, Size.Z))));
	}
	return 100.f;
}

float UVoxelPathFollowingComponent::GetArrivalTolerance() const
{
	const float VoxelExtent = GetVoxelExtent();
	float Tolerance = VoxelExtent * FMath::Max(ArrivalTolerancePct, 0.f) * 0.01f;

	// 移动组件驱动的角色是「物理停稳」的，收不到格心上的最后一个零头：
	// 加速受限 + 碰撞让它停在离格心几厘米的地方来回蹭，容差太紧（2%）就会永远差一点点、最后被卡住保护收尾。
	// 给它一格的一成，角色仍然稳稳在正确的格子里（下一回合反推格坐标不会跑偏）
	if (GetActiveDrive() == EVoxelMoveDrive::NavMovement)
	{
		Tolerance = FMath::Max(Tolerance, VoxelExtent * 0.1f);
	}

	return FMath::Max(Tolerance, 0.1f);
}

bool UVoxelPathFollowingComponent::HasReachedPathPoint(int32 Index, const FVector& Current, const FVector& Target, float Dist) const
{
	if (Dist <= GetArrivalTolerance())
	{
		return true;
	}

	if (Index <= 0 || !Path.IsValidIndex(Index) || !Path.IsValidIndex(Index - 1))
	{
		return false;		// 起点那一点（修正站位用）只按距离判定
	}

	// 已经越过这一跳的垂足 = 这一格到了。移动组件有加速/刹车，位置可能一步冲过格心，
	// 此时「到格心的距离」反而在变大，只按距离判会永远差一点点
	const FVector Hop = CellLocation(Path[Index].Coord) - CellLocation(Path[Index - 1].Coord);
	const FVector HopDirection = Hop.GetSafeNormal();
	return !HopDirection.IsNearlyZero() && FVector::DotProduct(Target - Current, HopDirection) <= 0.0;
}

bool UVoxelPathFollowingComponent::HasMovementAuthority() const
{
	const INavMovementInterface* NavMove = GetNavMovement();
	return !NavMove || NavMove->CanStopPathFollowing();
}

void UVoxelPathFollowingComponent::FaceDirection(const FVector& Direction, float DeltaTime)
{
	if (!bFaceMoveDirection)
	{
		return;
	}

	AActor* Owner = GetOwner();
	const FVector Flat(Direction.X, Direction.Y, 0.0);
	if (!Owner || Flat.IsNearlyZero())
	{
		return;
	}

	const float TargetYaw = static_cast<float>(Flat.Rotation().Yaw);
	if (TurnRate <= 0.f || DeltaTime <= 0.f)
	{
		Owner->SetActorRotation(FRotator(0.f, TargetYaw, 0.f));		// 配 0（或 dt 为 0）= 瞬时转向
		return;
	}

	// 平滑转向：按角速度限速，并且走最短的那一边（FixedTurn 自己处理 0/360 环绕，不会为 350°→10° 绕一大圈）
	const float CurrentYaw = Owner->GetActorRotation().Yaw;
	const float NewYaw = FMath::FixedTurn(CurrentYaw, TargetYaw, TurnRate * DeltaTime);
	Owner->SetActorRotation(FRotator(0.f, NewYaw, 0.f));
}

void UVoxelPathFollowingComponent::UpdateTickState()
{
	// 暂停 / 空闲都不用推进；等连接放行时反而**必须**继续 Tick —— 要靠它轮询代理的放行与超时
	const bool bNeedsAdvance = Status == EVoxelPathFollowingStatus::Moving
		|| Status == EVoxelPathFollowingStatus::WaitingLink;
	const bool bShouldTick = bTickWhileMoving && bNeedsAdvance;
	if (IsRegistered())
	{
		SetComponentTickEnabled(bShouldTick);
	}
}

void UVoxelPathFollowingComponent::StopNavMovement()
{
	if (INavMovementInterface* NavMove = GetNavMovement())
	{
		// 与 UE 一致：没有移动权限（坠落 / 根运动中）时不动移动组件
		if (HasMovementAuthority())
		{
			NavMove->StopMovementKeepPathing();
		}
	}
}

/* ===================== 请求 ===================== */

bool UVoxelPathFollowingComponent::RequestMove(TArray<FVoxelPathPoint> InPath)
{
	if (bSwitchingRequest)
	{
		// 只有「旧请求被新请求顶掉」的那次 Aborted 广播期间才会走到这里；链式移动请写在 Success 回调里
		UE_LOG(LogTemp, Warning, TEXT("[Voxel] RequestMove 在「被顶掉」的 Aborted 回调里重入，已拒绝"));
		return false;
	}

	AActor* Owner = GetOwner();
	AVoxelTerrainActor* FoundTerrain = ResolveTerrain();

	// 顶掉旧请求：先把它收尾（释放连接与占地登记），再收下新的路径
	if (Status != EVoxelPathFollowingStatus::Idle)
	{
		bSwitchingRequest = true;
		FinishMove(EVoxelPathFollowingResult::Aborted);
		bSwitchingRequest = false;
	}

	Path = MoveTemp(InPath);
	// 起点格就是「我现在站的格」（下面会校验），所以不需要先挪到它的格心上去：
	// 多点的路径直接从第 2 个点开始走，免得开局就卡在「差两厘米到不了自己脚下格心」上。
	// 只有一个点的路径是「对齐到这一格的格心」，那才需要走到位
	PathIndex = (Path.Num() > 1) ? 1 : 0;
	StallTimer = 0.f;
	StallBestDist = TNumericLimits<float>::Max();
	LastResult = FVoxelPathFollowingResultInfo{};
	LastResult.GoalCoord = Path.IsEmpty() ? FIntVector::ZeroValue : Path.Last().Coord;

	if (!Owner || !FoundTerrain)
	{
		Path.Reset();
		UE_LOG(LogTemp, Warning, TEXT("[Voxel] %s 找不到体素地形，移动取消"), *GetNameSafe(Owner));
		FinishMove(EVoxelPathFollowingResult::NoTerrain);
		return false;
	}

	// 组件不寻路：路径由调用方算好传进来，空路径没有什么可走的
	if (Path.IsEmpty())
	{
		UE_LOG(LogTemp, Warning, TEXT("[Voxel] %s 收到空路径，移动取消（路径要由调用方算好传进来）"), *GetNameSafe(Owner));
		FinishMove(EVoxelPathFollowingResult::InvalidPath);
		return false;
	}

	// 先定下驱动，后面算「起点格/站位高度」要用它（自动竖直偏移依赖移动组件的碰撞柱体）
	ResolveDrive();
	LogCellOffsetOnce();

	// 起点必须是 Agent 当前所在的格：对不上就拒绝 —— 直线走向远端 path[0] 可能切过墙体。
	// 终点合不合法、要不要换终点，一概是调用方的事，这里不判。
	const FIntVector StartCoord = GetCurrentCoord();
	if (Path[0].Coord != StartCoord)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Voxel] 路径起点 (%d,%d,%d) 与 %s 当前所在的格 (%d,%d,%d) 对不上，移动取消（路径要用同一个坐标口径算）"),
			Path[0].Coord.X, Path[0].Coord.Y, Path[0].Coord.Z,
			*GetNameSafe(Owner), StartCoord.X, StartCoord.Y, StartCoord.Z);
		Path.Reset();
		FinishMove(EVoxelPathFollowingResult::InvalidPath);
		return false;
	}

	// 坐标口径自检：Actor 应该就站在「路径起点格心 + 偏移」附近。差出两格以上说明这套坐标对不上
	// （组件的 Terrain 指到了另一块地形 / Actor 根本不在网格上 / 偏移配错了），继续走只会走到无关的地方
	const FVector ExpectedStart = CellLocation(Path[0].Coord);
	const double StartMismatch = FVector::Dist(Owner->GetActorLocation(), ExpectedStart);
	if (StartMismatch > GetVoxelExtent() * 2.0)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[Voxel] %s 在 %s，而路径起点格 %s 的格心在 %s，相差 %.1f（超过两格），移动取消：检查组件的 Terrain 指向与 LocationOffset"),
			*GetNameSafe(Owner), *Owner->GetActorLocation().ToCompactString(), *Path[0].Coord.ToString(),
			*ExpectedStart.ToCompactString(), StartMismatch);
		Path.Reset();
		FinishMove(EVoxelPathFollowingResult::InvalidPath);
		return false;
	}

	// LocationOffset 的水平分量一律忽略：角色必须落在格心上，否则「Actor 位置 ↔ 格坐标」就不是一一对应，
	// 目标点会整体偏出去（偏得多的话就是一路走到地形外面）
	if (FMath::Abs(LocationOffset.X) > 0.01 || FMath::Abs(LocationOffset.Y) > 0.01)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[Voxel] LocationOffset 的水平分量 (%f, %f) 已忽略：它只该用来把角色原点在竖直方向对齐格心，水平偏移会让角色走出格子"),
			LocationOffset.X, LocationOffset.Y);
	}

	// 相邻点的自检（只警告，不改行为）：不带代理类的跳必须同层水平相邻，否则会按直线走过去
	int32 SuspiciousHops = 0;
	FIntVector FirstSuspicious{};
	for (int32 i = 1; i < Path.Num(); ++i)
	{
		if (!Path[i].LinkClass && !IsPlanarHop(Path[i - 1].Coord, Path[i].Coord))
		{
			if (SuspiciousHops++ == 0)
			{
				FirstSuspicious = Path[i].Coord;
			}
		}
	}
	if (SuspiciousHops > 0)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[Voxel] 路径里有 %d 跳既不是同层水平相邻、也没带代理类（第一处在 (%d,%d,%d)），这些跳会按直线走；路径多半不是 FindPath 出来的"),
			SuspiciousHops, FirstSuspicious.X, FirstSuspicious.Y, FirstSuspicious.Z);
	}

	// 占地：先把自己脚下这格登记好，再预约终点（终点已经在自己手上就不用重复预约）
	if (bClaimCells)
	{
		SyncClaimToCurrentCoord();
	}

	if (bLogNavigation)
	{
		UE_LOG(LogTemp, Log, TEXT("[Voxel] %s RequestMove：起点格 %s 终点格 %s 点数 %d 从第 %d 个点开始 驱动 %s 速度 %.0f 站位偏移 %s Actor %s Terrain %s"),
			*GetNameSafe(Owner), *Path[0].Coord.ToString(), *LastResult.GoalCoord.ToString(), Path.Num(), PathIndex + 1,
			GetActiveDrive() == EVoxelMoveDrive::NavMovement ? TEXT("移动组件") : TEXT("直接插值"),
			MoveSpeed, *GetCellOffset().ToCompactString(), *Owner->GetActorLocation().ToCompactString(),
			*GetNameSafe(FoundTerrain));
	}

	Status = EVoxelPathFollowingStatus::Moving;
	UpdateTickState();
	return true;
}

void UVoxelPathFollowingComponent::StopMovement()
{
	if (Status == EVoxelPathFollowingStatus::Idle)
	{
		return;
	}

	FinishMove(EVoxelPathFollowingResult::Aborted);
}

bool UVoxelPathFollowingComponent::PauseMove()
{
	if (Status == EVoxelPathFollowingStatus::Paused)
	{
		return false;		// 已经暂停了
	}
	if (Status == EVoxelPathFollowingStatus::WaitingLink)
	{
		// 这一刻 Agent 归连接代理（跳跃弧线 / 爬梯动画）驱动，暂停会打断「占连接 -> 放行」的握手
		UE_LOG(LogTemp, Warning, TEXT("[Voxel] %s 正在等连接代理放行，无法暂停（要停请让代理停它自己的动作）"),
			*GetNameSafe(GetOwner()));
		return false;
	}
	if (Status != EVoxelPathFollowingStatus::Moving)
	{
		return false;		// 空闲，没什么可暂停的
	}

	// 路径、游标、占地登记一律保留：暂停不是结束，不广播结果
	Status = EVoxelPathFollowingStatus::Paused;
	UpdateTickState();
	StopNavMovement();		// 不清掉速度请求的话，移动组件会带着暂停前那一下的速度继续滑出去
	return true;
}

bool UVoxelPathFollowingComponent::ResumeMove()
{
	if (Status != EVoxelPathFollowingStatus::Paused)
	{
		return false;
	}

	// 不做 UE 那套「暂停期间是否被挪出路径」的判定：单角色回合制，没人会把它挤走
	StallTimer = 0.f;
	StallBestDist = TNumericLimits<float>::Max();
	Status = EVoxelPathFollowingStatus::Moving;
	UpdateTickState();
	return true;
}

/* ===================== 推进 ===================== */

void UVoxelPathFollowingComponent::AdvanceFollowing(float DeltaTime)
{
	if (Status == EVoxelPathFollowingStatus::Idle || Status == EVoxelPathFollowingStatus::Paused || DeltaTime <= 0.f)
	{
		return;
	}

	AActor* Owner = GetOwner();
	AVoxelTerrainActor* FoundTerrain = ResolveTerrain();
	if (!Owner || !FoundTerrain)
	{
		FinishMove(EVoxelPathFollowingResult::NoTerrain);
		return;
	}

	// 连接等待中：等代理（跳跃弧线 / 爬梯动画）做完再放行
	if (Status == EVoxelPathFollowingStatus::WaitingLink)
	{
		LinkWaitElapsed += DeltaTime;
		if (LinkWaitTimeout > 0.f && LinkWaitElapsed >= LinkWaitTimeout)
		{
			UE_LOG(LogTemp, Warning, TEXT("[Voxel] 连接代理超过 %.2f 秒没有放行，移动中止"), LinkWaitTimeout);
			FinishMove(EVoxelPathFollowingResult::Aborted);
			return;
		}
		if (!bLinkResumed)
		{
			return;
		}

		FinishLinkHop();
		if (Status != EVoxelPathFollowingStatus::Moving)
		{
			return;		// 这一跳就是终点，已经在 FinishLinkHop 里收尾了
		}
	}

	// 本帧最多把整条路径走完（MoveSpeed<=0 时就是逐格瞬移）；遇到连接走一段就交接返回
	for (int32 Guard = 0; Guard <= Path.Num() && Status == EVoxelPathFollowingStatus::Moving; ++Guard)
	{
		if (!Path.IsValidIndex(PathIndex))
		{
			FinishMove(EVoxelPathFollowingResult::Success);		// 防御：游标越界按走完处理
			return;
		}

		SyncClaimToCurrentCoord();

		const FVector Target = CellLocation(Path[PathIndex].Coord);
		const FVector Current = Owner->GetActorLocation();
		const float Dist = static_cast<float>(FVector::Dist(Target, Current));

		// 非平面跳：必须由代理把 Agent 挪到目标格（此处一定已经站在这一跳的起点格上）
		if (PathIndex > 0 && Path[PathIndex].LinkClass)
		{
			StartLinkHop();
			return;
		}

		if (HasReachedPathPoint(PathIndex, Current, Target, Dist))
		{
			OnReachedPathPoint(PathIndex);
			++PathIndex;
			if (PathIndex >= Path.Num())
			{
				FinishMove(EVoxelPathFollowingResult::Success);
				return;
			}

			StallTimer = 0.f;
			StallBestDist = TNumericLimits<float>::Max();
			continue;		// 不消耗 dt，接着走下一段
		}

		StepTowardTarget(Target, DeltaTime);
		return;
	}
}

void UVoxelPathFollowingComponent::StepTowardTarget(const FVector& Target, float DeltaTime)
{
	AActor* Owner = GetOwner();
	if (!Owner)
	{
		FinishMove(EVoxelPathFollowingResult::Aborted);
		return;
	}

	const FVector Current = Owner->GetActorLocation();
	const FVector ToTarget = Target - Current;
	const float Dist = static_cast<float>(ToTarget.Size());
	if (Dist <= KINDA_SMALL_NUMBER)
	{
		return;
	}
	const FVector Direction = ToTarget / Dist;

	if (GetActiveDrive() == EVoxelMoveDrive::DirectLocation)
	{
		// MoveSpeed<=0 表示瞬移：这一步直接落到目标点上（下一轮循环就会判定到点）
		const float Speed = MoveSpeed > 0.f ? MoveSpeed : Dist / FMath::Max(DeltaTime, KINDA_SMALL_NUMBER);
		const FVector Next = Current + Direction * FMath::Min(static_cast<double>(Speed) * DeltaTime, static_cast<double>(Dist));
		Owner->SetActorLocation(Next, bSweepWhileMoving);
		FaceDirection(Direction, DeltaTime);
		UpdateStall(Dist, DeltaTime);
		return;
	}

	INavMovementInterface* NavMove = GetNavMovement();
	if (!NavMove)
	{
		FinishMove(EVoxelPathFollowingResult::Aborted);		// 移动组件在移动途中没了
		return;
	}
	if (!HasMovementAuthority())
	{
		// 坠落 / 根运动期间不下发移动。这段时间照样计入卡住计时：角色要是掉出地形，1.5 秒后会以
		// Blocked 结束（而不是永远停在 Moving 里什么也不说）
		UpdateStall(Dist, DeltaTime);
		return;
	}

	// MoveSpeed<=0 时用角色自己的最大速度
	const float MaxSpeed = MoveSpeed > 0.f ? MoveSpeed : FMath::Max(NavMove->GetMaxSpeedForNavMovement(), 1.f);
	const bool bFinalPoint = (PathIndex == Path.Num() - 1);

	// 最后一点才收敛减速；中间的格心保持匀速**穿过去**。
	// 每个格心都按剩余距离降速的话，物理驱动的角色会在格心附近来回蹭（永远差几厘米进不了容差），
	// 而中间格心本来就不需要停 —— 「越过垂足」就算到了（见 HasReachedPathPoint）
	float Speed = bFinalPoint ? FMath::Min(MaxSpeed, Dist / FMath::Max(DeltaTime, KINDA_SMALL_NUMBER)) : MaxSpeed;

	// 末段减速：刹车距离用移动组件自己给的（默认 = MaxSpeed），但最多半格 —— 免得角色提前三个格子就开始挪不动
	if (bFinalPoint)
	{
		const float BrakeDistance = FMath::Min(NavMove->GetPathFollowingBrakingDistance(MaxSpeed), GetVoxelExtent() * 0.5f);
		if (BrakeDistance > KINDA_SMALL_NUMBER && Dist < BrakeDistance)
		{
			Speed *= Dist / BrakeDistance;
		}
	}

	FVector Velocity = Direction * Speed;
	PostProcessMove(Velocity);

	if (NavMove->UseAccelerationForPathFollowing())
	{
		// 加速度模式要归一化输入（大小即强度），末端自然减速
		NavMove->RequestPathMove(Velocity / FMath::Max(MaxSpeed, 1.f));
	}
	else
	{
		// 有意与 UE 的路径跟随不同：UE 传 bForceMaxSpeed 让角色一帧冲到路径点（会冲刺），
		// 这里速度上限始终是 MoveSpeed，参数恒为 false（MoveSpeed 超过角色 MaxWalkSpeed 时由移动组件截断）
		NavMove->RequestDirectMove(Velocity, /*bForceMaxSpeed=*/false);
	}

	FaceDirection(Direction, DeltaTime);
	UpdateStall(Dist, DeltaTime);
}

void UVoxelPathFollowingComponent::UpdateStall(float DistanceToTarget, float DeltaTime)
{
	if (MaxStallTime <= 0.f)
	{
		return;
	}

	if (DistanceToTarget < StallBestDist - StallProgressEpsilon)
	{
		StallBestDist = DistanceToTarget;
		StallTimer = 0.f;
		return;
	}

	StallTimer += DeltaTime;
	if (StallTimer >= MaxStallTime)
	{
		UE_LOG(LogTemp, Warning,
			TEXT("[Voxel] %s 在 %.2f 秒内没能靠近目标点 %s（当前距离 %.1f，%s），按 Blocked 收尾；检查 LocationOffset、碰撞，以及角色是不是掉出地形了"),
			*GetNameSafe(GetOwner()), MaxStallTime,
			Path.IsValidIndex(PathIndex) ? *Path[PathIndex].Coord.ToString() : TEXT("?"), DistanceToTarget,
			HasMovementAuthority() ? TEXT("有移动权限") : TEXT("当前没有移动权限：坠落或根运动中"));
		FinishMove(EVoxelPathFollowingResult::Blocked);
	}
}

void UVoxelPathFollowingComponent::OnReachedPathPoint(int32 Index)
{
	if (!Path.IsValidIndex(Index))
	{
		return;
	}

	AActor* Owner = GetOwner();
	if (bLogNavigation && Owner)
	{
		UE_LOG(LogTemp, Log, TEXT("[Voxel] %s 到达路径点 %d/%d 格 %s（Actor %s）"),
			*GetNameSafe(Owner), Index + 1, Path.Num(), *Path[Index].Coord.ToString(),
			*Owner->GetActorLocation().ToCompactString());
	}

	if (GetActiveDrive() != EVoxelMoveDrive::DirectLocation)
	{
		return;		// 移动组件模式的落点交给物理，不硬拽
	}

	if (Owner)
	{
		Owner->SetActorLocation(CellLocation(Path[Index].Coord), /*bSweep=*/false);
	}
}

/* ===================== 占地 ===================== */

void UVoxelPathFollowingComponent::SyncClaimToCurrentCoord()
{
	if (!bClaimCells || !IsValid(Terrain))
	{
		return;
	}

	const FIntVector Current = GetCurrentCoord();
	if (bHasClaim && Current == ClaimedCoord)
	{
		return;
	}

	if (Terrain->TryOccupyCoord(Current, GetOwner()))
	{
		if (bHasClaim)
		{
			Terrain->ReleaseCoord(ClaimedCoord, GetOwner());
		}
		ClaimedCoord = Current;
		bHasClaim = true;
	}
	else
	{
		// 单人回合制下不该发生。这里**不动**旧登记，免得把别人的记录删掉
		UE_LOG(LogTemp, Warning, TEXT("[Voxel] (%d,%d,%d) 已被别的 Agent 占着，本次占地登记跳过"), Current.X, Current.Y, Current.Z);
	}
}

/* ===================== 连接交接 ===================== */

void UVoxelPathFollowingComponent::StartLinkHop()
{
	AActor* Owner = GetOwner();
	if (!Owner || PathIndex <= 0 || !Path.IsValidIndex(PathIndex) || !IsValid(Terrain))
	{
		FinishMove(EVoxelPathFollowingResult::Aborted);
		return;
	}

	const FVoxelPathPoint& NextPoint = Path[PathIndex];
	ActiveLinkProxy = NewObject<UVoxelNavLinkProxy>(this, NextPoint.LinkClass);
	if (!ActiveLinkProxy)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Voxel] 连接创建失败，移动中止"));
		FinishMove(EVoxelPathFollowingResult::Blocked);
		return;
	}

	Status = EVoxelPathFollowingStatus::WaitingLink;
	bLinkResumed = false;
	LinkWaitElapsed = 0.f;
	UpdateTickState();

	// 代理可能同步放行（默认 UVoxelNavLinkProxy 就是），也可能异步（跳跃弧线 / 爬梯动画）
	ActiveLinkProxy->ReceiveLinkReached(Owner, Terrain, NextPoint.Coord);
	if (bLinkResumed)
	{
		FinishLinkHop();		// 就地收尾，不进递归
	}
}

void UVoxelPathFollowingComponent::FinishLinkHop()
{
	if (!Path.IsValidIndex(PathIndex))
	{
		FinishMove(EVoxelPathFollowingResult::Aborted);
		return;
	}

	ReleaseActiveLink();
	Status = EVoxelPathFollowingStatus::Moving;

	// DirectLocation 自己把 Agent 对齐到目标格心；NavMovement 信任代理的落点，不硬拽物理
	if (GetActiveDrive() == EVoxelMoveDrive::DirectLocation)
	{
		if (AActor* Owner = GetOwner())
		{
			Owner->SetActorLocation(CellLocation(Path[PathIndex].Coord), /*bSweep=*/false);
		}
	}

	++PathIndex;
	SyncClaimToCurrentCoord();
	if (PathIndex >= Path.Num())
	{
		FinishMove(EVoxelPathFollowingResult::Success);
		return;
	}

	StallTimer = 0.f;
	StallBestDist = TNumericLimits<float>::Max();
	UpdateTickState();
}

void UVoxelPathFollowingComponent::ResumeFromLink()
{
	if (Status != EVoxelPathFollowingStatus::WaitingLink)
	{
		return;		// 不是我们在等的连接（已被中止 / 旧代理迟到），忽略
	}

	// 只置位：收尾交给 StartLinkHop 的同步分支或下一次 AdvanceFollowing，避免在代理的调用栈里继续插值
	bLinkResumed = true;
}

void UVoxelPathFollowingComponent::ReleaseActiveLink()
{
	ActiveLinkProxy = nullptr;
	bLinkResumed = false;
}

/* ===================== 收尾 ===================== */

void UVoxelPathFollowingComponent::FinishMove(EVoxelPathFollowingResult Code)
{
	if (Status != EVoxelPathFollowingStatus::Idle)
	{
		ReleaseActiveLink();
		Status = EVoxelPathFollowingStatus::Idle;
		UpdateTickState();

		if (bClaimCells && IsValid(Terrain))
		{
			SyncClaimToCurrentCoord();		// 收尾时把脚下这格登记准（起点格的手续也在这里还回去）
		}

		if (bStopMovementOnFinish)
		{
			StopNavMovement();		// 清掉最后一次速度请求，免得角色停不下来
		}
	}

	LastResult.Code = Code;
	LastResult.PathPoints = Path.Num();
	LastResult.FinalCoord = GetCurrentCoord();
	OnMoveFinished.Broadcast(LastResult);
}
