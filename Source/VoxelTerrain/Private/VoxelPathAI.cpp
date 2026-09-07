// 体素寻路移动封装：像 AI Move To 一样直接可用的
//   ① latent 蓝图异步节点（UVoxelAsyncAction_VoxelMoveTo）
//   ② 行为树任务（UBTTask_VoxelPathMoveTo）
// 共同内核 FVoxelPathMover：拿到寻路结果后逐段下直行移动请求（MoveToLocation bUsePathfinding=false），
// 用位置轮询推进航点，带卡住超时与台阶起跳辅助。全程不依赖 NavMesh。

#include "VoxelPathAI.h"
#include "VoxelPath.h"
#include "VoxelTerrainActor.h"
#include "AIController.h"
#include "Navigation/PathFollowingComponent.h"
#include "BehaviorTree/BehaviorTreeComponent.h"
#include "BehaviorTree/BlackboardComponent.h"
#include "Components/CapsuleComponent.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/PhysicsVolume.h"
#include "GameFramework/WorldSettings.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Containers/Ticker.h"

namespace
{
	/** 脚底位置：取胶囊世界包围盒底（缩放/偏移都天然正确）；无胶囊的退回 Actor 原点 */
	FVector GetActorFeet(const AActor* Actor)
	{
		if (!Actor)
		{
			return FVector::ZeroVector;
		}
		FVector Feet = Actor->GetActorLocation();
		const UCapsuleComponent* Capsule = Actor->FindComponentByClass<UCapsuleComponent>();
		if (Capsule)
		{
			Feet.Z = static_cast<float>(Capsule->Bounds.GetBox().Min.Z);
		}
		return Feet;
	}

	/** 移动节点专用的端点容错：End 对齐时脚底恰好压在格边界，浮点微沉一格就会被 floor() 判成"脚在实体里"。
	 *  先本列向上 4/向下 8 扫，再允许 XY 1 格容错；找不到则返回 false，交给求解器报原始坐标的精确诊断。
	 *  注意：只用于"从世界坐标发起"的移动节点，不改变 FindVoxelPath API 端点必须精确可站的严格语义。 */
	bool SnapMoveEndpoint(const AVoxelTerrainActor& Terrain, const FVector& WorldPos, int32 AgentHeight, FIntVector& OutCell)
	{
		const FIntVector Base = Terrain.WorldLocationToCoord(WorldPos);
		for (int32 Ring = 0; Ring <= 1; ++Ring)
		{
			for (int32 dy = -Ring; dy <= Ring; ++dy)
			{
				for (int32 dx = -Ring; dx <= Ring; ++dx)
				{
					if (Ring > 0 && (FMath::Abs(dx) != Ring && FMath::Abs(dy) != Ring))
					{
						continue;	// 外圈只走切比雪夫距离恰为 Ring 的
					}
					for (int32 z = Base.Z + 4; z >= Base.Z - 8; --z)
					{
						const FIntVector C(Base.X + dx, Base.Y + dy, z);
						// 这里先不查静态网格（逐格 overlap 太贵）；求解器会用完整规则复核，届时失败有精确诊断日志
						if (Terrain.IsVoxelStandable(C, AgentHeight, false))
						{
							OutCell = C;
							return true;
						}
					}
				}
			}
		}
		return false;
	}

	/** 合并共线的中间航点（XY 偏离直线 <5cm 且高度不外凸）：格直线段只留拐点，减少请求次数与停顿 */
	TArray<FVector> SimplifyCollinear(const TArray<FVector>& In)
	{
		const int32 N = In.Num();
		if (N <= 2)
		{
			return In;
		}
		TArray<FVector> Out;
		Out.Reserve(N);
		Out.Add(In[0]);
		for (int32 i = 1; i + 1 < N; ++i)
		{
			const FVector& A = Out.Last();
			const FVector& B = In[i];
			const FVector& C = In[i + 1];
			const FVector2D AB(B.X - A.X, B.Y - A.Y);
			const FVector2D AC(C.X - A.X, C.Y - A.Y);
			const double LenAC = AC.Size();
			if (LenAC < 1.0)
			{
				continue;		// A、C 近乎重合：B 冗余
			}
			const double Deviation = FMath::Abs(AB.X * AC.Y - AB.Y * AC.X) / LenAC;	// 点到直线距离
			const bool bZBetween = B.Z >= FMath::Min(A.Z, C.Z) - 5.0 && B.Z <= FMath::Max(A.Z, C.Z) + 5.0;
			if (Deviation > 5.0 || !bZBetween)
			{
				Out.Add(B);
			}
		}
		Out.Add(In[N - 1]);
		return Out;
	}
	/** 目标 Actor 的落脚格被占据时，退到它周围 4 个正交邻格里离出发者最近的可站格中心。
	 *  Actor 本身可站则返回其脚底；找不到任何可站邻格也返回脚底（交给求解器复核并给诊断）。 */
	FVector ResolveActorDestination(const AVoxelTerrainActor& Terrain, const AActor* Target, const FVector& FromFeet, int32 AgentHeight)
	{
		const FVector TargetFeet = GetActorFeet(Target);
		const FIntVector Center = Terrain.WorldLocationToCoord(TargetFeet);

		// 本列/邻列向上 2、向下 4 找可站格（覆盖"目标站在格边界/陷半格"的常见姿态）
		auto ColumnCell = [&](int32 X, int32 Y, FIntVector& Out)
		{
			for (int32 Z = Center.Z + 2; Z >= Center.Z - 4; --Z)
			{
				const FIntVector C(X, Y, Z);
				if (Terrain.IsVoxelStandable(C, AgentHeight, true))
				{
					Out = C;
					return true;
				}
			}
			return false;
		};

		FIntVector Own;
		if (ColumnCell(Center.X, Center.Y, Own))
		{
			return TargetFeet;		// 目标所在列可站：沿用脚底，Surface 吸附就落在本格
		}

		static const FIntVector Dirs[4] = { {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0} };
		double BestDist = TNumericLimits<double>::Max();
		FVector BestPos = TargetFeet;
		bool bFound = false;
		for (const FIntVector& Dir : Dirs)
		{
			FIntVector Cell;
			if (!ColumnCell(Center.X + Dir.X, Center.Y + Dir.Y, Cell))
			{
				continue;
			}
			const FVector Candidate = Terrain.CoordToWorldLocation(Cell) - FVector(0.0, 0.0, Terrain.GetVoxelSize().Z * 0.5);
			const double Dist = FVector::DistSquared2D(Candidate, FromFeet);
			if (Dist < BestDist)
			{
				BestDist = Dist;
				BestPos = Candidate;
				bFound = true;
			}
		}
		return bFound ? BestPos : TargetFeet;
	}
}

/* ===================== 跟随内核 ===================== */

enum class EVoxelPathMoveState : uint8 { Idle, Running, Succeeded, Failed };

class FVoxelPathMover
{
public:
	/**
	 * @param InWaypoints      跟随折线（优先 ControlPath）
	 * @param InRadius         终点精确到达半径（厘米）
	 * @param InBlockedTimeout 连续无前进的判卡时间（秒）
	 * @param InJumpStepHeight 高差超过此值触发弹道跳（约 0.7 格：一格台阶归跳跃管，落地浮动等噪声无感）
	 * @param InCellSize       体素水平边长（厘米），起跳窗口按 2.2 格取
	 * @param InStepAssistHeight 跟随期间临时抬高的 MaxStepHeight（约 0.55 格：只用来平滑消化棱线噪声，
	 *                           绝不用于跨整格台阶——CMC 的 step-up 是单帧位移，一格高就是“闪现”）
	 */
	bool Start(AAIController* InController, const TArray<FVector>& InWaypoints, float InRadius, float InBlockedTimeout, float InJumpStepHeight, float InCellSize, float InStepAssistHeight, bool bInJumpAssist, bool bInSmoothTurning = true)
	{
		Controller = InController;
		Waypoints = SimplifyCollinear(InWaypoints);
		Radius = FMath::Max(InRadius, 10.f);
		RadiusSq = Radius * Radius;
		// 中间航点用更大的"过弯半径"：在拐角前减速但不完全停死，消除逐格"到位→刹停→换向→再加速"的忽快忽慢；
		// 最后一段仍用精确到达半径，保证停在请求的目标点上
		CornerRadius = FMath::Max(Radius * 1.8f, InJumpStepHeight * 1.5f);
		CornerRadiusSq = static_cast<double>(CornerRadius) * CornerRadius;
		// 起跳窗口取 2.2 格：保证触发时离立面 ≥1.5 格，抛物线有真实前向速度、顶点跨过棱线；
		// 窗口太窄（贴脸触发）就会退化成“原地干蹦”
		LaunchWindow = FMath::Max(static_cast<double>(InCellSize) * 2.2, static_cast<double>(CornerRadius));
		CellSizeXY = FMath::Max(10.f, InCellSize);
		StepAssistHeight = bInJumpAssist ? InStepAssistHeight : 0.f;	// 关闭辅助时不动角色的 MaxStepHeight
		BlockedTimeout = FMath::Max(InBlockedTimeout, 0.5f);
		JumpStepHeight = InJumpStepHeight;
		bJumpAssist = bInJumpAssist;
		Index = 0;
		BlockedAccum = 0.f;
		SegmentAge = 0.f;
		MinDistToTarget = TNumericLimits<float>::Max();
		Result = EVoxelPathMoveResult::Success;

		APawn* Pawn = InController ? InController->GetPawn() : nullptr;
		if (!Pawn || Waypoints.IsEmpty())
		{
			State = EVoxelPathMoveState::Failed;
			Result = Pawn ? EVoxelPathMoveResult::NoPath : EVoxelPathMoveResult::NoController;
			return false;
		}
		LastFeet = GetActorFeet(Pawn);
		State = EVoxelPathMoveState::Running;
		ApplyMovementOverrides(Pawn, bInSmoothTurning);
		SkipReachedWaypoints(LastFeet);		// 吃掉出发时已在半径内的航点
		if (State != EVoxelPathMoveState::Running)
		{
			RestoreMovementOverrides();			// 出发即达/异常：本就没有移动，恢复原设置
			return State == EVoxelPathMoveState::Succeeded;
		}
		if (!IssueSegment())
		{
			RestoreMovementOverrides();
			return false;
		}
		return true;
	}

	EVoxelPathMoveState Tick(float DeltaSeconds)
	{
		const EVoxelPathMoveState NewState = TickInternal(DeltaSeconds);
		if (NewState != EVoxelPathMoveState::Running)
		{
			RestoreMovementOverrides();
		}
		return NewState;
	}

	~FVoxelPathMover()
	{
		RestoreMovementOverrides();		// 兜底：任何路径销毁都还原角色的旋转设置
	}

	EVoxelPathMoveState TickInternal(float DeltaSeconds)
	{
		if (State != EVoxelPathMoveState::Running)
		{
			return State;
		}
		AAIController* C = Controller.Get();
		APawn* Pawn = C ? C->GetPawn() : nullptr;
		if (!Pawn)
		{
			State = EVoxelPathMoveState::Failed;
			Result = EVoxelPathMoveResult::NoController;
			return State;
		}
		const FVector Feet = GetActorFeet(Pawn);

		if (SkipReachedWaypoints(Feet))
		{
			if (State != EVoxelPathMoveState::Running)
			{
				return State;
			}
			if (!IssueSegment())
			{
				return State;
			}
		}

		// 进度检测：距离目标点没再明显拉近就累计卡住时间
		const float DistToTarget = static_cast<float>(FVector::Dist2D(Feet, Waypoints[Index]));
		if (DistToTarget < MinDistToTarget - 5.0)
		{
			MinDistToTarget = DistToTarget;
			BlockedAccum = 0.f;
			if (DistToTarget > LaunchWindow)
			{
				JumpAttempts = 0;		// 还在正常接近段（比如已落在两级台阶之间的平台）：重置升级序列
			}
		}
		else
		{
			BlockedAccum += DeltaSeconds;
		}
		if (BlockedAccum >= BlockedTimeout)
		{
			State = EVoxelPathMoveState::Failed;
			Result = EVoxelPathMoveResult::Blocked;
			return State;
		}

		// 引擎侧意外转入 Idle（被外部 StopMovement 等）：隔 0.5s 补发一次当前段请求
		SegmentAge += DeltaSeconds;
		if (SegmentAge > 0.5f && C->GetMoveStatus() == EPathFollowingStatus::Idle)
		{
			IssueSegment();
		}

		// 台阶跳跃（弹道驱动）：见 HandleStepJump
		if (bJumpAssist)
		{
			if (ACharacter* Character = Cast<ACharacter>(Pawn))
			{
				if (UCharacterMovementComponent* Move = Character->GetCharacterMovement())
				{
					HandleStepJump(Character, Move, Feet, DistToTarget);
				}
			}
		}
		LastFeet = Feet;
		return State;
	}

	/* 静默终止：停下移动并进入 Idle（调用方负责给出自己的结果，如 BT 的 Aborted） */
	void Abort()
	{
		RestoreMovementOverrides();
		if (State == EVoxelPathMoveState::Running)
		{
			if (AAIController* C = Controller.Get())
			{
				C->StopMovement();
			}
		}
		State = EVoxelPathMoveState::Idle;
		Result = EVoxelPathMoveResult::Aborted;
		Waypoints.Empty();
		Index = 0;
	}

	bool IsRunning() const { return State == EVoxelPathMoveState::Running; }
	EVoxelPathMoveResult GetResult() const { return Result; }
	FVector GetLastLocation() const { return LastFeet; }

private:
	/* ---------- 移动期组件参数覆盖（转向平滑 + 台阶辅助），结束/销毁时还原 ---------- */

	/** APawn 默认 bUseControllerRotationYaw=true：AIController 路径跟随每帧直接改写控制旋转，
	 *  pawn“瞬间转头”、角色自己的 RotationRate 完全不生效。跟随期间切到 CharacterMovement 的
	 *  “朝移动方向朝向”（由 RotationRate 插值）。
	 *  台阶噪声辅助：MaxStepHeight 临时抬到 0.55 格，只用于平滑消化棱线/落地浮动；
	 *  整格台阶走弹道跳——StepUp 是单帧位移，抬到一格高视觉上是“闪现”。 */
	static UCharacterMovementComponent* GetCharMove(APawn* Pawn)
	{
		const ACharacter* Character = Cast<ACharacter>(Pawn);
		return Character ? const_cast<UCharacterMovementComponent*>(Character->GetCharacterMovement()) : nullptr;
	}

	void ApplyMovementOverrides(APawn* Pawn, bool bSmoothTurning)
	{
		if (bOverridesApplied || !Pawn)
		{
			return;
		}
		UCharacterMovementComponent* Move = GetCharMove(Pawn);
		if (!Move)
		{
			return;		// 非 Character：不动它的移动设置
		}
		OverridePawn = Pawn;
		bSavedUseControllerYaw = Pawn->bUseControllerRotationYaw;
		bSavedOrientToMovement = Move->bOrientRotationToMovement;
		SavedMaxStepHeight = Move->MaxStepHeight;
		if (bSmoothTurning)
		{
			Pawn->bUseControllerRotationYaw = false;
			Move->bOrientRotationToMovement = true;	// 转身速度 = 角色的 RotationRate，蓝图可调
		}
		Move->MaxStepHeight = FMath::Max(SavedMaxStepHeight, StepAssistHeight);
		bOverridesApplied = true;
	}

	void RestoreMovementOverrides()
	{
		if (!bOverridesApplied)
		{
			return;
		}
		bOverridesApplied = false;
		APawn* Pawn = OverridePawn.Get();
		UCharacterMovementComponent* Move = GetCharMove(Pawn);
		if (Pawn)
		{
			Pawn->bUseControllerRotationYaw = bSavedUseControllerYaw;
		}
		if (Move)
		{
			Move->bOrientRotationToMovement = bSavedOrientToMovement;
			Move->MaxStepHeight = SavedMaxStepHeight;
		}
		OverridePawn.Reset();
	}

	TWeakObjectPtr<APawn> OverridePawn;
	bool bSavedUseControllerYaw = true;
	bool bSavedOrientToMovement = false;
	float SavedMaxStepHeight = 0.f;
	bool bOverridesApplied = false;

	/* ---------- 台阶通过策略（阈值分层）----------
	 * ≤0.55 格（棱线噪声/落地浮动）：跟随期间把 MaxStepHeight 临时抬到 0.55 格，正常行走平滑消化——
	 *   注意绝不让它覆盖整格：CMC 的 StepUp 是单帧位移，一格 100cm 抬上去视觉效果就是“闪现”。
	 * >0.7 格（一格台阶 100cm、两格台 200cm）：弹道跳。
	 *   时机：距目标 ≤2.2 格（起跳点离立面 ≥1.5 格，保证水平初速度），停滞贴墙时窗口放宽 1.6 倍兜底；
	 *   解算：vz=√(2g·(Δz+25cm+12%·Δz)) 使顶点高于棱线，vx=立面距离/上升时间（XY 覆盖式给定——
	 *   空中引擎输入会被弱化、撞墙水平速度被碰撞清零，叠加式给法保不住弹道）；
	 *   失败按尝试次数加码（每次 +25%，上限 ×2.0），仍不行按 Blocked 失败交给行为树换路。
	 */

	/** 生效重力(cm/s², 取正)：物理体积 → 世界默认 → 移动组件缓存，再乘角色的 GravityScale */
	double ResolveGravity(const APawn* Pawn, const UCharacterMovementComponent* Move) const
	{
		double Gravity = 0.0;
		if (const APhysicsVolume* Volume = Pawn->GetPhysicsVolume())
		{
			Gravity = FMath::Abs(static_cast<double>(Volume->GetGravityZ()));
		}
		if (Gravity <= 1.0 && IsValid(Pawn->GetWorld()) && Pawn->GetWorld()->GetWorldSettings())
		{
			Gravity = FMath::Abs(static_cast<double>(Pawn->GetWorld()->GetWorldSettings()->GetGravityZ()));
		}
		if (Gravity <= 1.0)
		{
			Gravity = FMath::Abs(static_cast<double>(Move->GetGravityZ()));
		}
		if (Gravity <= 1.0)
		{
			Gravity = 1000.0;		// 兜底用引擎默认 1G，别让除零/负值毁掉弹道计算
		}
		return Gravity * FMath::Max(0.25, static_cast<double>(Move->GravityScale));
	}

	void HandleStepJump(ACharacter* Character, UCharacterMovementComponent* Move, const FVector& Feet, float DistToTarget)
	{
		const double DeltaZ = Waypoints[Index].Z - Feet.Z;
		if (DeltaZ <= JumpStepHeight)
		{
			JumpAttempts = 0;		// ≤0.7 格：交给 MaxStepHeight(0.55 格)+正常行走消化，8cm 浮动等噪声根本到不了这里
			return;
		}
		// 时机：① 已进入起跳窗口（≈2.2 格，起跳点离立面 ≥1.5 格，抛物线有真实前向速度）；
		// ② 停滞且仍在窗口 1.6 倍内（兜底）。必须在落地状态，空中不重复发跳。
		const bool bInLaunchWindow = DistToTarget <= LaunchWindow;
		const bool bStalledNearWall = BlockedAccum > 0.15f && DistToTarget <= LaunchWindow * 1.6;
		if ((!bInLaunchWindow && !bStalledNearWall) || !Move->IsMovingOnGround())
		{
			return;
		}

		const double Gravity = ResolveGravity(Character, Move);
		const double Margin = 25.0 + DeltaZ * 0.12;
		const double ApexHeight = DeltaZ + Margin;			// 顶点要高于棱线并留擦碰余量
		double Vz = FMath::Sqrt(2.0 * Gravity * ApexHeight);
		Vz *= FMath::Min(1.0 + 0.25 * JumpAttempts, 2.0);	// 失败逐级加码
		const double RiseTime = Vz / Gravity;				// 起跳到顶点的时间

		FVector Launch(0.0, 0.0, Vz);
		bool bOverrideXY = false;
		if (DistToTarget > 1.0)
		{
			// 弹道条件取“顶点恰好出现在跨过立面的瞬间”：水平位移 = 立面距离时正好到达顶点。
			// 水平速度用覆盖而非叠加：空中 PathFollowing 输入被弱化、撞墙又被碰撞逐帧清零，
			// 叠加既超速又保不住弹道，覆盖才把“顶点跨棱”的解算真正执行出来。
			const double WallDist = FMath::Max(static_cast<double>(DistToTarget) - static_cast<double>(CellSizeXY) * 0.6, 5.0);
			const double Vx = FMath::Clamp(WallDist / RiseTime, 80.0, static_cast<double>(Move->MaxWalkSpeed) * 1.05);
			const FVector2D Dir = FVector2D(Waypoints[Index].X - Feet.X, Waypoints[Index].Y - Feet.Y).GetSafeNormal();
			Launch.X = Dir.X * Vx;
			Launch.Y = Dir.Y * Vx;
			bOverrideXY = true;
		}
		Character->LaunchCharacter(Launch, /*bXYOverride*/ bOverrideXY, /*bZOverride*/ true);
		++JumpAttempts;
		BlockedAccum = 0.f;
		EnsureDrivingRequest();
	}

	/* 起跳瞬间若引擎的移动请求已经结束（贴墙被判 Blocked / Idle），立即重发当前段请求：
	   空中继续保持朝目标点的移动输入，避免“原地起跳、落地还原地”。 */
	void EnsureDrivingRequest()
	{
		const AAIController* C = Controller.Get();
		if (C && C->GetMoveStatus() != EPathFollowingStatus::Moving)
		{
			IssueSegment();
		}
	}

	int32 JumpAttempts = 0;
	double LaunchWindow = 200.0;
	float StepAssistHeight = 55.f;	// 跟随期间的 MaxStepHeight 抬升目标（≈0.55 格，只消化噪声；整格交给弹道跳）
	float CellSizeXY = 100.f;		// 体素水平边长（立面距离估算用）

	/* 吃掉所有已在到达半径内的航点；返回是否发生了推进（推进后需要重新发请求） */
	bool SkipReachedWaypoints(const FVector& Feet)
	{
		bool bAdvanced = false;
		while (Index < Waypoints.Num())
		{
			const bool bIsLast = (Index == Waypoints.Num() - 1);
			// “向上台阶”的落脚点不许用过弯半径跳过——必须真的站上去才放行，
			// 否则角色会从台阶前掠过、跳不起来，还会一路“跳过”楼梯
			const bool bStepUp = Waypoints[Index].Z - Feet.Z > JumpStepHeight;
			const double AllowedSq = (bIsLast || bStepUp) ? RadiusSq : CornerRadiusSq;
			if (FVector::DistSquared2D(Feet, Waypoints[Index]) > AllowedSq)
			{
				break;
			}
			++Index;
			bAdvanced = true;
		}
		if (Index >= Waypoints.Num())
		{
			State = EVoxelPathMoveState::Succeeded;
			Result = EVoxelPathMoveResult::Success;
			return bAdvanced;
		}
		if (bAdvanced)
		{
			BlockedAccum = 0.f;
			JumpAttempts = 0;
			MinDistToTarget = TNumericLimits<float>::Max();
		}
		return bAdvanced;
	}

	bool IssueSegment()
	{
		AAIController* C = Controller.Get();
		if (!C)
		{
			State = EVoxelPathMoveState::Failed;
			Result = EVoxelPathMoveResult::NoController;
			return false;
		}
		if (Index >= Waypoints.Num())
		{
			State = EVoxelPathMoveState::Succeeded;
			return true;
		}
		// bUsePathfinding=false：直线移动请求，不查 NavMesh；航点间可走性已由体素寻路保证。
		// 中间角点用过弯半径（提前转入下一段、不停死），最后一段用精确半径停在目标上
		const float SegmentRadius = (Index == Waypoints.Num() - 1) ? Radius : CornerRadius;
		const EPathFollowingRequestResult::Type RequestResult =
			C->MoveToLocation(Waypoints[Index], SegmentRadius, /*bStopOnOverlap*/ false, /*bUsePathfinding*/ false,
				/*bProjectDestinationToNavigation*/ false, /*bCanStrafe*/ true);
		SegmentAge = 0.f;
		if (RequestResult == EPathFollowingRequestResult::Failed)
		{
			State = EVoxelPathMoveState::Failed;
			Result = EVoxelPathMoveResult::Blocked;
			return false;
		}
		if (RequestResult == EPathFollowingRequestResult::AlreadyAtGoal)
		{
			const APawn* CurrentPawn = C ? C->GetPawn() : nullptr;
			LastFeet = GetActorFeet(CurrentPawn);
			if (SkipReachedWaypoints(LastFeet) && State == EVoxelPathMoveState::Running)
			{
				return IssueSegment();
			}
		}
		return State != EVoxelPathMoveState::Failed;
	}

	TWeakObjectPtr<AAIController> Controller;
	TArray<FVector> Waypoints;
	int32 Index = 0;
	float Radius = 50.f;
	double RadiusSq = 2500.0;
	float CornerRadius = 90.f;			// 中间航点过弯半径（Start 里按到达半径/跳跃高差推大）
	double CornerRadiusSq = 8100.0;
	float BlockedTimeout = 3.f;
	float BlockedAccum = 0.f;
	float SegmentAge = 0.f;
	float MinDistToTarget = TNumericLimits<float>::Max();
	float JumpStepHeight = 60.f;
	bool bJumpAssist = true;
	FVector LastFeet = FVector::ZeroVector;
	EVoxelPathMoveState State = EVoxelPathMoveState::Idle;
	EVoxelPathMoveResult Result = EVoxelPathMoveResult::Success;
};

/* ===================== 公共小工具 ===================== */

namespace
{
	AVoxelTerrainActor* ResolveVoxelTerrain(const UWorld* World, AVoxelTerrainActor* Specified)
	{
		if (IsValid(Specified))
		{
			return Specified;
		}
		if (World)
		{
			for (TActorIterator<AVoxelTerrainActor> It(World); It; ++It)
			{
				return *It;		// 约定：多地形场景请显式传 Terrain
			}
		}
		return nullptr;
	}

	EVoxelPathMoveResult MapPathFailure(EVoxelPathResult PathResult)
	{
		switch (PathResult)
		{
		case EVoxelPathResult::Found:          return EVoxelPathMoveResult::Success;
		case EVoxelPathResult::StartInvalid:   return EVoxelPathMoveResult::StartInvalid;
		case EVoxelPathResult::GoalInvalid:    return EVoxelPathMoveResult::GoalInvalid;
		case EVoxelPathResult::BudgetExceeded: return EVoxelPathMoveResult::BudgetExceeded;
		case EVoxelPathResult::AreaTooLarge:   return EVoxelPathMoveResult::AreaTooLarge;
		case EVoxelPathResult::NoPath:
		default:                               return EVoxelPathMoveResult::NoPath;
		}
	}

	/* 从棋子脚底算到目的地的寻路结果；OutFollowPath 取稀疏控制折线 */
	bool ComputeMovePath(const AVoxelTerrainActor& Terrain, const FVoxelPathParams& Params, bool bSurfacePath,
		const FVector& FromFeet, const FVector& Destination, FVoxelPathResult& OutPath, const TArray<FVector>** OutFollowPath)
	{
		if (bSurfacePath)
		{
			OutPath = Terrain.FindSurfacePath(FromFeet, Destination, Params);
		}
		else
		{
			// Grid 模式：端点先做容错吸附（End 对齐时脚底恰好压在格边界，直接 floor 会落进实体格）；
			// 吸附失败仍传原始坐标进求解器，让它打出带原因的精确诊断
			FIntVector StartCell = Terrain.WorldLocationToCoord(FromFeet);
			FIntVector GoalCell = Terrain.WorldLocationToCoord(Destination);
			SnapMoveEndpoint(Terrain, FromFeet, Params.AgentHeight, StartCell);
			SnapMoveEndpoint(Terrain, Destination, Params.AgentHeight, GoalCell);
			OutPath = Terrain.FindVoxelPath(StartCell, GoalCell, Params);
		}
		if (OutPath.Result != EVoxelPathResult::Found)
		{
			return false;
		}
		*OutFollowPath = OutPath.ControlPath.IsEmpty() ? &OutPath.WorldPath : &OutPath.ControlPath;
		return true;
	}
}

/* ===================== latent 蓝图节点 ===================== */

UVoxelAsyncAction_VoxelMoveTo* UVoxelAsyncAction_VoxelMoveTo::VoxelMoveToLocation(UObject* WorldContextObject, AAIController* Controller,
	const FVector& Destination, AVoxelTerrainActor* Terrain, const FVoxelPathParams& Params, bool bSurfacePath, float AcceptanceRadius, float BlockedTimeout, bool bSmoothTurning)
{
	UVoxelAsyncAction_VoxelMoveTo* Node = NewObject<UVoxelAsyncAction_VoxelMoveTo>();
	Node->WorldContextRef = WorldContextObject;
	Node->ControllerRef = Controller;
	Node->TerrainRef = Terrain;
	Node->Destination = Destination;
	Node->Params = Params;
	Node->bSurfacePath = bSurfacePath;
	Node->bSmoothTurning = bSmoothTurning;
	Node->AcceptanceRadius = AcceptanceRadius;
	Node->BlockedTimeout = BlockedTimeout;
	return Node;
}

UVoxelAsyncAction_VoxelMoveTo* UVoxelAsyncAction_VoxelMoveTo::VoxelMoveToActor(UObject* WorldContextObject, AAIController* Controller,
	AActor* TargetActor, AVoxelTerrainActor* Terrain, const FVoxelPathParams& Params, bool bSurfacePath, float AcceptanceRadius, float BlockedTimeout, bool bSmoothTurning)
{
	UVoxelAsyncAction_VoxelMoveTo* Node = VoxelMoveToLocation(WorldContextObject, Controller, FVector::ZeroVector, Terrain, Params, bSurfacePath, AcceptanceRadius, BlockedTimeout, bSmoothTurning);
	Node->TargetRef = TargetActor;
	return Node;
}

void UVoxelAsyncAction_VoxelMoveTo::Activate()
{
	RegisterWithGameInstance(WorldContextRef.Get());
	BeginMove();		// 失败会同步广播（委托在 Activate 前已由节点绑定，安全）
}

void UVoxelAsyncAction_VoxelMoveTo::BeginMove()
{
	const UWorld* World = GEngine ? GEngine->GetWorldFromContextObject(WorldContextRef.Get(), EGetWorldErrorMode::ReturnNull) : nullptr;
	AAIController* Controller = ControllerRef.Get();
	APawn* Pawn = Controller ? Controller->GetPawn() : nullptr;
	if (!World || !Pawn)
	{
		Finish(EVoxelPathMoveResult::NoController);
		return;
	}
	AVoxelTerrainActor* Terrain = ResolveVoxelTerrain(World, TerrainRef.Get());
	if (!Terrain)
	{
		Finish(EVoxelPathMoveResult::TerrainNotFound);
		return;
	}
	FVector TargetLocation = Destination;
	if (TargetRef.IsValid())
	{
		const AActor* Target = TargetRef.Get();
		if (!Target)
		{
			Finish(EVoxelPathMoveResult::DestinationInvalid);
			return;
		}
		// 目标 Actor 所在格被占据时，退到其四正交邻格中离自己最近的可站格
		TargetLocation = ResolveActorDestination(*Terrain, Target, GetActorFeet(Pawn), Params.AgentHeight);
	}

	FVoxelPathResult PathResult;
	const TArray<FVector>* FollowPath = nullptr;
	if (!ComputeMovePath(*Terrain, Params, bSurfacePath, GetActorFeet(Pawn), TargetLocation, PathResult, &FollowPath))
	{
		UE_LOG(LogTemp, Warning, TEXT("VoxelMoveTo：寻路失败（%d），目标 %s"), static_cast<int32>(PathResult.Result), *TargetLocation.ToString());
		Finish(MapPathFailure(PathResult.Result));
		return;
	}

	const FVector Size = Terrain->GetVoxelSize();
	const float Radius = AcceptanceRadius > 0.f ? AcceptanceRadius : static_cast<float>(FMath::Min(Size.X, Size.Y) * 0.5);
	Mover = MakeShared<FVoxelPathMover>();
	if (!Mover->Start(Controller, *FollowPath, Radius, BlockedTimeout, static_cast<float>(Size.Z * 0.7), static_cast<float>(FMath::Min(Size.X, Size.Y)), static_cast<float>(Size.Z * 0.55), true, bSmoothTurning))
	{
		Finish(Mover->GetResult() == EVoxelPathMoveResult::Success ? EVoxelPathMoveResult::NoPath : Mover->GetResult());
		return;
	}
	if (!Mover->IsRunning())
	{
		Finish(Mover->GetResult());		// 出发即达
		return;
	}

	TickerHandle = FTSTicker::GetCoreTicker().AddTicker(
		FTickerDelegate::CreateLambda([Weak = TWeakObjectPtr<UVoxelAsyncAction_VoxelMoveTo>(this)](float DeltaTime)
		{
			UVoxelAsyncAction_VoxelMoveTo* Self = Weak.Get();
			if (!Self || !Self->Mover.IsValid())
			{
				return false;
			}
			const EVoxelPathMoveState State = Self->Mover->Tick(DeltaTime);
			if (State == EVoxelPathMoveState::Idle)
			{
				Self->Finish(EVoxelPathMoveResult::Aborted);
			}
			else if (State != EVoxelPathMoveState::Running)
			{
				Self->Finish(State == EVoxelPathMoveState::Succeeded ? EVoxelPathMoveResult::Success : Self->Mover->GetResult());
			}
			return Self->Mover.IsValid();
		}), 0.f);
}

void UVoxelAsyncAction_VoxelMoveTo::Finish(EVoxelPathMoveResult Result)
{
	if (TickerHandle.IsValid())
	{
		FTSTicker::GetCoreTicker().RemoveTicker(TickerHandle);
		TickerHandle.Reset();
	}
	FVector EndLocation = Destination;
	if (Mover.IsValid())
	{
		EndLocation = Mover->GetLastLocation();
		Mover->Abort();		// 失败路径下确保停步；成功后内部已是 Idle，不会重复 Stop
		Mover.Reset();
	}
	OnFinished.Broadcast(Result, EndLocation);
	SetReadyToDestroy();
}

/* ===================== 行为树任务 ===================== */

/* 跟随器堆分配放进 NodeMemory（结构体仅持一个裸指针，与引擎 FBTMoveToTaskMemory 同为平凡结构） */
struct FBTVoxelPathMoveToTaskMemory
{
	FVoxelPathMover* Mover = nullptr;
};

UBTTask_VoxelPathMoveTo::UBTTask_VoxelPathMoveTo(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	NodeName = "Voxel Path Move To";
	INIT_TASK_NODE_NOTIFY_FLAGS();
	// 引擎惯例：用 Add*Filter 注册编辑器下拉框里可选的黑板键类型（目的地支持 Vector 或 Actor）
	BlackboardKey.AddVectorFilter(this, GET_MEMBER_NAME_CHECKED(UBTTask_VoxelPathMoveTo, BlackboardKey));
	BlackboardKey.AddObjectFilter(this, GET_MEMBER_NAME_CHECKED(UBTTask_VoxelPathMoveTo, BlackboardKey), AActor::StaticClass());
}

EBTNodeResult::Type UBTTask_VoxelPathMoveTo::ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	FBTVoxelPathMoveToTaskMemory* Memory = CastInstanceNodeMemory<FBTVoxelPathMoveToTaskMemory>(NodeMemory);
	if (Memory->Mover)
	{
		delete Memory->Mover;
		Memory->Mover = nullptr;
	}

	AAIController* Controller = OwnerComp.GetAIOwner();
	APawn* Pawn = Controller ? Controller->GetPawn() : nullptr;
	if (!Pawn)
	{
		UE_LOG(LogTemp, Warning, TEXT("VoxelPathMoveTo：Agent 没有 AIController/Pawn"));
		return EBTNodeResult::Failed;
	}

	UBlackboardComponent* Blackboard = OwnerComp.GetBlackboardComponent();
	FVector Destination;
	if (!Blackboard || !Blackboard->GetLocationFromEntry(BlackboardKey.GetSelectedKeyID(), Destination))
	{
		UE_LOG(LogTemp, Warning, TEXT("VoxelPathMoveTo：黑板键 '%s' 没有有效的目的地（Vector/Actor）"), *BlackboardKey.SelectedKeyName.ToString());
		return EBTNodeResult::Failed;
	}

	const UWorld* World = OwnerComp.GetWorld();
	AVoxelTerrainActor* TerrainActor = ResolveVoxelTerrain(World, Terrain);
	if (!TerrainActor)
	{
		UE_LOG(LogTemp, Warning, TEXT("VoxelPathMoveTo：场景里没有 AVoxelTerrainActor（或没指定 Terrain）"));
		return EBTNodeResult::Failed;
	}

	FVoxelPathResult PathResult;
	const TArray<FVector>* FollowPath = nullptr;
	if (!ComputeMovePath(*TerrainActor, PathParams, bSurfacePath, GetActorFeet(Pawn), Destination, PathResult, &FollowPath))
	{
		UE_LOG(LogTemp, Verbose, TEXT("VoxelPathMoveTo：寻路失败（%d），目标 %s"), static_cast<int32>(PathResult.Result), *Destination.ToString());
		return EBTNodeResult::Failed;
	}

	const FVector Size = TerrainActor->GetVoxelSize();
	const float Radius = AcceptanceRadius > 0.f ? AcceptanceRadius : static_cast<float>(FMath::Min(Size.X, Size.Y) * 0.5);
	Memory->Mover = new FVoxelPathMover();
	if (!Memory->Mover->Start(Controller, *FollowPath, Radius, BlockedTimeout, static_cast<float>(Size.Z * 0.7), static_cast<float>(FMath::Min(Size.X, Size.Y)), static_cast<float>(Size.Z * 0.55), bJumpAssist, bSmoothTurning))
	{
		UE_LOG(LogTemp, Warning, TEXT("VoxelPathMoveTo：无法开始跟随（%d）"), static_cast<int32>(Memory->Mover->GetResult()));
		delete Memory->Mover;
		Memory->Mover = nullptr;
		return EBTNodeResult::Failed;
	}
	if (!Memory->Mover->IsRunning())
	{
		// 出发即达
		delete Memory->Mover;
		Memory->Mover = nullptr;
		return EBTNodeResult::Succeeded;
	}
	return EBTNodeResult::InProgress;
}

void UBTTask_VoxelPathMoveTo::TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds)
{
	FBTVoxelPathMoveToTaskMemory* Memory = CastInstanceNodeMemory<FBTVoxelPathMoveToTaskMemory>(NodeMemory);
	if (!Memory->Mover)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
		return;
	}
	const EVoxelPathMoveState State = Memory->Mover->Tick(DeltaSeconds);
	if (State == EVoxelPathMoveState::Running)
	{
		return;
	}
	if (State == EVoxelPathMoveState::Succeeded)
	{
		FinishLatentTask(OwnerComp, EBTNodeResult::Succeeded);
		return;
	}
	UE_LOG(LogTemp, Verbose, TEXT("VoxelPathMoveTo：移动未完成（%d）"), static_cast<int32>(Memory->Mover->GetResult()));
	FinishLatentTask(OwnerComp, EBTNodeResult::Failed);
}

EBTNodeResult::Type UBTTask_VoxelPathMoveTo::AbortTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory)
{
	FBTVoxelPathMoveToTaskMemory* Memory = CastInstanceNodeMemory<FBTVoxelPathMoveToTaskMemory>(NodeMemory);
	if (Memory->Mover)
	{
		Memory->Mover->Abort();
		delete Memory->Mover;
		Memory->Mover = nullptr;
	}
	return EBTNodeResult::Aborted;
}

uint16 UBTTask_VoxelPathMoveTo::GetInstanceMemorySize() const
{
	return sizeof(FBTVoxelPathMoveToTaskMemory);
}

void UBTTask_VoxelPathMoveTo::InitializeMemory(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, EBTMemoryInit::Type InitType) const
{
	InitializeNodeMemory<FBTVoxelPathMoveToTaskMemory>(NodeMemory, InitType);
}

void UBTTask_VoxelPathMoveTo::CleanupMemory(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, EBTMemoryClear::Type CleanupType) const
{
	FBTVoxelPathMoveToTaskMemory* Memory = CastInstanceNodeMemory<FBTVoxelPathMoveToTaskMemory>(NodeMemory);
	if (Memory && Memory->Mover)
	{
		delete Memory->Mover;
		Memory->Mover = nullptr;
	}
	CleanupNodeMemory<FBTVoxelPathMoveToTaskMemory>(NodeMemory, CleanupType);
}

#if WITH_EDITOR
FString UBTTask_VoxelPathMoveTo::GetStaticDescription() const
{
	const FString Mode = bSurfacePath ? TEXT("表面自由") : TEXT("严格格路径");
	const FString Extra = bJumpAssist ? TEXT("，带起跳辅助") : FString();
	return FString::Printf(TEXT("沿体素地形（%s）移动到黑板键 '%s'%s"), *Mode, *BlackboardKey.SelectedKeyName.ToString(), *Extra);
}
#endif // WITH_EDITOR
