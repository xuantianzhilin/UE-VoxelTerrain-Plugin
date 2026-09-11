#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "VoxelNavLinkProxy.h"
#include "VoxelPathFollowingComponent.generated.h"

class UVoxelNavLinkProxy;
class AVoxelTerrainActor;
class INavMovementInterface;

USTRUCT(BlueprintType)
struct FVoxelPathPoint
{
	GENERATED_BODY()

	FVoxelPathPoint() = default;
	FVoxelPathPoint(FIntVector InCoord) : Coord(MoveTemp(InCoord)) {}
	FVoxelPathPoint(FIntVector InCoord, TSubclassOf<UVoxelNavLinkProxy> Class)
		: Coord(MoveTemp(InCoord))
		, LinkClass(Class)
	{}

	UPROPERTY()
	FIntVector Coord = FIntVector::ZeroValue;
	UPROPERTY()
	TSubclassOf<UVoxelNavLinkProxy> LinkClass;

	bool operator==(const FVoxelPathPoint& Other) const
	{
		return Coord == Other.Coord && LinkClass == Other.LinkClass;
	}
};

FORCEINLINE uint32 GetTypeHash(const FVoxelPathPoint& Key)
{
	return HashCombine(GetTypeHash(Key.Coord), GetTypeHash(Key.LinkClass));
}

/**
 * 一次移动的最终结果。
 * 路径合不合法是调用方的事，所以这里只有「走成了没有」和「为什么没走成」，没有「终点站不住」这类码。
 */
UENUM(BlueprintType)
enum class EVoxelPathFollowingResult : uint8
{
	/** 沿传入的路径走完 */
	Success			UMETA(DisplayName = "到达"),
	/** 传入的路径不可用：空路径，或路径起点不是 Agent 当前所在的格 */
	InvalidPath		UMETA(DisplayName = "路径不可用"),
	/** 没指定也没在世界里找到体素地形（拿不到格心坐标就没法走） */
	NoTerrain		UMETA(DisplayName = "没找到体素地形"),
	/** 用到的连接被别人占着 / 卡住走不动 */
	Blocked			UMETA(DisplayName = "被占或卡住"),
	/** 被新请求、StopMovement 或销毁中止 */
	Aborted			UMETA(DisplayName = "被中止"),
};

/** 位移由谁执行 */
UENUM(BlueprintType)
enum class EVoxelMoveDrive : uint8
{
	/** 有 INavMovementInterface 就用它，否则直接插值 */
	Auto			UMETA(DisplayName = "自动（有移动组件就用它）"),
	/** 强制直接插值 SetActorLocation：落点精确到格心，不吃物理 */
	DirectLocation	UMETA(DisplayName = "直接插值"),
	/** 强制走 INavMovementInterface（CharacterMovement 等）；找不到移动组件时退回直接插值 */
	NavMovement		UMETA(DisplayName = "走移动组件"),
};

/** 组件当前在干什么（只读，方便 PIE 里观察） */
UENUM(BlueprintType)
enum class EVoxelPathFollowingStatus : uint8
{
	Idle		UMETA(DisplayName = "空闲"),
	Moving		UMETA(DisplayName = "行走中"),
	/** 暂停中：路径与进度都保留着，ResumeMove 接着走；不会广播结果 */
	Paused		UMETA(DisplayName = "已暂停"),
	WaitingLink	UMETA(DisplayName = "等连接代理放行"),
};

/** 移动结束时给调用方的回执 */
USTRUCT(BlueprintType)
struct FVoxelPathFollowingResultInfo
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Voxel|Navigation")
	EVoxelPathFollowingResult Code = EVoxelPathFollowingResult::Success;

	/** 传入路径的最后一个点（组件不做吸附/替换，也不替调用方挑终点） */
	UPROPERTY(BlueprintReadOnly, Category = "Voxel|Navigation")
	FIntVector GoalCoord = FIntVector::ZeroValue;

	/** 结束时所处的格 */
	UPROPERTY(BlueprintReadOnly, Category = "Voxel|Navigation")
	FIntVector FinalCoord = FIntVector::ZeroValue;

	/** 传入路径的点数（0 = 没路径可走） */
	UPROPERTY(BlueprintReadOnly, Category = "Voxel|Navigation")
	int32 PathPoints = 0;
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FVoxelMoveFinishedSignature, const FVoxelPathFollowingResultInfo&, Result);

/**
 * 让 Owner 沿「调用方算好的路径」逐格行走（不寻路、不依赖 NavMesh，也不驱动行为树）。
 *
 * 路径从哪来：调用方自己算（通常就是 AVoxelTerrainActor::FindPath），再交给 RequestMove。
 * 组件只负责走：不寻路、不判断终点合法性、也不替调用方换终点。
 *
 * 怎么走：平面跳（同层水平相邻，FVoxelPathPoint::LinkClass 为空）由本组件插值走过去；
 * 非平面跳（台阶 / 手动连接，LinkClass 非空）交给 UVoxelNavLinkProxy：把 Agent 交出去，
 * 等它调 ResumePathFollowing 放行后接着走剩下的路。
 *
 * 位移有两套驱动（EVoxelMoveDrive）：
 *   - DirectLocation：直接插值 SetActorLocation，落点精确到格心，不吃物理；
 *   - NavMovement：Owner 上有 INavMovementInterface（CharacterMovement / NavMovementComponent）时
 *     下发速度/输入，由移动组件决定实际位移，到点按容差判定。
 * Auto（默认）就是「有移动组件就用它，没有就退回直接插值」。
 *     ⚠ 用 UFloatingPawnMovement 这类「没在跟引擎路径就不吃 RequestDirectMove」的移动组件时，
 *       请在移动组件上打开 bUseAccelerationForPaths（Nav Movement 分类里）：本组件发现这个开关后会自动
 *       改用 RequestPathMove 下发输入，否则那一套会把你请求的速度当成「没有输入」慢慢抹掉。
 *       CharacterMovement 不受影响（它每帧直接吃 RequestedVelocity）。
 *
 * 约定：
 *   - 组件的活动范围是「一个 Owner 一个跟随组件」；
 *   - 不做路径重算、不做中途阻挡规避（回合制一次只有一个角色动），路径一旦收下就一路走完；
 *   - 走的过程中可以用 PauseMove / ResumeMove 暂停与继续：保留路径与进度，不广播结果、不重新寻路；
 *   - 只要 bClaimCells 开着，组件就会用地形的占地表：出发时预约终点、行进中维护自己占的那一格、
 *     过连接时占住那条连接，这样别人的 A* 会自动绕开角色。
 *     ⚠ 这条约定有顺序要求：要连着重发请求时先 StopMovement() 再算路径，否则上一个请求预约的终点
 *       还占在占地表里，A* 会把它当成别人占着。
 */
UCLASS(ClassGroup = (Voxel), meta = (BlueprintSpawnableComponent))
class VOXELTERRAIN_API UVoxelPathFollowingComponent : public UActorComponent
{
	GENERATED_BODY()

public:

	UVoxelPathFollowingComponent();

	/* ===================== 配置 ===================== */

	/** 行走速度（厘米/秒）。<=0 表示瞬移：直接插值模式逐格吸附，移动组件模式改用角色自身的最大速度 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation", meta = (ClampMin = "0"))
	float MoveSpeed = 300.f;

	/** 位移交给谁执行 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation")
	EVoxelMoveDrive Drive = EVoxelMoveDrive::Auto;

	/**
	 * 期望角色原点相对「落脚格格心」的偏移，**只有 Z 有意义**（水平分量会被忽略并告警）。
	 * 例：体素 100、胶囊半高 88 的角色，想让脚底踩在格底面上 → offset.Z = 88 - 50 = 38。
	 * 水平方向不能偏：角色必须落在格心上，「Actor 位置 ↔ 格坐标」才是一一对应的。
	 * 留 0 时按 bAutoLocationOffset 自动推算（角色类默认就能对上）。
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation")
	FVector LocationOffset = FVector::ZeroVector;

	/**
	 * LocationOffset.Z 留 0（默认）时，按移动组件的碰撞柱体自动推算竖直偏移：
	 * 取「半高 − 半个体素」，让胶囊底正好踩在落点格的底面上。
	 * 角色原点若停在格心高度是**够不到**的（格心在地板里，差半个胶囊高），表现就是
	 * 「一直差 40 厘米、卡住」——见日志里的 Blocked 提示。
	 * 手配了 LocationOffset.Z 就以手配为准；没有移动组件（直接插值）或没有碰撞柱体时按 0 处理。
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation")
	bool bAutoLocationOffset = true;

	/**
	 * 到点判定容差（体素边长的百分比，下限 0.1 厘米）。
	 * DirectLocation 模式到点会精确吸附到格心，2% 就够；移动组件驱动的角色是物理停稳的、
	 * 收不到最后那几厘米，所以那种模式下的容差至少按一格的 10% 算（角色仍稳稳在正确的格子里）。
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation", meta = (ClampMin = "0"))
	float ArrivalTolerancePct = 2.f;

	/** 行走时把角色转向移动方向（只改 Yaw），按 TurnRate 平滑转。AIController 管朝向的角色请保持关闭，免得两边抢方向 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation")
	bool bFaceMoveDirection = false;

	/**
	 * 转向角速度（度/秒）：只在 bFaceMoveDirection 打开时生效，转的时候走最短的一边（不会为 350°→10° 绕一大圈）。
	 * 默认 360 差不多是一秒转一圈；调小更迟缓、调大更利落。配 <=0 就是瞬时转向（生硬，一般不要）。
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation", meta = (ClampMin = "0", EditCondition = "bFaceMoveDirection"))
	float TurnRate = 360.f;

	/** DirectLocation 模式的每步位移是否带扫描（撞到东西就停下）；关掉就是纯粹的瞬移式插值 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation", meta = (EditCondition = "Drive != EVoxelMoveDrive::NavMovement"))
	bool bSweepWhileMoving = false;

	/** 是否使用地形的占地表（预约终点 / 维护自己那格 / 用连接时占住连接） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation")
	bool bClaimCells = true;

	/** 移动组件模式：结束时清掉最后一次速度请求，免得角色停不下来 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation", meta = (EditCondition = "Drive != EVoxelMoveDrive::DirectLocation"))
	bool bStopMovementOnFinish = true;

	/** 卡住保护（秒）：这么久都没能更靠近当前目标点就按 Blocked 收尾；<=0 关闭 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation", meta = (ClampMin = "0"))
	float MaxStallTime = 1.5f;

	/** 等连接代理放行的超时（秒）：<=0 表示无限等。防止代理忘了放行把回合永久挂住 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation", meta = (ClampMin = "0"))
	float LinkWaitTimeout = 0.f;

	/** 移动期间自动开组件 Tick。想让游戏自己按节奏调 AdvanceFollowing 就关掉 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation")
	bool bTickWhileMoving = true;

	/** 排查用：打开后 RequestMove 会打印起点/终点/路径点数/驱动/偏移/Actor 位置，行进中每跨一格打印一行 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation")
	bool bLogNavigation = false;

	/** 用哪块地形。留空时自动取世界里的第一块（多块地形请显式指定） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation")
	TObjectPtr<AVoxelTerrainActor> Terrain;

	/* ===================== 请求与状态 ===================== */

	/**
	 * 沿调用方算好的路径走。组件不寻路、不校验终点，只负责把这条路径走完。
	 *
	 * 对路径的要求：
	 *   - 第一个点必须是 Agent 当前所在的格（用与调用方相同的坐标口径算出来的），否则拒绝并报 InvalidPath：
	 *     起点对不上时直线走向 path[0] 可能切过墙体（通常说明路径是旧的、或来自别的地形/别的位置）；
	 *   - 相邻两点要么「同层水平相邻」（组件自己插值过去），要么后一个点的 LinkClass 非空
	 *     （这一跳交给 UVoxelNavLinkProxy）；跨格又没带代理类的跳会记一条警告，但仍按直线走；
	 *   - 只有一个点的路径表示「走到这一格心」（比如自己脚下这格）。
	 *
	 * 路径一般这么来：
	 * @code
	 *   FIntVector Start = Terrain->WorldLocationToCoord(Actor->GetActorLocation() - Comp->LocationOffset);
	 *   TArray<FVoxelPathPoint> Path = Terrain->FindPath(Start, Goal, AgentHeight);
	 *   Comp->RequestMove(Path);   // 空数组会被拒（InvalidPath）
	 * @endcode
	 * ⚠ 连续重发请求时先 StopMovement() 再算路径：上一个请求预约的终点还占在占地表里，
	 *   FindPath 会把自己预约的格当成别人占着。
	 *
	 * 返回是否出发了。失败（InvalidPath / NoTerrain / Blocked）会立刻广播一次 OnMoveFinished，不会动 Actor。
	 * 暂停中发新请求：旧请求照旧被顶掉（收到 Aborted），新请求直接开始走 —— 新请求不受暂停影响。
	 * 在 OnMoveFinished 回调里再次调用它是允许的（链式移动），唯一例外见下面 Aborted 那条：
	 * 「本次广播本身就来自被新请求顶掉的那次中止」时重入会被拒绝并告警。
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	bool RequestMove(const TArray<FVoxelPathPoint>& InPath);

	/** 中止当前移动（正在等的连接会被释放），结果码 Aborted */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	void StopMovement();

	/**
	 * 暂停当前移动：路径、游标、占地登记全都原样保留，也不会广播结果，随时可以 ResumeMove 接着走。
	 *
	 * 只有正在行走（Moving）时才能暂停：
	 *   - 空闲 / 已经暂停 → 什么都不做，返回 false；
	 *   - 正在等连接代理放行（IsUsingLink()）→ 拒绝并告警：那一刻 Agent 归代理驱动（跳跃弧线 / 爬梯动画），
	 *     要停得让代理停它自己的动作，硬暂停会打断「占连接 -> 放行」的握手。
	 * 移动组件模式下会顺手把最后一次速度请求清零，否则角色会带着暂停前那一下的速度继续滑出去。
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	bool PauseMove();

	/**
	 * 从暂停处继续走：停在原地接着走剩下的路径，不重新寻路、不校验路径是否还成立
	 * （单角色回合制，暂停期间没人会把它挤走；要是你自己在这期间挪了 Actor，它也照原路径走）。
	 * 只有确实处于暂停状态时才返回 true。
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	bool ResumeMove();

	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	bool IsPaused() const { return Status == EVoxelPathFollowingStatus::Paused; }

	/** 按 dt 推进一次。Tick 会调它；不想让组件自己 Tick 时由游戏调它 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	void AdvanceFollowing(float DeltaTime);

	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	bool IsFollowingPath() const { return Status != EVoxelPathFollowingStatus::Idle; }

	/** 是否正等人（或别的系统）把非平面连接的动作做完 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	bool IsUsingLink() const { return Status == EVoxelPathFollowingStatus::WaitingLink; }

	/** 当前所处的格（按 Actor 位置反推）。没有地形 / 没有 Owner 时返回 (MAX_int32, MAX_int32, MAX_int32) */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	FIntVector GetCurrentCoord() const;

	/** 最近一次传入路径的终点（路径的最后一个点） */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	FIntVector GetDestinationCoord() const { return ActiveGoal; }

	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	FVoxelPathFollowingResultInfo GetLastResult() const { return LastResult; }

	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	void SetTerrain(AVoxelTerrainActor* InTerrain) { Terrain = InTerrain; }

	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	AVoxelTerrainActor* GetTerrain() const;

	/** 最近一次请求的路径（调试/画线用；请求失败时为空） */
	const TArray<FVoxelPathPoint>& GetPathPoints() const { return Path; }

	/** 当前路径走到第几个点（指向正在前往/正站着的那一点） */
	int32 GetPathIndex() const { return PathIndex; }

	/** 仅供 UVoxelNavLinkProxy 回调：非平面连接的动作做完了，放行继续走 */
	void ResumeFromLink();

	/**
	 * 把 Owner 精确放到某一格的「站位」上（格心 + GetCellOffset()）。给默认的连接代理复用它，
	 * 免得代理直接把角色按到格心里、陷进地板（那不是它该站的高度）。
	 */
	void SnapToCell(const FIntVector& Coord);

	/** 移动结束时广播一次（进度不广播；失败也广播，调用方挂这一个回调就能处理所有分支） */
	UPROPERTY(BlueprintAssignable, Category = "Voxel|Navigation")
	FVoxelMoveFinishedSignature OnMoveFinished;

	/* ===================== 状态（只读，方便观察） ===================== */

	UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Transient, Category = "Voxel|Navigation")
	EVoxelPathFollowingStatus Status = EVoxelPathFollowingStatus::Idle;

protected:

	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	/**
	 * 把速度交给移动组件之前调用，子类可在这里做避障/调速（也用于 DirectLocation 模式之外的统一 hook）。
	 * @param Velocity	本帧期望速度（厘米/秒），可原地改写
	 */
	virtual void PostProcessMove(FVector& Velocity) const {}

	/** bFaceMoveDirection 打开时，每帧按 TurnRate 把 Yaw 平滑转向移动方向 */
	virtual void FaceDirection(const FVector& Direction, float DeltaTime);

private:

	/** 本帧实际使用的驱动 */
	EVoxelMoveDrive GetActiveDrive() const;

	/** 请求开始时定下驱动（Auto/NavMovement 会去找 Owner 上的 INavMovementInterface 并缓存） */
	void ResolveDrive();

	/** 只读地找地形（不写缓存），给 const 查询用 */
	AVoxelTerrainActor* FindTerrain() const;

	/** 找地形并缓存 */
	AVoxelTerrainActor* ResolveTerrain();

	INavMovementInterface* GetNavMovement() const;

	/** 角色原点该停的位置：格心 + LocationOffset（只取 Z） */
	FVector CellLocation(const FIntVector& Coord) const;

	/** LocationOffset 的有效部分：水平分量一律忽略（角色必须落在格心上）；Z 留 0 时按碰撞柱体自动推算 */
	FVector GetCellOffset() const;

	/** 自动竖直偏移生效时说明一次；手配偏移与自动值差太多时告警（两者都只打印一次/每次请求） */
	void LogCellOffsetOnce();

	/** 体素边长（取三轴最小值，当作一格的长度） */
	float GetVoxelExtent() const;

	float GetArrivalTolerance() const;

	/**
	 * 是否算走到了第 Index 个路径点。
	 * 除了距离容差，还要判「是否已经走过这一跳的垂足」：移动组件带加速/刹车，实际位置可能一步越过格心，
	 * 那时距离反而变大，只按距离判会让角色在格心两侧来回蹭（UE 的 HasReachedCurrentTarget 同理）。
	 */
	bool HasReachedPathPoint(int32 Index, const FVector& Current, const FVector& Target, float Dist) const;

	/** 移动组件模式下本帧不下发移动（坠落/根运动中）*/
	bool HasMovementAuthority() const;

	/** 朝目标推进一步（本帧就把 dt 用完或者交给移动组件） */
	void StepTowardTarget(const FVector& Target, float DeltaTime);

	/** 卡住保护：距离没有实质改善就累计时间，超时按 Blocked 收尾 */
	void UpdateStall(float DistanceToTarget, float DeltaTime);

	/** 跨格时更新「自己占的那一格」（失败只告警，不动旧登记，免得删掉别人的记录） */
	void SyncClaimToCurrentCoord();

	/** 走到第 Index 个路径点时的收尾（DirectLocation 模式在这里精确吸附到格心） */
	void OnReachedPathPoint(int32 Index);

	/** 走上一条非平面连接：占连接 -> 交给代理 -> 代理可能同步放行 */
	void StartLinkHop();

	/** 连接动作结束：释放连接、对齐（运动学模式）、推进路径游标 */
	void FinishLinkHop();

	void ReleaseActiveLink();

	/** 结束当前移动：释放连接与占地、停 Tick、广播结果。空闲状态下调用它就只是广播 */
	void FinishMove(EVoxelPathFollowingResult Code);

	/** 移动组件模式下清掉最后一次速度请求（暂停与收尾共用；没有移动权限时不动它） */
	void StopNavMovement();

	/** 按当前状态开关组件 Tick */
	void UpdateTickState();

	FIntVector InvalidCoord() const { return FIntVector(MAX_int32, MAX_int32, MAX_int32); }

	/* ---- 运行期状态 ---- */

	UPROPERTY(Transient)
	TObjectPtr<UActorComponent> MovementComponent;			// 实现 INavMovementInterface 的那个组件

	/** 最近一次传入路径的终点（路径的最后一个点）：用于 GetDestinationCoord 与占位预约 */
	FIntVector ActiveGoal = FIntVector::ZeroValue;

	TArray<FVoxelPathPoint> Path;
	int32 PathIndex = 0;

	FVoxelPathFollowingResultInfo LastResult;

	/** 当前登记在占地表里属于自己的一格 */
	FIntVector ClaimedCoord = FIntVector::ZeroValue;
	bool bHasClaim = false;
	/** 出发时预约下来的终点格 */
	FIntVector ReservedGoal = FIntVector::ZeroValue;
	bool bHasGoalReservation = false;

	/** 正在使用的连接（用于 ReleaseLink） */
	FVoxelNavLinkProxyData ActiveLink;
	UPROPERTY(Transient)
	TObjectPtr<UVoxelNavLinkProxy> ActiveLinkProxy;
	bool bLinkResumed = false;
	float LinkWaitElapsed = 0.f;

	float StallTimer = 0.f;
	float StallBestDist = TNumericLimits<float>::Max();

	/** RequestMove 正在顶掉旧请求：这期间的回调里重入 RequestMove 会被拒绝 */
	bool bSwitchingRequest = false;

	/** 「自动竖直偏移」只在第一次用到时打印一次，免得每帧刷屏 */
	bool bLoggedAutoOffset = false;
};
