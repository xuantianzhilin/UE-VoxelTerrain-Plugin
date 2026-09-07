#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintAsyncActionBase.h"
#include "Containers/Ticker.h"
#include "BehaviorTree/Tasks/BTTask_BlackboardBase.h"
#include "VoxelPath.h"
#include "VoxelPathAI.generated.h"

class AAIController;
class AVoxelTerrainActor;
class AActor;
class FVoxelPathMover;

/* 寻路移动的最终结果。除 Success/Aborted 外，其余值在寻路阶段失败时给出（含义同 EVoxelPathResult） */
UENUM(BlueprintType)
enum class EVoxelPathMoveResult : uint8
{
	Success         UMETA(DisplayName = "到达终点"),
	TerrainNotFound UMETA(DisplayName = "找不到体素地形"),
	NoController    UMETA(DisplayName = "控制器/棋子无效"),
	DestinationInvalid UMETA(DisplayName = "目标点无效"),
	StartInvalid    UMETA(DisplayName = "起点不可站"),
	GoalInvalid     UMETA(DisplayName = "终点不可站"),
	NoPath          UMETA(DisplayName = "无可达路径"),
	BudgetExceeded  UMETA(DisplayName = "超出展开预算"),
	AreaTooLarge    UMETA(DisplayName = "搜索区过大"),
	Blocked         UMETA(DisplayName = "行进中被卡住"),
	Aborted         UMETA(DisplayName = "被打断"),
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FVoxelPathMoveFinishedSignature, EVoxelPathMoveResult, Result, const FVector&, EndLocation);

/**
 * 体素寻路版 "AI Move To"（latent 蓝图节点）。给一个目标点/Aktor，
 * 先用 FindSurfacePath / FindVoxelPath 算出体素路径，再逐点驱动 AI 走完，结束时广播 OnFinished。
 * 不依赖 NavMesh：每段用直线移动请求（bUsePathfinding=false），落点由体素寻路保证可走。
 * 同一个 Controller 上不要与导航 MoveTo 混用（互相打断）。
 */
UCLASS()
class VOXELTERRAIN_API UVoxelAsyncAction_VoxelMoveTo : public UBlueprintAsyncActionBase
{
	GENERATED_BODY()

public:
	/**
	 * 让 AI 沿体素地形寻路移动到世界坐标。
	 * @param Terrain           留空 = 自动取场景中第一个 AVoxelTerrainActor
	 * @param bSurfacePath      true=表面自由路径（可斜走）；false=严格格路径
	 * @param AcceptanceRadius  每段到达半径（厘米），<=0 时取半个体素
	 * @param BlockedTimeout    连续多少秒没有前进判为被卡住（Failed/Blocked）
	 * @param bSmoothTurning    跟随期间把角色切成“朝移动方向转向”（由 RotationRate 控制转身快慢），
	 *                          否则 APawn 默认的控制器转向会让转身看起来是瞬间完成
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Path", meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject", Keywords = "move ai 寻路 移动 导航", DisplayName = "体素寻路移动到位置"))
	static UVoxelAsyncAction_VoxelMoveTo* VoxelMoveToLocation(UObject* WorldContextObject, AAIController* Controller, const FVector& Destination, AVoxelTerrainActor* Terrain = nullptr, const FVoxelPathParams& Params = FVoxelPathParams(), bool bSurfacePath = true, float AcceptanceRadius = 0.f, float BlockedTimeout = 3.f, bool bSmoothTurning = true);

	/* 同上，目标取 Actor 的脚底位置（发起时取一次，之后不跟踪移动目标）；目标所在格被占据时退到其四正交邻格中离自己最近的可站格 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Path", meta = (BlueprintInternalUseOnly = "true", WorldContext = "WorldContextObject", Keywords = "move ai 寻路 移动 导航 actor", DisplayName = "体素寻路移动到Actor"))
	static UVoxelAsyncAction_VoxelMoveTo* VoxelMoveToActor(UObject* WorldContextObject, AAIController* Controller, AActor* TargetActor, AVoxelTerrainActor* Terrain = nullptr, const FVoxelPathParams& Params = FVoxelPathParams(), bool bSurfacePath = true, float AcceptanceRadius = 0.f, float BlockedTimeout = 3.f, bool bSmoothTurning = true);

	/* 移动结束（到达/失败/打断）时广播。EndLocation = 结束时棋子的脚底位置 */
	UPROPERTY(BlueprintAssignable, Category = "Voxel|Path")
	FVoxelPathMoveFinishedSignature OnFinished;

	virtual void Activate() override;

private:
	void BeginMove();
	void Finish(EVoxelPathMoveResult Result);

	TWeakObjectPtr<UObject> WorldContextRef;
	TWeakObjectPtr<AAIController> ControllerRef;
	TWeakObjectPtr<AVoxelTerrainActor> TerrainRef;
	TWeakObjectPtr<AActor> TargetRef;
	FVector Destination = FVector::ZeroVector;
	FVoxelPathParams Params;
	bool bSurfacePath = true;
	bool bSmoothTurning = true;
	float AcceptanceRadius = 0.f;
	float BlockedTimeout = 3.f;

	TSharedPtr<FVoxelPathMover> Mover;
	FTSTicker::FDelegateHandle TickerHandle;
};

/* ===================== 行为树任务 ===================== */

/**
 * 行为树 "Move To" 的体素寻路版：黑板键（Vector 或 Actor）给目标，
 * 自动算体素路径并沿路走过去（不依赖 NavMesh）。寻路失败/被卡住返回 Failed，
 * 可配合 BT 的重试/降级分支使用。Blackboard Key 选 Vector 或 Actor 类型均可。
 */
UCLASS(ClassGroup = (BehaviorTree), meta = (DisplayName = "Voxel Path Move To", Keywords = "voxel path move to 体素 寻路 移动"))
class VOXELTERRAIN_API UBTTask_VoxelPathMoveTo : public UBTTask_BlackboardBase
{
	GENERATED_UCLASS_BODY()

	/** true=表面自由路径（可斜走，推荐）；false=严格体素格路径 */
	UPROPERTY(EditAnywhere, Category = Move, meta = (DisplayName = "表面自由路径", ToolTip = "关闭则走严格体素格路径（正交 4 邻）。端点都会自动吸附到附近可站格。"))
	bool bSurfacePath = true;

	/** 留空 = 自动取场景中第一个 AVoxelTerrainActor */
	UPROPERTY(EditAnywhere, Category = Move, meta = (DisplayName = "地形 Actor"))
	TObjectPtr<AVoxelTerrainActor> Terrain;

	/** 每段到达半径（厘米），<=0 时取半个体素 */
	UPROPERTY(EditAnywhere, Category = Move, meta = (ClampMin = "0.0", DisplayName = "到达半径"))
	float AcceptanceRadius = 0.f;

	/** 连续多少秒没有前进判为被卡住（任务 Failed） */
	UPROPERTY(EditAnywhere, Category = Move, meta = (ClampMin = "0.5", DisplayName = "卡住超时（秒）"))
	float BlockedTimeout = 3.f;

	/** 台阶辅助：≤0.55 格棱线噪声由抬高的 MaxStepHeight 平滑走上；更高（含一格台阶）用弹道跳（顶点跨棱线解算） */
	UPROPERTY(EditAnywhere, Category = Move, meta = (DisplayName = "台阶辅助（噪声+弹道跳）"))
	bool bJumpAssist = true;

	/** 跟随期间把角色切成“朝移动方向转向”（转身快慢由角色的 RotationRate 决定）；
	 *  关掉则沿用 APawn 默认的控制器转向——通常表现为瞬间转头 */
	UPROPERTY(EditAnywhere, Category = Move, meta = (DisplayName = "平滑转向"))
	bool bSmoothTurning = true;

	/** 寻路参数（跳跃高度/净空/权重等，含义见 FindVoxelPath） */
	UPROPERTY(EditAnywhere, Category = Move)
	FVoxelPathParams PathParams;

	virtual EBTNodeResult::Type ExecuteTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;
	virtual void TickTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, float DeltaSeconds) override;
	virtual EBTNodeResult::Type AbortTask(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory) override;

	virtual uint16 GetInstanceMemorySize() const override;
	virtual void InitializeMemory(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, EBTMemoryInit::Type InitType) const override;
	virtual void CleanupMemory(UBehaviorTreeComponent& OwnerComp, uint8* NodeMemory, EBTMemoryClear::Type CleanupType) const override;

#if WITH_EDITOR
	virtual FString GetStaticDescription() const override;
#endif // WITH_EDITOR
};
