#pragma once

#include "CoreMinimal.h"
#include "VoxelPath.generated.h"

class UActorComponent;

/* 寻路结果状态 */
UENUM(BlueprintType)
enum class EVoxelPathResult : uint8
{
	Found         UMETA(DisplayName = "找到路径"),
	StartInvalid  UMETA(DisplayName = "起点不可站/非法"),
	GoalInvalid   UMETA(DisplayName = "终点不可站/非法"),
	NoPath        UMETA(DisplayName = "无可达路径"),
	BudgetExceeded UMETA(DisplayName = "超出展开预算"),
	AreaTooLarge  UMETA(DisplayName = "搜索区过大"),
};

/**
 * 寻路参数。JumpHeight / AgentHeight 以体素为单位。
 * 一次查询构造一个求解器，结果不缓存（体素/权重可随时被改，AI 寻路是离散事件）。
 */
USTRUCT(BlueprintType)
struct FVoxelPathParams
{
	GENERATED_BODY()

	/** 可跨越的高度差（体素数，对称：上跳与下落都受此限）。0=只能走同层 */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Path", meta = (ClampMin = "0", ToolTip = "两个落脚格的高度差绝对值 <= 此值才视为可通过。"))
	int32 JumpHeight = 1;

	/** 净空高度：落脚点自身起，向上需连续 AgentHeight 格为空。默认 1（只看落脚格本身），按 AI 体型调大 */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Path", meta = (ClampMin = "1"))
	int32 AgentHeight = 1;

	/** Grid 模式是否允许 8 邻（含水平斜向格步）。Surface 模式恒为真，此项无效。默认 false=严格正交 */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Path")
	bool bAllowDiagonal = false;

	/** 是否把其他 Actor 的静态网格体所占坐标等效为体素被占据 */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Path")
	bool bCheckStaticMeshes = true;

	/** 骨骼网格体（USkeletalMeshComponent）是否也计入占据（跟随 bCheckStaticMeshes 生效；
	 *  Pawn/Character 自己身上的网格始终不算障碍，避免“路过的角色挡住路”） */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Path")
	bool bCheckSkeletalMeshes = true;

	/** 是否计入区域代价权重（关掉即无视已刷的权重区） */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Path")
	bool bUseCostWeights = true;

	/** 阻挡检测的组件类过滤。留空=仅 UStaticMeshComponent（体素地形的 ProceduralMesh 始终排除） */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Path", meta = (AllowAbstract = ""))
	TSubclassOf<UActorComponent> BlockerComponentClass;

	/** A* 展开节点数上限。超过返回 BudgetExceeded */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Path", meta = (ClampMin = "1"))
	int32 MaxExpansions = 20000;

	/** 搜索区在起止包围盒外扩的体素数（同时决定静态网格烘焙区边界） */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Path", meta = (ClampMin = "0"))
	int32 SearchPadding = 16;

	/** Surface 模式端点吸附：在起止世界坐标附近，XY 方向搜索可站格的最大半径（格） */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Path", meta = (ClampMin = "0"))
	int32 SnapRadiusXY = 4;

	/** Surface 模式端点吸附：相对给定点向上可容忍的层数（角色陷进方块/表面被重建抬高一点时靠它救回） */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Path", meta = (ClampMin = "0"))
	int32 SnapUpTolerance = 4;

	/** Surface 模式端点吸附：相对给定点向下可容忍的层数 */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Path", meta = (ClampMin = "0"))
	int32 SnapDownTolerance = 8;

	/** 画调试线（仅游戏世界），把格路径/世界路径/网格占据/权重区可视化出来 */
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Path")
	bool bDebugDraw = false;
};

/** 寻路输出。两种模式都会填 VoxelPath（格路径），WorldPath 是最终可跟随的世界坐标折线 */
USTRUCT(BlueprintType)
struct FVoxelPathResult
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Path")
	EVoxelPathResult Result = EVoxelPathResult::NoPath;

	/** 原始 A* 格路径（格心坐标）。Grid 模式即最终路径；Surface 模式是平滑前的骨架 */
	UPROPERTY(BlueprintReadOnly, Category = "Path")
	TArray<FIntVector> VoxelPath;

	/** 世界坐标路径：Grid=格折线；Surface=贴地平滑折线，段方向任意，可直接喂 AI 移动。
	 *  点语义 = 落脚面（角色脚底目标），Z 比格心低半个体素 */
	UPROPERTY(BlueprintReadOnly, Category = "Path")
	TArray<FVector> WorldPath;

	/** 稀疏控制折线：Surface 模式为拉紧平滑后的转弯点（点数少、方向任意），移动跟随建议吃这条；
	 *  Grid 模式与 WorldPath 相同。密采贴面点保留在 WorldPath 里，适合画调试线/轨迹回放。Z 同 WorldPath=落脚面 */
	UPROPERTY(BlueprintReadOnly, Category = "Path")
	TArray<FVector> ControlPath;

	UPROPERTY(BlueprintReadOnly, Category = "Path")
	float TotalCost = 0.f;

	UPROPERTY(BlueprintReadOnly, Category = "Path")
	int32 Expansions = 0;
};
