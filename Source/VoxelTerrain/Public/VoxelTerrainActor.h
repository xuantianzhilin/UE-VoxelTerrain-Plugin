#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VoxelChunk.h"
#include "VoxelNavLinkProxy.h"
#include "VoxelNavReservation.h"
#include "VoxelTerrainActor.generated.h"

class UVoxelGenerator;
struct FVoxelPathPoint;
class UProceduralMeshComponent;

namespace Voxel
{
	// 向 -inf 取整的整数除法：C++ 的 "/" 向 0 截断，负坐标会把 Section 索引/本地坐标算错
	inline int32 FloorDivide(int32 Dividend, int32 Divisor)
	{
		check(Divisor > 0);
		return Dividend >= 0 ? Dividend / Divisor : (Dividend - Divisor + 1) / Divisor;
	}
}

USTRUCT(BlueprintType)
struct FVoxelTraceHit
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Voxel")
	bool bHit = false;
	UPROPERTY(BlueprintReadOnly, Category = "Voxel")
	FIntVector Coord = FIntVector::ZeroValue;
	UPROPERTY(BlueprintReadOnly, Category = "Voxel")
	FVector Normal = FVector::ZeroVector;
	UPROPERTY(BlueprintReadOnly, Category = "Voxel")
	FVector ImpactPoint = FVector::ZeroVector;
	UPROPERTY(BlueprintReadOnly, Category = "Voxel")
	FVoxelState Voxel;
};

UCLASS()
class VOXELTERRAIN_API AVoxelTerrainActor : public AActor
{
	GENERATED_BODY()
	
public:

	AVoxelTerrainActor();

	UFUNCTION(BlueprintCallable, Category = "Voxel")
	void SetVoxel(FIntVector Coord, FName TypeName, const FRotator& Rotation = FRotator::ZeroRotator);
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	void SetVoxels(const TArray<FIntVector>& Coords, FName TypeName, const FRotator& Rotation = FRotator::ZeroRotator);
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	FVoxelState GetVoxel(FIntVector Coord) const;
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	const FVector& GetVoxelSize() const { return VoxelSize; }

	UFUNCTION(BlueprintCallable, Category = "Voxel|Coord")
	FIntVector WorldLocationToCoord(const FVector& WorldLocation) const;
	UFUNCTION(BlueprintCallable, Category = "Voxel|Coord")
	FVector CoordToWorldLocation(FIntVector Coord) const;
	/*将世界坐标转换为Section坐标和Section内本地坐标*/
	static TPair<FIntVector, FIntVector> WorldCoordToSectionLocalCoord(const FIntVector& WorldCoord);
	static FIntVector SectionLocalCoordToWorldCoord(const FIntVector& SectionCoord, const FIntVector& LocalCoord);
	UFUNCTION(BlueprintCallable, Category = "Voxel|Coord")
	static FIntVector GetSectionCoordFromWorldCoord(const FIntVector& WorldCoord);

	/*重建该 Section 的网格与导航数据；编辑处落在边界上时，相邻 Section 的遮挡/连接也会变，一并标脏*/
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	void MarkSectionDirty(FIntVector SectionCoord, FIntVector LocalCoord);
	/*按预算消化脏区：每个脏 Section 同时重建网格与导航数据（MaxCount<=0 表示不限数量）*/
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	void RebuildDirtySections(int32 MaxCount = 0);

	/*清地形：销毁网格组件、丢掉 Chunk（连带其导航数据）与刷过的区域权重*/
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	void ClearTerrain();

	const FVoxelSection* GetChunkSection(FIntVector SectionCoord) const;
	FVoxelSection* GetChunkSection(FIntVector SectionCoord);

	/**
	 * 填充长方体。Min/Max 是体素坐标的闭区间（含两端），写反会自动纠正，Z 超出有效高度范围会被裁掉。
	 * TypeName 传 NAME_None 即为挖空该区域。
	 * @param RebuildCount	默认 0：批量写入后全部交给 Tick 分摊重建。编辑器里不跑 Tick，要立刻看到结果就传 >0 或再调 BuildAllMeshes()
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel", meta = (Keywords = "box fill rect 长方体 立方体 填充 挖空"))
	void FillBox(FIntVector Min, FIntVector Max, FName TypeName, const FRotator& Rotation = FRotator::ZeroRotator);

	/**
	 * 生成球体。以 Center 体素为球心，体素索引距离 <= Radius 的体素被填充（Radius=0 即单个体素）。
	 * @param bSolid	false 时只保留最外 1 体素厚的球壳（做洞穴/陨坑边缘好用）
	 * @param RebuildCount	同 FillBox，0 表示交给 Tick 分摊
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel", meta = (Keywords = "sphere ball round 球 球体 圆形 生成"))
	void FillSphere(FIntVector Center, double Radius, FName TypeName, const FRotator& Rotation = FRotator::ZeroRotator, bool bSolid = true);

	UFUNCTION(BlueprintCallable, Category = "Voxel|Generator")
	void RunDefaultGenerator();
	UFUNCTION(BlueprintCallable, Category = "Voxel|Generator")
	void RunGenerator(TSubclassOf<UVoxelGenerator> GeneratorClass);

#if WITH_EDITOR
	/* 细节面板按钮：调用 VoxelGenerator 生成地形。编辑器世界不跑 Tick，所以内部会强制重建全部 Section */
	UFUNCTION(Category = "Voxel", meta = (CallInEditor = "true", DisplayName = "生成默认地形", DisplayPriority = "1", Tooltip = "用 VoxelGenerator 指向的生成器蓝图填充体素并立刻重建网格。"))
	void EditorRunDefaultGenerator();

	/* 细节面板按钮：销毁全部 Section 网格组件、清空 Chunk 数据与其导航数据/权重 */
	UFUNCTION(Category = "Voxel", meta = (CallInEditor = "true", DisplayName = "清除地形", DisplayPriority = "2", Tooltip = "销毁所有 Section 网格组件、清空 Chunk 数据（连带已烘焙的导航数据）与刷过的区域权重，保存关卡后地图里就不再有地形。同时这也是解锁 VoxelSize / MinHeight / MaxHeight 的手段（有数据时这三项不可修改）。"))
	void EditorClearTerrain();

	/* 细节面板按钮：按现有体素数据重建网格并删掉多余组件（正常载入不需要，存档里已经带着网格）*/
	UFUNCTION(Category = "Voxel", meta = (CallInEditor = "true", DisplayName = "重建地形网格", DisplayPriority = "3", Tooltip = "不重跑生成器，按现有体素数据重建所有 Section 的网格。体素与网格不同步（例如改了材质或存档来自旧版本）时用它修复。"))
	void EditorRebuildAllSections();
#endif

	/*================================ 地形网格 ======================================*/

public:

	UFUNCTION(BlueprintCallable, Category = "Voxel|Mesh")
	void BuildAllMeshes();

private:

#if WITH_EDITOR
	void BuildMeshesInEditor(bool bForced = false);
#endif

	/* ===================== 寻路（不依赖 NavMesh，走烘焙出来的导航数据） ===================== */

public:

	/** 净空统计的格数上限：落脚格向上数这么多格还没被挡，就按“至少这么高”封顶（见 FVoxelNavCell::AllowHeight）。
	 *  刻意夹在 [1, Voxel::LENGTH]：净空是向上看的，不超过一层，改一体素时才只需连下方那一层一起重烘 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	int32 GetMaxAllowHeight() const { return FMath::Clamp(MaxAllowHeight, 1, Voxel::LENGTH - 1); }

	/*全量烘焙所有 Section 的导航数据（BeginPlay 已经调过一次；运行期改体素走脏区局部重建）*/
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	void BuildNavData();

	/* 区域通行权重：查询期数据，不参与烘焙（刷权重不需要重烘，烘焙也不读它）。
	   1=正常，>1 更难走，0=软墙（能站但没人绕过来），负数/NaN 拒收；等于 1 的条目不落表 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	void SetCoordNavWeight(FIntVector Coord, float Weight);
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	void ClearCoordNavWeight(FIntVector Coord);
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	float GetCoordNavWeight(FIntVector Coord) const;
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	void SetCoordNavWeightBox(FIntVector Min, FIntVector Max, float Weight);


	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	void AddLinkProxy(const FVoxelNavLinkProxyData& ProxyData, bool bRebuildNavData = true);
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	void RemoveLinkProxy(const FVoxelNavLinkProxyData& ProxyData, bool bRebuildNavData = true);

	/** 自动连接的开关与参数（烘焙要用；NavLinkMaxHeightDiff 夹在 [1, Voxel::LENGTH]，理由同 MaxAllowHeight）*/
	/**
	 * 运行期改「自动非平面连接」的开关与参数：会立刻全量重烘一次导航（连接是烘在 NavData 里的，
	 * 不重烘不生效）。bEnable 为 true 时 ProxyClass 不能为空。
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation", meta = (Keywords = "nav link 连接 台阶"))
	void ConfigureAutoNavLinks(bool bEnable, TSubclassOf<UVoxelNavLinkProxy> ProxyClass, int32 MaxHeightDiff);

	bool ShouldAutoSpawnNavLinks() const { return bAutoSpawNavLink && AutoLinkProxy != nullptr; }
	int32 GetNavLinkMaxHeightDiff() const { return FMath::Clamp(NavLinkMaxHeightDiff, 1, Voxel::LENGTH - 1); }
	TSubclassOf<UVoxelNavLinkProxy> GetAutoNavLinkProxyClass() const { return AutoLinkProxy; }
	const TArray<FVoxelNavLinkProxyData>& GetLinkProxyData() const { return LinkData; }

	/*==================== 体型档（AI 横向占地）====================*/

	/** 体型档表：每项是一个正方形 footprint 的边长（格数）。烘焙与查询用的档号是「归一化后」(升序去重、恒含 1、每项夹在 [1,8]) 的下标。
	 *  改了它必须重烘（ConfigureAgentSizes 会立刻做）；上限 8 是为了让 footprint 的牵连范围不超过一层（Section 边长 16）。 */
	UPROPERTY(EditAnywhere, Category = "Voxel|Navigation", meta = (ClampMin = "1", ClampMax = "8", ToolTip = "支持的 AI 体型档：每行的数字是正方形 footprint 的边长（格数）。1=旧式单格角色。档位越多烘焙越慢越占内存；改完要重烘（点“重烘导航”或调 ConfigureAgentSizes）。footprint 净空按覆盖列的 min 记，查询期直接过滤，不再展开。"))
	TArray<int32> AgentFootprintWidths{ 1 };

	/** 归一化后的体型档表（升序、去重、恒含 1，每项夹在 [1,8]）。烘焙与 ResolveNavTier 用的是同一份。 */
	TArray<int32> GetNavFootprintWidths() const;
	/** 宽 W 对应的档号；没配这一档返回 INDEX_NONE（调用方按「体型不支持」处理） */
	int32 ResolveNavTier(int32 Width) const;
	/** 最大档宽（≥1）。脏区横向牵连 = 它减 1 */
	int32 GetMaxAgentFootprintWidth() const;
	/** 运行期改体型档表：立刻全量重烘导航（footprint 是烘在 NavData 里的） */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation", meta = (Keywords = "agent size footprint 体型"))
	void ConfigureAgentSizes(const TArray<int32>& Widths);

	/** footprint 以 anchor 为格、向 -X/-Y 覆盖 Width/2 格、向 +X/+Y 覆盖 (Width-1)-Width/2 格（奇数档 anchor 即中心格）。 */
	static FIntVector FootprintOrigin(const FIntVector& Anchor, int32 Width);
	/** anchor 格的格心 + 宽体型的水平半格偏移 = 整块 footprint 的中心（1 格体型就是格心） */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	FVector FootprintCenterToWorld(FIntVector Anchor, int32 AgentWidth) const;
	/** FootprintCenterToWorld 的逆运算：footprint 中心世界坐标 → anchor 格（「Actor 位置 ↔ anchor 格」保持一一对应） */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	FIntVector WorldToFootprintAnchor(const FVector& WorldLocation, int32 AgentWidth) const;

	/*==================== AI 占地（硬保证：一格同时只有一个 AI，footprint 整体占）====================*/

	bool TryOccupyCoord(FIntVector Coord, AActor* Occupant);
	void ReleaseCoord(const FIntVector& Coord, AActor* Occupant);
	AActor* GetCoordOccupant(const FIntVector& Coord) const;
	/** 宽体型版：footprint 覆盖的每一格都空闲（或已属于同一 Occupant）才整体登记，否则一格都不写 */
	bool TryOccupyFootprint(FIntVector Anchor, int32 AgentWidth, AActor* Occupant);
	void ReleaseFootprint(FIntVector Anchor, int32 AgentWidth, AActor* Occupant);
	/** footprint 里是否有别人占着的格（IgnoreAgent 自己占的不算） */
	bool IsFootprintBlocked(FIntVector Anchor, int32 AgentWidth, const AActor* IgnoreAgent = nullptr) const;

	/** 吸附用：目标格附近哪些落脚点空着（含体型过滤与 footprint 占地判定），按到目标的切比雪夫距离分组返回 */
	TArray<TPair<FIntVector, float>> FindFreeNearbyCoord(FIntVector Target, int32 AgentHeight, int32 Radius, int32 AgentWidth = 1) const;

	/** 按全局体素坐标取导航节点；返回 nullptr 表示该格（对该体型档）不是落脚格（不是地面、被挡住、没烘到或没有 Chunk）*/
	const FVoxelNavCell* FindNavCell(const FIntVector& GlobalCoord, int32 AgentWidth = 1) const;

	/**
	 * 空间 A*：只躲「当前被占着的格」（硬占地表），不看时间预约。
	 * AgentWidth 必须是已配置的体型档，否则返回空数组并告警；途经/终点格的 footprint 里有别人的占格也绕开。
	 * 寻路结果按 footprint 中心走：路径点坐标是 anchor 格。
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation", meta = (Keywords = "astar path 寻路 路径 找路"))
	TArray<FVoxelPathPoint> FindPath(FIntVector StartCoord, FIntVector EndCoord, int32 AgentHeight, int32 AgentWidth = 1, const AActor* IgnoreAgent = nullptr) const;

	/**
	 * 时空 A*：在空间避让之外，再多看一层「预约表」——按预计到达时刻绕开别人登记在那一格时间窗里的路线，
	 * 对向/交叉冲突在规划期就消解（规划期不安排原地等待；兜底的先占后走见 UVoxelPathFollowingComponent）。
	 * SecondsPerStep：一格平步的基准秒数（调用方按自己的 MoveSpeed 算，<=0 按 1 步 1 秒退化）；
	 * OutTravelTime：整条路径预计耗时（秒）。找不到路（含被预约挡死）返回空数组。
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation", meta = (Keywords = "astar path schedule reservation 预约 寻路"))
	TArray<FVoxelPathPoint> FindPathScheduled(FIntVector StartCoord, FIntVector EndCoord, int32 AgentHeight, int32 AgentWidth,
		const AActor* Agent, float SecondsPerStep, float StartDelay, float& OutTravelTime) const;

	/*==================== 时空预约（群体避障的软规划层）====================*/

	/** 把一条路径按时间窗登记进预约表（footprint 展开到每一格）。整条先做冲突检测，全空才写入（原子提交）；
	 *  失败（有冲突/路径非法）返回 false 且不写任何东西。IgnoreAgent 默认排除 Agent 自己的旧预约。 */
	bool CommitPathReservations(AActor* Agent, const TArray<FVoxelPathPoint>& Path, int32 AgentWidth,
		float SecondsPerStep, float NowSeconds, float& OutTravelTime);
	/** 释放某 Agent 的全部预约（移动结束/中止/销毁时调；弱指针失效的条目也会被读时清理兜底） */
	void RemoveReservationsFor(const AActor* Agent);
	/** [tEnter,tExit] 窗口里 Coord 是否被别人预约占用；给了 From 就连「对穿」（别人此刻正从 Coord 走到 From）一起判 */
	bool IsCoordReservedAt(const FIntVector& Coord, float tEnter, float tExit, const AActor* IgnoreAgent,
		const FIntVector* SwapFrom = nullptr) const;

private:

	/*手动加/删连接后，把两端所在的 Section 立刻重烘一遍（连接是烘在 NavData 里的）*/
	void RebuildLinksAround(const FVoxelNavLinkProxyData& ProxyData);

	/*按档号取节点（档号是归一化体型表的下标；越界返回 nullptr）*/
	const FVoxelNavCell* FindNavCellAtTier(const FIntVector& GlobalCoord, int32 Tier) const;

	/*FindPath / FindPathScheduled 的共同实现。bScheduled 为 true 时代价即秒：G 值就是预计到达时刻，
	  展开时额外查预约表（绕开别人登记的格与对穿），OutTravelTime 带回全程耗时*/
	TArray<FVoxelPathPoint> FindPathInternal(FIntVector StartCoord, FIntVector EndCoord, int32 AgentHeight, int32 AgentWidth,
		const AActor* IgnoreAgent, bool bScheduled, float SecondsPerStep, float StartDelay, float& OutTravelTime) const;

	/*一跳的预计耗时（秒）：平面跳 = BaseSecondsPerStep × 区域权重；代理跳 = max(基准, ExpectedMoveDuration × 代理权重 × 区域权重)。
	  FindPathScheduled 的时间推进与 CommitPathReservations 登记窗口共用这一份口径。*/
	float HopDurationSeconds(const FVoxelPathPoint& ToPoint, float BaseSecondsPerStep) const;

	/*丢掉窗口已过（含宽限）与 Agent 已失效的预约条目；Tick 定期与查询读时都会做*/
	void PruneExpiredReservations(float NowSeconds);

	/*====================== 地形射线检测 ============================*/

public:

	/**
	* 对体素数据做 DDA 射线检测（世界坐标线段 Start→End，不依赖物理碰撞）。
	* 起点在有效高度 [MinHeight, MaxHeight) 之外时，内部会先把线段沿 Z 裁进范围再从交点起测，
	*/
	UFUNCTION(BlueprintCallable, Category = "Voxel|Query")
	bool LineSingleTraceVoxel(const FVector& Start, const FVector& End, FVoxelTraceHit& OutHit);

protected:

	UPROPERTY(EditAnywhere, Category = "Voxel", meta = (ToolTip = "单个体素的边长。已有地形数据时不可修改（先点“清除地形”），否则已生成的网格会与坐标尺度对不上。"))
	FVector VoxelSize{ 100.0 };
	UPROPERTY(EditAnywhere, Category = "Voxel", meta = (ToolTip = "已有地形数据时不可修改（先点“清除地形”），Section 的划分依赖它。"))
	int32 MinHeight = -16;
	UPROPERTY(EditAnywhere, Category = "Voxel", meta = (ToolTip = "已有地形数据时不可修改（先点“清除地形”），Section 的划分依赖它。"))
	int32 MaxHeight = 64;
	UPROPERTY(EditAnywhere, Category = "Voxel|Generator")
	TSoftClassPtr<UVoxelGenerator> VoxelGenerator;
	UPROPERTY(EditAnywhere, Category = "Voxel|Generator")
	bool bRunGeneratorOnBeginPlay = false;
	UPROPERTY(EditAnywhere, Category = "Voxel|Navigation", meta = (ClampMin = "1", ClampMax = "15", ToolTip = "净空（AllowHeight）统计的格数上限：落脚格向上数这么多格还没被挡就按“至少这么高”封顶。必须 >= 最重的 AI 身高，且不能超过一层的 16 格（否则局部重建要往下牵连的层数就不止一层）。宽体型的横向占地按体型档（AgentFootprintWidths）在烘焙期分档记录，查询期按 AgentHeight 过滤分档净空。"))
	int32 MaxAllowHeight = 4;
	UPROPERTY(EditAnywhere, Category = "Voxel|Navigation")
	bool bAutoSpawNavLink = false;
	UPROPERTY(EditAnywhere, Category = "Voxel|Navigation", meta = (ClampMin = "1", ClampMax = "15", EditCondition = "bAutoSpawNavLink", ToolTip = "自动生成连接时允许的最大高度差（格数）。超过这个高差的两处落脚点，只能靠手动的 AddLinkProxy 连起来。夹在 1~16：不超过一层，否则局部重烘要牵连的层数就不止一层。"))
	int32 NavLinkMaxHeightDiff = 1;
	UPROPERTY(EditAnywhere, Category = "Voxel|Navigation", meta = (EditCondition = "bAutoSpawNavLink"))
	TSubclassOf<UVoxelNavLinkProxy> AutoLinkProxy;

	UPROPERTY(VisibleAnywhere, Category = "Voxel")
	TObjectPtr<USceneComponent> VoxelRoot;

	/* 体素变化数超过该阈值才进行 Compact*/
	static constexpr int32 CompactThreshold = 100;
	/* 每帧最多重建区域的数量 */
	static constexpr int32 MaxRebuildCountPerTick = 5;
	/* 单次批量填充允许的最大体素数（包围盒计数），防止蓝图里一个笔误把编辑器卡死 */
	static constexpr int64 MaxBulkVoxelCount = 1024 * 1024;
	/* 单次批量刷权重允许的最大格数 */
	static constexpr int32 MaxBulkWeightCells = 64 * 1024;

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	virtual void PostRegisterAllComponents() override;

	/**
	 * 体素数据是 UPROPERTY，序列化交给反射系统（网格组件是派生数据，不入档）；
	 * 这里只做载入包体后的收尾：清脏区、校验高度范围、打上“需要补建网格”的标记。
	 */
	virtual void Serialize(FArchive& Ar) override;

#if WITH_EDITOR
	/** 已有地形数据时锁死 VoxelSize / MinHeight / MaxHeight，避免数据与坐标尺度错位 */
	virtual bool CanEditChange(const FProperty* InProperty) const override;
#endif

private:

	FVoxelChunk& FindOrAddChunk(FIntVector2 ChunkCoord);

	/** 改一体素时，导航在 Z 方向能牵连到多远的格子：净空向上要看 MaxAllowHeight 格，
 *  自动连接又可能把连接拉到 NavLinkMaxHeightDiff 格高 —— 取二者较大，脏区判定用它 */
	int32 GetMaxImpactHeight() const;

	/** 改一体素时，导航在水平方向能牵连到多远：宽档 footprint 最远读到旁边 MaxW-1 格，脏区横向判定用它
	 *  （体型档上限 8 < Section 边长 16，牵连不会越过相邻 Section；1×1 档时为 0，与旧行为一致） */
	int32 GetMaxImpactPad() const;

	UPROPERTY()
	TMap<FIntVector2, FVoxelChunk> Chunks;
	TMap<FIntVector2, TArray<FIntVector>> DirtySections;
#if WITH_EDITOR
	bool bNeedsMeshBuild = false;
#endif

	UPROPERTY()
	TMap<FIntVector, float> NavWeights;
	UPROPERTY()
	TArray<FVoxelNavLinkProxyData> LinkData;

	mutable TMap<FIntVector, TWeakObjectPtr<AActor>> CoordRecords;

	/* 时空预约表（群体避障的软规划层）：一格 -> 若干时间窗。纯运行期数据，派生自移动中的 AI，不入档不序列化 */
	TMap<FIntVector, TArray<FNavReservation>> Reservations;
};
