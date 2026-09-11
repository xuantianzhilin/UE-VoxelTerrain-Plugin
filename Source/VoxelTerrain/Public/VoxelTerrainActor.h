#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VoxelChunk.h"
#include "VoxelNavLinkProxy.h"
#include "VoxelTerrainActor.generated.h"

class UVoxelGenerator;
struct FVoxelPathPoint;
class UProceduralMeshComponent;

namespace Voxel
{
	// 向 -inf 取整的整数除法：C++ 的 "/" 向 0 截断，负坐标会把 Section 索引/本地坐标算错
	// inline 不能省：这是公开头文件里的函数体，多个 .cpp 包含它会在链接时报 LNK2005 重复定义
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
	void AddLinkProxy(const FVoxelNavLinkProxyData& ProxyData);
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	void RemoveLinkProxy(const FVoxelNavLinkProxyData& ProxyData);

	/** 自动连接的开关与参数（烘焙要用；NavLinkMaxHeightDiff 夹在 [1, Voxel::LENGTH]，理由同 MaxAllowHeight）*/
	/**
	 * 运行期改「自动非平面连接」的开关与参数：会立刻全量重烘一次导航（连接是烘在 NavData 里的，
	 * 不重烘不生效）。bEnable 为 true 时 ProxyClass 不能为空。
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation", meta = (Keywords = "nav link 连接 台阶"))
	void ConfigureAutoNavLinks(bool bEnable, TSubclassOf<UVoxelNavLinkProxy> ProxyClass, int32 MaxHeightDiff);

	bool ShouldAutoSpawnNavLinks() const { return bAutoSpawNavLink && DefaultLinkProxy != nullptr; }
	int32 GetNavLinkMaxHeightDiff() const { return FMath::Clamp(NavLinkMaxHeightDiff, 1, Voxel::LENGTH - 1); }
	TSubclassOf<UVoxelNavLinkProxy> GetAutoNavLinkProxyClass() const { return DefaultLinkProxy; }
	const TArray<FVoxelNavLinkProxyData>& GetLinkProxyData() const { return LinkData; }

	/* ===================== AI 占地（AI 与 AI 的互相避让） ===================== */

	bool TryOccupyCoord(FIntVector Coord);
	void ReleaseCoord(const FIntVector& Coord);
	bool IsCoordOccupied(const FIntVector& Coord) const;
	UVoxelNavLinkProxy* TryOccupyLink(FVoxelNavLinkProxyData Link);
	void ReleaseLink(const FVoxelNavLinkProxyData& Link);
	bool IsLinkOccupied(const FVoxelNavLinkProxyData& Link) const;

	TArray<TPair<FIntVector, float>> FindFreeNearbyCoord(FIntVector Target, int32 AgentHeight, int32 Radius) const;

	/** 按全局体素坐标取导航节点；返回 nullptr 表示该格不是落脚格（不是地面、被挡住、没烘到或没有 Chunk）*/
	const FVoxelNavCell* FindNavCell(const FIntVector& GlobalCoord) const;

	/**
	 * 限制：只适用于「占地 1 格」的 AI。AgentHeight 管的是落脚点自己那一列的竖直净空，横向完全没判，
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation", meta = (Keywords = "astar path 寻路 路径 找路"))
	TArray<FVoxelPathPoint> FindPath(FIntVector StartCoord, FIntVector EndCoord, int32 AgentHeight) const;

	/*====================== 地形射线检测 ============================*/

public:

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
	UPROPERTY(EditAnywhere, Category = "Voxel|Navigation", meta = (ClampMin = "1", ClampMax = "15", ToolTip = "净空（AllowHeight）统计的格数上限：落脚格向上数这么多格还没被挡就按“至少这么高”封顶。必须 >= 最重的 AI 体型，且不能超过一层的 16 格（否则局部重建要往下牵连的层数就不止一层）。只统计竖直方向，不判 AI 的横向占地：导航图始终按“一格宽”的体型烘焙。"))
	int32 MaxAllowHeight = 4;
	UPROPERTY(EditAnywhere, Category = "Voxel|Navigation")
	bool bAutoSpawNavLink = false;
	UPROPERTY(EditAnywhere, Category = "Voxel|Navigation", meta = (ClampMin = "1", ClampMax = "15", EditCondition = "bAutoSpawNavLink", ToolTip = "自动生成连接时允许的最大高度差（格数）。超过这个高差的两处落脚点，只能靠手动的 AddLinkProxy 连起来。夹在 1~16：不超过一层，否则局部重烘要牵连的层数就不止一层。"))
	int32 NavLinkMaxHeightDiff = 1;
	UPROPERTY(EditAnywhere, Category = "Voxel|Navigation", meta = (EditCondition = "bAutoSpawNavLink"))
	TSubclassOf<UVoxelNavLinkProxy> DefaultLinkProxy;

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

	/*手动加/删连接后，把两端所在的 Section 立刻重烘一遍（连接是烘在 NavData 里的）*/
	void RebuildLinksAround(const FVoxelNavLinkProxyData& ProxyData);

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

	TSet<FIntVector> CoordRecords;
	UPROPERTY(Transient)
	TMap<FVoxelNavLinkProxyData, UVoxelNavLinkProxy*> LinkProxyRecords;
};
