#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VoxelChunk.h"
#include "VoxelTerrainActor.generated.h"

class UVoxelGenerator;
class UProceduralMeshComponent;
struct FVoxelPathSolver;

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
	FIntVector Coord;
	UPROPERTY(BlueprintReadOnly, Category = "Voxel")
	FVector Normal;
	UPROPERTY(BlueprintReadOnly, Category = "Voxel")
	FVector ImpactPoint;
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
	FIntVector WorldLocationToCoord(const FVector& WorldLocation) const;
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	FVector CoordToWorldLocation(FIntVector Coord) const;
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	const FVector& GetVoxelSize() const { return VoxelSize; }
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	int32 GetMaxNavHeight() const { return FMath::Max(MaxHeight, MaxNavHeight); }

	UFUNCTION(BlueprintCallable, Category = "Voxel")
	void SetVoxel(FIntVector Coord, FName TypeName, const FRotator& Rotation = FRotator::ZeroRotator);
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	void SetVoxels(const TArray<FIntVector>& Coords, FName TypeName, const FRotator& Rotation = FRotator::ZeroRotator);

	UFUNCTION(BlueprintCallable, Category = "Voxel")
	FVoxelState GetVoxel(FIntVector Coord) const;

	const FVoxelSection* GetChunkSection(FIntVector SectionCoord) const;
	FVoxelSection* GetChunkSection(FIntVector SectionCoord);

	/**
	 * 填充长方体。Min/Max 是体素坐标的闭区间（含两端），写反会自动纠正，Z 超出有效高度范围会被裁掉。
	 * TypeName 传 NAME_None 即为挖空该区域。
	 * @param RebuildCount	默认 0：批量写入后全部交给 Tick 分摊重建。编辑器里不跑 Tick，要立刻看到结果就传 >0 或再调 RebuildAllSections()
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

	/*重建该 Section 的网格；编辑处位于 Section 边界时，相邻 Section 的遮挡关系也会变化，一并重建*/
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	void MarkSectionDirty(FIntVector SectionCoord, FIntVector LocalCoord);
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	void RebuildDirtySections(int32 MaxCount = 0);
	/*重建所有 Section 的网格：不走脏区队列，数据没变化时也能把网格恢复出来（编辑器按钮用）*/
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	void RebuildAllSections();
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	void ClearTerrain();

	UFUNCTION(BlueprintCallable, Category = "Voxel|Generator")
	void RunDefaultGenerator();
	UFUNCTION(BlueprintCallable, Category = "Voxel|Generator")
	void RunGenerator(TSubclassOf<UVoxelGenerator> GeneratorClass);

#if WITH_EDITOR
	/* 细节面板按钮：调用 VoxelGenerator 生成地形。编辑器世界不跑 Tick，所以内部会强制重建全部 Section */
	UFUNCTION(Category = "Voxel", meta = (CallInEditor = "true", DisplayName = "生成默认地形", DisplayPriority = "1", Tooltip = "用 VoxelGenerator 指向的生成器蓝图填充体素并立刻重建网格。"))
	void EditorRunDefaultGenerator();

	/* 细节面板按钮：销毁全部 Section 网格组件并清空 Chunk 数据 */
	UFUNCTION(Category = "Voxel", meta = (CallInEditor = "true", DisplayName = "清除地形", DisplayPriority = "2", Tooltip = "销毁所有 Section 网格组件并清空 Chunk 数据，保存关卡后地图里就不再有地形。同时这也是解锁 VoxelSize / MinHeight / MaxHeight 的手段（有数据时这三项不可修改）。"))
	void EditorClearTerrain();

	/* 细节面板按钮：按现有体素数据重建网格并删掉多余组件（正常载入不需要，存档里已经带着网格）*/
	UFUNCTION(Category = "Voxel", meta = (CallInEditor = "true", DisplayName = "重建地形网格", DisplayPriority = "3", Tooltip = "不重跑生成器，按现有体素数据重建所有 Section 的网格。体素与网格不同步（例如改了材质或存档来自旧版本）时用它修复。"))
	void EditorRebuildAllSections();
#endif

	/* ===================== 寻路（不依赖 NavMesh，直接基于体素数据） ===================== */

	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	void BuildNavData();


	//UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	//void SetVoxelPathWeight(FIntVector Coord, float Weight);
	//UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	//void ClearVoxelPathWeight(FIntVector Coord);
	//UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	//float GetVoxelPathWeight(FIntVector Coord) const;
	//UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	//void SetVoxelPathWeightBox(FIntVector Min, FIntVector Max, float Weight);

	UFUNCTION(BlueprintCallable, Category = "Voxel|Query")
	bool LineSingleTraceVoxel(const FVector& Start, const FVector& End, FVoxelTraceHit& OutHit);

protected:

	UPROPERTY(EditAnywhere, Category = "Voxel", meta = (ToolTip = "单个体素的边长。已有地形数据时不可修改（先点“清除地形”），否则已生成的网格会与坐标尺度对不上。"))
	FVector VoxelSize{ 100.0 };
	/** 体素高度范围（体素为单位，必须是 Voxel::LENGTH 的整数倍） */
	UPROPERTY(EditAnywhere, Category = "Voxel", meta = (ToolTip = "已有地形数据时不可修改（先点“清除地形”），Section 的划分依赖它。"))
	int32 MinHeight = -16;
	UPROPERTY(EditAnywhere, Category = "Voxel", meta = (ToolTip = "已有地形数据时不可修改（先点“清除地形”），Section 的划分依赖它。"))
	int32 MaxHeight = 64;
	UPROPERTY(EditAnywhere, Category = "Voxel|Generator")
	TSoftClassPtr<UVoxelGenerator> VoxelGenerator;
	UPROPERTY(EditAnywhere, Category = "Voxel|Generator")
	bool bRunGeneratorOnBeginPlay = false;
	UPROPERTY(EditAnywhere, Category = "Voxel|Navigation")
	int32 MaxNavHeight = 70;

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

	/*将世界坐标转换为Section坐标和Section内本地坐标*/
	static TPair<FIntVector, FIntVector> WorldCoordToChunkLocalCoord(const FIntVector& WorldCoord);
	FVoxelChunk& FindOrAddChunk(FIntVector2 ChunkCoord);
	void BuildMeshesIfNeeded();

	UPROPERTY()
	TMap<FIntVector2, FVoxelChunk> Chunks;
	TMap<FIntVector2, TArray<FIntVector>> DirtySections;
	bool bNeedsMeshBuild = false;

	UPROPERTY()
	TMap<FIntVector, float> NavWeights;
};
