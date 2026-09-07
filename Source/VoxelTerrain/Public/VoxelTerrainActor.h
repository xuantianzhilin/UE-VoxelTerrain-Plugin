#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "VoxelChunk.h"
#include "VoxelPath.h"
#include "VoxelTerrainActor.generated.h"

class UVoxelGenerator;
class UProceduralMeshComponent;
struct FVoxelPathSolver;

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
	void RunDefaultGenerator();
	UFUNCTION(BlueprintCallable, Category = "Voxel")
	void RunGenerator(TSubclassOf<UVoxelGenerator> GeneratorClass);

#if WITH_EDITOR
	/* 细节面板按钮：调用 VoxelGenerator 生成地形。编辑器世界不跑 Tick，所以内部会强制重建全部 Section */
	UFUNCTION(Category = "Voxel", meta = (CallInEditor = "true", DisplayName = "生成默认地形", DisplayPriority = "1",
		Tooltip = "用 VoxelGenerator 指向的生成器蓝图填充体素并立刻重建网格。", Keywords = "generate terrain 生成 地形"))
	void GenerateDefaultTerrain();

	/* 细节面板按钮：销毁全部 Section 网格组件并清空 Chunk 数据 */
	UFUNCTION(Category = "Voxel", meta = (CallInEditor = "true", DisplayName = "清除地形", DisplayPriority = "2",
		Tooltip = "销毁所有 Section 网格组件并清空 Chunk 数据，保存关卡后地图里就不再有地形。同时这也是解锁 VoxelSize / MinHeight / MaxHeight 的手段（有数据时这三项不可修改）。", Keywords = "clear terrain 清除 地形"))
	void ClearTerrain();

	/* 细节面板按钮：按现有体素数据重建网格并删掉多余组件（正常载入不需要，存档里已经带着网格）*/
	UFUNCTION(Category = "Voxel", meta = (CallInEditor = "true", DisplayName = "重建地形网格", DisplayPriority = "3",
		Tooltip = "不重跑生成器，按现有体素数据重建所有 Section 的网格。体素与网格不同步（例如改了材质或存档来自旧版本）时用它修复。", Keywords = "rebuild mesh 重建 网格 修复"))
	void RebuildTerrainMesh();
#endif

	/* ===================== 寻路（不依赖 NavMesh，直接基于体素数据） ===================== */

	/**
	 * 模式 A：严格以体素为单位的 Grid 寻路（默认只走正交 4 邻，可经 bAllowDiagonal 放开 8 邻）。
	 * 起终点是体素坐标，必须本身可站立（不可站时返回 StartInvalid/GoalInvalid，不做吸附）。
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Path", meta = (Keywords = "path find ai 寻路 网格 grid"))
	FVoxelPathResult FindVoxelPath(FIntVector Start, FIntVector End, const FVoxelPathParams& Params = FVoxelPathParams()) const;

	/**
	 * 模式 B：表面自由寻路。起终点是世界坐标，自动吸附到附近可站格；
	 * 路径经贴面拉紧平滑，可以任意方向斜着走（不必沿格线），WorldPath 可直接交给 AI 跟随。
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Path", meta = (Keywords = "path find ai 寻路 斜走 表面 surface"))
	FVoxelPathResult FindSurfacePath(const FVector& StartWorld, const FVector& EndWorld, const FVoxelPathParams& Params = FVoxelPathParams()) const;

	/**
	 * 单格可站性查询（不做 A*）：格内与头顶净空无体素/网格占据 + 脚下有支撑 + 权重非 0。
	 * 判定规则与寻路内部保持一致，bCheckStaticMeshes 时附带单格碰撞查询。
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Path", meta = (Keywords = "walkable standable 可站 站立"))
	bool IsVoxelStandable(FIntVector Coord, int32 AgentHeight = 1, bool bCheckStaticMeshes = true) const;

	/**
	 * 给单个体素坐标写步代价权重（乘算）：1=默认（等同清除），0=不可进入（软墙），(0,1)=快速区，>1=难行区。
	 * 非法值（NaN / 负数）记 Warning 并忽略。只影响寻路，不影响网格与碰撞。
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Path", meta = (Keywords = "weight cost 权重 代价 区域 nav"))
	void SetVoxelPathWeight(FIntVector Coord, float Weight);

	/* 等价于 SetVoxelPathWeight(Coord, 1.0) */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Path", meta = (Keywords = "weight clear 清除 权重"))
	void ClearVoxelPathWeight(FIntVector Coord);

	/* 未刷过的坐标返回 1 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Path", meta = (Keywords = "weight get 权重"))
	float GetVoxelPathWeight(FIntVector Coord) const;

	/** 批量刷长方体权重。Min/Max 是体素坐标闭区间（含两端），写反自动纠正；体积超上限记 Warning 忽略 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Path", meta = (Keywords = "weight box fill 批量 权重 区域 长方体"))
	void SetVoxelPathWeightBox(FIntVector Min, FIntVector Max, float Weight);

protected:

	UPROPERTY(EditAnywhere, Category = "Voxel", meta = (ToolTip = "单个体素的边长。已有地形数据时不可修改（先点“清除地形”），否则已生成的网格会与坐标尺度对不上。"))
	FVector VoxelSize{ 100.0 };
	/** 体素高度范围（体素为单位，必须是 Voxel::LENGTH 的整数倍） */
	UPROPERTY(EditAnywhere, Category = "Voxel", meta = (ToolTip = "已有地形数据时不可修改（先点“清除地形”），Section 的划分依赖它。"))
	int32 MinHeight = -16;
	UPROPERTY(EditAnywhere, Category = "Voxel", meta = (ToolTip = "已有地形数据时不可修改（先点“清除地形”），Section 的划分依赖它。"))
	int32 MaxHeight = 64;
	UPROPERTY(EditAnywhere, Category = "Voxel")
	TSoftClassPtr<UVoxelGenerator> VoxelGenerator;
	UPROPERTY(EditAnywhere, Category = "Voxel")
	bool bRunGeneratorOnBeginPlay = false;

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
	/* 组件注册完成后补建一次网格：体素数据入档，网格组件不入档 */
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

	friend struct FVoxelPathSolver;

	/*将世界坐标转换为Section坐标和Section内本地坐标*/
	static TPair<FIntVector, FIntVector> WorldCoordToChunkLocalCoord(const FIntVector& WorldCoord);

	FVoxelChunk& FindOrAddChunk(FIntVector2 ChunkCoord);

	/*载入后逐个校验 Chunk 的高度范围，与当前 MinHeight/MaxHeight 不匹配的存档直接丢弃*/
	void ValidateLoadedChunks();

	/*bNeedsMeshBuild 为真时全量重建一次并清标记（组件注册/BeginPlay/Tick 三处都会尝试，谁先到谁做）*/
	void BuildMeshesIfNeeded();

	/* 冷启动竞态防护：AssetManager 的 PrimaryAsset 扫描是异步的，编辑器第一次打开关卡时
	   VoxelType 可能还解析不出来，网格（连同碰撞）会缺失；重建后检测到解析失败就退避重试，
	   直到类型可解析或用完预算（预算耗尽报一次 Error）*/
	void ScheduleMeshBuildRetry();

	double NextMeshBuildTime = 0.0;			// 重试节流：本时间点之前不再真正重建（FPlatformTime::Seconds）
	bool bMeshBuildRetryScheduled = false;
	bool bMeshBuildRetryGaveUp = false;
	int32 MeshBuildRetriesLeft = 8;
	static constexpr double MeshBuildRetryDelay = 1.0;

	UPROPERTY(VisibleAnywhere, Category = "Voxel")
	TObjectPtr<USceneComponent> VoxelRoot;
	UPROPERTY()
	TMap<FIntVector2, FVoxelChunk> Chunks;
	// 待重建的 Section，键是 Chunk 坐标而不是 FVoxelChunk*：Chunks 插入新元素会让 TMap 重排，指针键会全部失效
	TMap<FIntVector2, TArray<FIntVector>> DirtySections;
	/* 载入存档后置真：网格组件不入档，需要按体素数据补建一次 */
	bool bNeedsMeshBuild = false;

	/** 寻路步代价权重表：键=体素坐标，值=乘算系数（0=不可进入，缺省 1）。UPROPERTY 随体素数据一起入档；网格与碰撞不受影响 */
	UPROPERTY()
	TMap<FIntVector, float> PathCostWeights;
};
