#include "VoxelTerrainActor.h"
#include "VoxelType.h"
#include "VoxelGenerator.h"
#include "Components/SceneComponent.h"
#include "Containers/Ticker.h"

namespace
{
	// 向 -inf 取整的整数除法：C++ 的 "/" 向 0 截断，负坐标会把 Section 索引/本地坐标算错
	int32 FloorDivide(int32 Dividend, int32 Divisor)
	{
		check(Divisor > 0);
		return Dividend >= 0 ? Dividend / Divisor : (Dividend - Divisor + 1) / Divisor;
	}
}

AVoxelTerrainActor::AVoxelTerrainActor()
{
	PrimaryActorTick.bCanEverTick = true;

	VoxelRoot = CreateDefaultSubobject<USceneComponent>(TEXT("VoxelRoot"));

	RootComponent = VoxelRoot;
}

FIntVector AVoxelTerrainActor::WorldLocationToCoord(const FVector& WorldLocation) const
{
	FVector LocalLocation = (WorldLocation - GetActorLocation()) / VoxelSize;
	const int32 X = FMath::FloorToInt(LocalLocation.X);
	const int32 Y = FMath::FloorToInt(LocalLocation.Y);
	const int32 Z = FMath::FloorToInt(LocalLocation.Z);
	return { X, Y, Z };
}

FVector AVoxelTerrainActor::CoordToWorldLocation(FIntVector Coord) const
{
	static const FVector HalfVoxelSize{ 0.5 };
	return GetActorLocation() + (FVector{ Coord } + HalfVoxelSize) * VoxelSize;
}

const FVoxelSection* AVoxelTerrainActor::GetChunkSection(FIntVector SectionCoord) const
{
	return const_cast<AVoxelTerrainActor*>(this)->GetChunkSection(SectionCoord);
}

FVoxelSection* AVoxelTerrainActor::GetChunkSection(FIntVector SectionCoord)
{
	if (FVoxelChunk* Chunk = Chunks.Find(FIntVector2{ SectionCoord }))
	{
		return Chunk->GetSection(SectionCoord.Z);
	}
	return nullptr;
}

void AVoxelTerrainActor::RunDefaultGenerator()
{
	if (!VoxelGenerator.IsNull())
	{
		UClass* GeneratorClass = VoxelGenerator.IsValid() ? VoxelGenerator.Get() : VoxelGenerator.LoadSynchronous();
		RunGenerator(GeneratorClass);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("未指定默认体素生成器"));
	}
}

void AVoxelTerrainActor::RunGenerator(TSubclassOf<UVoxelGenerator> GeneratorClass)
{
	const UVoxelGenerator* Generator = GeneratorClass ? GeneratorClass->GetDefaultObject<UVoxelGenerator>() : nullptr;
	if (!Generator)
	{
		UE_LOG(LogTemp, Warning, TEXT("体素生成器无效，跳过生成"));
		return;
	}

	Generator->GenerateVoxel(this);
}

#if WITH_EDITOR
void AVoxelTerrainActor::GenerateDefaultTerrain()
{
	// CallInEditor 外层已有事务，Modify() 让被改到的 UPROPERTY 状态可撤销
	Modify();

	RunDefaultGenerator();

	// 编辑器世界默认不 Tick，也不走脏区分摊；这里强制重建一遍，
	// 保证重复点击（数据未变化、或网格组件曾被销毁）也能拿到和 Chunk 数据一致的画面。
	// 走 BuildMeshesIfNeeded 是为了复用“VoxelType 解析失败 → 退避重试”的冷启动防护；手动点击重置预算
	bNeedsMeshBuild = true;
	NextMeshBuildTime = 0.0;
	MeshBuildRetriesLeft = 8;
	bMeshBuildRetryGaveUp = false;
	BuildMeshesIfNeeded();

	MarkPackageDirty();
}

void AVoxelTerrainActor::ClearTerrain()
{
	Modify();

	// DirtySections 缓存的是 FVoxelChunk*，必须在 Chunk 销毁之前一起清掉，否则会留下野指针
	DirtySections.Reset();
	for (auto& [ChunkCoord, Chunk] : Chunks)
	{
		Chunk.ClearAllMeshes();
	}
	Chunks.Reset();

	MarkPackageDirty();
}

void AVoxelTerrainActor::RebuildTerrainMesh()
{
	// 同 GenerateDefaultTerrain：走 BuildMeshesIfNeeded 复用解析失败的退避重试，手动点击重置预算
	bNeedsMeshBuild = true;
	NextMeshBuildTime = 0.0;
	MeshBuildRetriesLeft = 8;
	bMeshBuildRetryGaveUp = false;
	BuildMeshesIfNeeded();
}
#endif

void AVoxelTerrainActor::Serialize(FArchive& Ar)
{
	// 对象复制（进 PIE / 复制 Actor）和撤销记录不是包体 IO，那种场合数据本来就整体拷贝。
	const bool bIsPackageIO = Ar.IsPersistent()
		&& !Ar.HasAnyPortFlags(PPF_Duplicate | PPF_DuplicateForPIE) && !Ar.IsTransacting();

	Super::Serialize(Ar);

	if (Ar.IsLoading() && bIsPackageIO)
	{
		DirtySections.Reset();
		ValidateLoadedChunks();

		// 网格组件是派生数据（RF_Transient，不入档），载入后需要按体素数据补建一次
		bNeedsMeshBuild = !Chunks.IsEmpty();
	}
}

void AVoxelTerrainActor::ValidateLoadedChunks()
{
	for (auto It = Chunks.CreateIterator(); It; ++It)
	{
		if (!It->Value.SyncHeightRange(MinHeight, MaxHeight))
		{
			UE_LOG(LogTemp, Warning, TEXT("%s：Chunk (%d,%d) 的地形存档与当前高度范围 [%d,%d) 不匹配，已丢弃"),
				*GetName(), It->Key.X, It->Key.Y, MinHeight, MaxHeight);
			It->Value.ClearAllMeshes();		// 网格组件是随存档回来的，丢数据就得连带销毁组件
			It.RemoveCurrent();
		}
	}
}

#if WITH_EDITOR
bool AVoxelTerrainActor::CanEditChange(const FProperty* InProperty) const
{
	if (InProperty && !Chunks.IsEmpty())
	{
		// 这三项决定了体素尺度与 Section 的划分，有数据之后再改会让已存的体素和网格整体错位
		const FName Name = InProperty->GetFName();
		if (Name == GET_MEMBER_NAME_CHECKED(AVoxelTerrainActor, VoxelSize) ||
			Name == GET_MEMBER_NAME_CHECKED(AVoxelTerrainActor, MinHeight) ||
			Name == GET_MEMBER_NAME_CHECKED(AVoxelTerrainActor, MaxHeight))
		{
			return false;
		}
	}

	return Super::CanEditChange(InProperty);
}
#endif

FVoxelState AVoxelTerrainActor::GetVoxel(FIntVector Coord) const
{
	const auto [SectionCoord, LocalCoord] = WorldCoordToChunkLocalCoord(Coord);
	const FVoxelChunk* Chunk = Chunks.Find(FIntVector2{ SectionCoord });
	const FVoxelSection* Section = Chunk ? Chunk->GetSection(SectionCoord.Z) : nullptr;
	return Section ? Section->GetVoxel(LocalCoord) : FVoxelState{};
}

void AVoxelTerrainActor::SetVoxel(FIntVector Coord, FName TypeName, const FRotator& Rotation)
{
	SetVoxels({ Coord }, TypeName, Rotation);
}

void AVoxelTerrainActor::SetVoxels(const TArray<FIntVector>& Coords, FName TypeName, const FRotator& Rotation)
{
	// 键用 Section 坐标：FindOrAddChunk 可能插入新元素导致 TMap 重排，缓存的 FVoxelSection* 会失效
	TMap<FIntVector, int32> ChangeMap;
	for (const auto& Coord : Coords)
	{
		if (Coord.Z < MinHeight || Coord.Z >= MaxHeight)
		{
			// 批量写入里只要有一个越界点，也不能把后面合法的体素一起丢掉（原来这里是 return，会顺带跳过 Compact 与重建）
			UE_LOG(LogTemp, Warning, TEXT("Voxel (%d,%d,%d) 超出高度范围 [%d,%d)，已忽略"),
				Coord.X, Coord.Y, Coord.Z, MinHeight, MaxHeight);
			continue;
		}

		const auto [SectionCoord, LocalCoord] = WorldCoordToChunkLocalCoord(Coord);
		FVoxelChunk& Chunk = FindOrAddChunk(FIntVector2{ SectionCoord });
		if (FVoxelSection* Section = Chunk.GetSection(SectionCoord.Z))
		{
			if (Section->SetVoxel(LocalCoord, FVoxelState{ TypeName, Rotation }))
			{
				++ChangeMap.FindOrAdd(SectionCoord, 0);
				MarkSectionDirty(SectionCoord, LocalCoord);
			}
		}
	}

	for (const auto& [SectionCoord, Count] : ChangeMap)
	{
		if (Count > CompactThreshold)
		{
			if (FVoxelSection* Section = GetChunkSection(SectionCoord))
			{
				Section->Compact();
			}
		}
	}
}

void AVoxelTerrainActor::FillBox(FIntVector Min, FIntVector Max, FName TypeName, const FRotator& Rotation)
{
	const FIntVector Lo(FMath::Min(Min.X, Max.X), FMath::Min(Min.Y, Max.Y), FMath::Min(Min.Z, Max.Z));
	const FIntVector Hi(FMath::Max(Min.X, Max.X), FMath::Max(Min.Y, Max.Y), FMath::Max(Min.Z, Max.Z));

	// 与有效高度范围求交集：越界的部分裁掉即可，不像逐点写入那样需要告警
	const int32 LowZ = FMath::Max(Lo.Z, MinHeight);
	const int32 HighZ = FMath::Min(Hi.Z, MaxHeight - 1);
	if (LowZ > HighZ)
	{
		return;
	}

	const int64 Volume = static_cast<int64>(Hi.X - Lo.X + 1) * (Hi.Y - Lo.Y + 1) * (HighZ - LowZ + 1);
	if (Volume > MaxBulkVoxelCount)
	{
		UE_LOG(LogTemp, Warning, TEXT("FillBox 请求 %lld 个体素，超过单次上限 %lld，已忽略"), Volume, MaxBulkVoxelCount);
		return;
	}

	TArray<FIntVector> Coords;
	Coords.Reserve(static_cast<int32>(Volume));
	for (int32 z = LowZ; z <= HighZ; ++z)
	{
		for (int32 y = Lo.Y; y <= Hi.Y; ++y)
		{
			for (int32 x = Lo.X; x <= Hi.X; ++x)
			{
				Coords.Emplace(x, y, z);
			}
		}
	}

	SetVoxels(Coords, TypeName, Rotation);
}

void AVoxelTerrainActor::FillSphere(FIntVector Center, double Radius, FName TypeName, const FRotator& Rotation, bool bSolid)
{
	// 半径先限到一个包围盒必然已经超上限的量级，避免下面 ±Extent 的索引与 int64 计数溢出
	constexpr int32 MaxSphereExtent = 1024;
	if (Radius < 0.0 || Radius > MaxSphereExtent)
	{
		UE_LOG(LogTemp, Warning, TEXT("FillSphere 半径非法（%f，允许 0~%d），已忽略"), Radius, MaxSphereExtent);
		return;
	}

	const int32 Extent = FMath::CeilToInt(Radius);
	const int32 LowZ = FMath::Max(Center.Z - Extent, MinHeight);
	const int32 HighZ = FMath::Min(Center.Z + Extent, MaxHeight - 1);
	if (LowZ > HighZ)
	{
		return;
	}

	// 用包围盒计数做保护：球只占其中约一半，判定偏保守
	const int64 BoundedVolume = static_cast<int64>(Extent * 2 + 1) * (Extent * 2 + 1) * (HighZ - LowZ + 1);
	if (BoundedVolume > MaxBulkVoxelCount)
	{
		UE_LOG(LogTemp, Warning, TEXT("FillSphere 包围盒 %lld 个体素，超过单次上限 %lld，已忽略"), BoundedVolume, MaxBulkVoxelCount);
		return;
	}

	const double RadiusSquared = Radius * Radius;
	const double InnerSquared = FMath::Square(Radius - 1.0);	// 球壳的内边界，只留最外 1 体素厚

	TArray<FIntVector> Coords;
	Coords.Reserve(static_cast<int32>(BoundedVolume));
	for (int32 z = LowZ; z <= HighZ; ++z)
	{
		for (int32 y = Center.Y - Extent; y <= Center.Y + Extent; ++y)
		{
			for (int32 x = Center.X - Extent; x <= Center.X + Extent; ++x)
			{
				// 以“体素索引到球心的距离”判定：关于球心完全对称，也与遍历顺序无关
				const double DistSquared = FMath::Square(static_cast<double>(x - Center.X))
					+ FMath::Square(static_cast<double>(y - Center.Y))
					+ FMath::Square(static_cast<double>(z - Center.Z));
				if (DistSquared > RadiusSquared || (!bSolid && DistSquared < InnerSquared))
				{
					continue;
				}
				Coords.Emplace(x, y, z);
			}
		}
	}

	SetVoxels(Coords, TypeName, Rotation);
}

void AVoxelTerrainActor::MarkSectionDirty(FIntVector SectionCoord, FIntVector LocalCoord)
{
	// 本 Section 必然要重建；改动落在边界上时，相邻 Section 的遮挡判断结果也会变，同样要重建
	FIntVector SectionCoords[7] = { SectionCoord };
	int32 Num = 1;
	auto AddNeighbor = [&](const FIntVector& Offset)
	{
		SectionCoords[Num++] = SectionCoord + Offset;
	};

	if (LocalCoord.X == 0)					AddNeighbor({ -1, 0, 0 });
	if (LocalCoord.X == Voxel::LENGTH - 1)	AddNeighbor({ 1, 0, 0 });
	if (LocalCoord.Y == 0)					AddNeighbor({ 0, -1, 0 });
	if (LocalCoord.Y == Voxel::LENGTH - 1)	AddNeighbor({ 0, 1, 0 });
	if (LocalCoord.Z == 0)					AddNeighbor({ 0, 0, -1 });
	if (LocalCoord.Z == Voxel::LENGTH - 1)	AddNeighbor({ 0, 0, 1 });
	// Z 方向相邻 Section 必然属于同一个 Chunk（Chunk 只按 X/Y 划分），超出高度范围时 BuildMesh 内部会跳过

	for (int32 i = 0; i < Num; ++i)
	{
		// 不存在的相邻 Chunk 本来也没有要重建的网格；键一律用坐标，
		// 因为 FindOrAddChunk 插入新元素会让 TMap 重排，任何 FVoxelChunk* 键都会失效
		const FIntVector2 NeighborChunkCoord{ SectionCoords[i].X, SectionCoords[i].Y };
		if (Chunks.Contains(NeighborChunkCoord))
		{
			DirtySections.FindOrAdd(NeighborChunkCoord).AddUnique(SectionCoords[i]);
		}
	}
}

void AVoxelTerrainActor::RebuildDirtySections(int32 MaxCount)
{
	int32 Budget = MaxCount <= 0 ? TNumericLimits<int32>::Max() : MaxCount;
	for (auto It = DirtySections.CreateIterator(); It && Budget > 0;)
	{
		FVoxelChunk* Chunk = Chunks.Find(It->Key);
		TArray<FIntVector>& Pending = It->Value;

		while (Pending.Num() > 0 && Budget > 0)
		{
			const FIntVector SectionCoord = Pending.Pop();
			if (Chunk)
			{
				Chunk->BuildMesh(this, SectionCoord);
			}
			--Budget;
		}

		// 只有处理干净了才摘掉这一项，预算用尽时剩下的留给下一帧
		if (Pending.Num() == 0)
		{
			It.RemoveCurrent();
		}
	}
}

void AVoxelTerrainActor::RebuildAllSections()
{
	// 直接重建所有 Section：脏区队列在"数据没变"时是空的，光靠它无法把被销毁的网格组件恢复回来。
	// 空 Section 会在 BuildMeshData 里立刻返回，代价可以接受。
	DirtySections.Reset();
	for (auto& [ChunkCoord, Chunk] : Chunks)
	{
		Chunk.BuildAllMeshes(this, ChunkCoord);
	}
}

void AVoxelTerrainActor::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();

	// 编辑器世界不跑 BeginPlay，补建只能挂在这里；游戏/PIE 统一交给 BeginPlay，免得重建两遍。
	// （对象复制不会带上 bNeedsMeshBuild 这个 C++ 成员，所以 PIE 必须靠 BeginPlay 兜底）
	const UWorld* World = GetWorld();
	if (!World || (World->WorldType != EWorldType::Editor && World->WorldType != EWorldType::EditorPreview))
	{
		return;
	}

	BuildMeshesIfNeeded();
}

void AVoxelTerrainActor::BeginPlay()
{
	Super::BeginPlay();

	if (bRunGeneratorOnBeginPlay)
	{
		RunDefaultGenerator();
	}

	// 运行时一律保证网格与体素一致：数据可能是存档带进来的，也可能是生成器刚写的
	bNeedsMeshBuild = true;
	BuildMeshesIfNeeded();
}

void AVoxelTerrainActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// 兜底：上面两个回调都没走到时（编辑器世界的组件注册时机因路径而异），第一帧 Tick 补建
	BuildMeshesIfNeeded();

	if (DirtySections.Num() > 0)
	{
		RebuildDirtySections(MaxRebuildCountPerTick);
	}
}

void AVoxelTerrainActor::BuildMeshesIfNeeded()
{
	if (!bNeedsMeshBuild)
	{
		return;
	}
	// 重试节流窗口内不重复全量重建（ticker 与 Tick 两条路都会打进来）
	const double Now = FPlatformTime::Seconds();
	if (Now < NextMeshBuildTime)
	{
		return;
	}

	bNeedsMeshBuild = false;		// 一次性消费，避免每次重跑 Construction Script 都全量重建
	Voxel::VoxelTypeResolveFailures() = 0;
	RebuildAllSections();

	if (Voxel::VoxelTypeResolveFailures() > 0)
	{
		if (MeshBuildRetriesLeft > 0)
		{
			// 多半是 AssetManager 首轮 PrimaryAsset 扫描没完成：保持标记，延迟后再重建一次
			--MeshBuildRetriesLeft;
			bNeedsMeshBuild = true;
			NextMeshBuildTime = Now + MeshBuildRetryDelay;
			ScheduleMeshBuildRetry();
		}
		else if (!bMeshBuildRetryGaveUp)
		{
			bMeshBuildRetryGaveUp = true;
			UE_LOG(LogTemp, Error, TEXT("%s：VoxelType 解析重试已用尽，地形网格/碰撞不完整——请核对 DA 资产的 TypeName 与 /Game/Voxel 是否在 PrimaryAssetTypesToScan 扫描目录内"), *GetName());
		}
	}
}

void AVoxelTerrainActor::ScheduleMeshBuildRetry()
{
	if (bMeshBuildRetryScheduled)
	{
		return;
	}
	bMeshBuildRetryScheduled = true;
	TWeakObjectPtr<AVoxelTerrainActor> WeakSelf(this);
	FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([WeakSelf](float)
	{
		AVoxelTerrainActor* Self = WeakSelf.Get();
		if (!Self)
		{
			return false;		// Actor 已销毁，组件随它走了
		}
		Self->bMeshBuildRetryScheduled = false;
		Self->BuildMeshesIfNeeded();
		return false;			// 一次性；若仍失败，BuildMeshesIfNeeded 会再挂一个新的
	}), MeshBuildRetryDelay);
}

TPair<FIntVector, FIntVector> AVoxelTerrainActor::WorldCoordToChunkLocalCoord(const FIntVector& WorldCoord)
{
	// 体素坐标 -> Section 坐标 + Section 内体素坐标，FloorDivide 保证负坐标向 -inf 取整
	const FIntVector SectionCoord{
		FloorDivide(WorldCoord.X, Voxel::LENGTH),
		FloorDivide(WorldCoord.Y, Voxel::LENGTH),
		FloorDivide(WorldCoord.Z, Voxel::LENGTH) };

	const FIntVector LocalCoord{
		WorldCoord.X - SectionCoord.X * Voxel::LENGTH,
		WorldCoord.Y - SectionCoord.Y * Voxel::LENGTH,
		WorldCoord.Z - SectionCoord.Z * Voxel::LENGTH };

	return { SectionCoord, LocalCoord };
}

FVoxelChunk& AVoxelTerrainActor::FindOrAddChunk(FIntVector2 ChunkCoord)
{
	if (FVoxelChunk* Find = Chunks.Find(ChunkCoord))
	{
		return *Find;
	}

	return Chunks.Emplace(ChunkCoord, FVoxelChunk{ MinHeight, MaxHeight });
}
