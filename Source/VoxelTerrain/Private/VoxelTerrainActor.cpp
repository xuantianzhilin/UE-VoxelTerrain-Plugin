#include "VoxelTerrainActor.h"
#include "VoxelType.h"
#include "VoxelGenerator.h"
#include "Components/SceneComponent.h"
#include "Engine/AssetManager.h"

AVoxelTerrainActor::AVoxelTerrainActor()
{
	PrimaryActorTick.bCanEverTick = true;

	VoxelRoot = CreateDefaultSubobject<USceneComponent>(TEXT("VoxelRoot"));

	RootComponent = VoxelRoot;
}

FIntVector AVoxelTerrainActor::WorldLocationToCoord(const FVector& WorldLocation) const
{
	FVector LocalLocation = GetActorTransform().InverseTransformPosition(WorldLocation);
	LocalLocation /= VoxelSize;
	const int32 X = FMath::FloorToInt32(LocalLocation.X);
	const int32 Y = FMath::FloorToInt32(LocalLocation.Y);
	const int32 Z = FMath::FloorToInt32(LocalLocation.Z);
	return { X, Y, Z };
}

FVector AVoxelTerrainActor::CoordToWorldLocation(FIntVector Coord) const
{
	static const FVector HalfVoxelSize{ 0.5 };
	const FVector LocalPos = (FVector{ Coord } + HalfVoxelSize) * VoxelSize;
	return GetActorTransform().TransformPosition(LocalPos);
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
void AVoxelTerrainActor::EditorRunDefaultGenerator()
{
	// CallInEditor 外层已有事务，Modify() 让被改到的 UPROPERTY 状态可撤销
	Modify();
	RunDefaultGenerator();
	RebuildAllSections();
	MarkPackageDirty();
}

void AVoxelTerrainActor::EditorClearTerrain()
{
	Modify();
	ClearTerrain();
	MarkPackageDirty();
}

void AVoxelTerrainActor::EditorRebuildAllSections()
{
	RebuildAllSections();
}
#endif

void AVoxelTerrainActor::BuildNavData()
{
	if (Chunks.IsEmpty()) return;

	// Section 划分只由 MinHeight/MaxHeight 决定（构造 Chunk 时已保证 MinHeight 是 LENGTH 的整数倍），
	// 每个 Chunk 的 Z 覆盖范围都一样，直接按全局 Section Z 取即可
	const int32 FirstSectionZ = MinHeight / Voxel::LENGTH;
	const int32 SectionCount = (MaxHeight - MinHeight) / Voxel::LENGTH;

	int32 SectionCountBaked = 0;
	for (auto& [ChunkCoord, Chunk] : Chunks)
	{
		for (int32 i = 0; i < SectionCount; ++i)
		{
			if (FVoxelSection* Section = Chunk.GetSection(FirstSectionZ + i))
			{
				Section->BuildNavData(this);
				++SectionCountBaked;
			}
		}
	}

	UE_LOG(LogTemp, Verbose, TEXT("[Voxel] %s：已烘焙 %d 个 Section 的导航数据"),
		*GetName(), SectionCountBaked);
}

void AVoxelTerrainActor::Serialize(FArchive& Ar)
{
	// 对象复制（进 PIE / 复制 Actor）和撤销记录不是包体 IO，那种场合数据本来就整体拷贝。
	const bool bIsPackageIO = Ar.IsPersistent()
		&& !Ar.HasAnyPortFlags(PPF_Duplicate | PPF_DuplicateForPIE) && !Ar.IsTransacting();

	Super::Serialize(Ar);

	if (Ar.IsLoading() && bIsPackageIO)
	{
		DirtySections.Reset();
		// 网格组件是派生数据（RF_Transient，不入档），载入后需要按体素数据补建一次
		bNeedsMeshBuild = !Chunks.IsEmpty();
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
	TMap<FIntVector, int32> ChangeMap;
	for (const auto& Coord : Coords)
	{
		if (Coord.Z < MinHeight || Coord.Z >= MaxHeight)
		{
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
				Chunk->BuildMesh(this, SectionCoord.Z);
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
		Chunk.BuildAllMeshes(this);
	}
}

void AVoxelTerrainActor::ClearTerrain()
{
	DirtySections.Reset();
	for (auto& [ChunkCoord, Chunk] : Chunks)
	{
		Chunk.ClearAllMeshes();
	}
	Chunks.Reset();
}

bool AVoxelTerrainActor::LineSingleTraceVoxel(const FVector& Start, const FVector& End, FVoxelTraceHit& OutHit)
{
	if ((End - Start).IsNearlyZero()) return false;

	const FVector P0 = GetActorTransform().InverseTransformPosition(Start) / VoxelSize;
	const FVector V = GetActorTransform().InverseTransformVector(End) / VoxelSize - P0;

	double TMax[3];
	double TDelta[3];
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		if (FMath::IsNearlyZero(V[Axis]))
		{
			TMax[Axis] = TNumericLimits<double>::Max();
			TDelta[Axis] = TNumericLimits<double>::Max();
		}
		else
		{
			const double Step = FMath::Sign(V[Axis]);
			const double NextVoxelBoundary = Step > 0.0 ? FMath::FloorToDouble(P0[Axis]) + 1.0 : FMath::CeilToDouble(P0[Axis]) - 1.0;
			TMax[Axis] = (NextVoxelBoundary - P0[Axis]) / V[Axis];
			TDelta[Axis] = Step / V[Axis];
		}
	}

	constexpr int32 MaxSteps = 10000;
	int32 Steps = 0;

	// 坐标必须是 int32：FMath::FloorToInt(double) 返回 int64，花括号初始化会因收窄转换报 C2398
	FIntVector Coord{ FMath::FloorToInt32(P0.X), FMath::FloorToInt32(P0.Y), FMath::FloorToInt32(P0.Z) };
	const FIntVector Delta{ static_cast<int32>(FMath::Sign(V.X)), static_cast<int32>(FMath::Sign(V.Y)), static_cast<int32>(FMath::Sign(V.Z)) };
	while (Steps++ < MaxSteps)
	{
		const int32 Axis = (TMax[0] <= TMax[1])
			? ((TMax[0] <= TMax[2]) ? 0 : 2)
			: ((TMax[1] <= TMax[2]) ? 1 : 2);

		FVoxelState Voxel = GetVoxel(Coord);
		if (!Voxel.IsNone())
		{
			FVector LocalNormal = FVector::ZeroVector;
			LocalNormal[Axis] = -Delta[Axis];

			OutHit.bHit = true;
			OutHit.Coord = Coord;
			OutHit.Normal = GetActorTransform().TransformVectorNoScale(LocalNormal);
			OutHit.ImpactPoint = GetActorTransform().TransformPosition((P0 + V * TMax[Axis]) * VoxelSize);
			OutHit.Voxel = MoveTemp(Voxel);
			return true;
		}

		Coord[Axis] += Delta[Axis];
		TMax[Axis] += TDelta[Axis];

		if (TMax[Axis] > 1.0)
		{
			break; // 超出线段范围
		}
		if (P0.X < MinHeight || P0.X >= MaxHeight ||
			P0.Y < MinHeight || P0.Y >= MaxHeight ||
			P0.Z < MinHeight || P0.Z >= MaxHeight)
		{
			break; // 超出高度范围
		}
	}
	return false;
}

void AVoxelTerrainActor::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();

#if WITH_EDITOR

	// 编辑器世界不跑 BeginPlay，补建只能挂在这里；游戏/PIE 统一交给 BeginPlay，免得重建两遍。
	// （对象复制不会带上 bNeedsMeshBuild 这个 C++ 成员，所以 PIE 必须靠 BeginPlay 兜底）
	const UWorld* World = GetWorld();
	if (!World || (World->WorldType != EWorldType::Editor && World->WorldType != EWorldType::EditorPreview))
	{
		return;
	}

	BuildMeshesIfNeeded();
#endif
}

void AVoxelTerrainActor::BeginPlay()
{
	Super::BeginPlay();

	// 运行时一律保证网格与体素一致：数据可能是存档带进来的，也可能是生成器刚写的
	RebuildAllSections();

	if (bRunGeneratorOnBeginPlay)
	{
		RunDefaultGenerator();
	}
}

void AVoxelTerrainActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

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

	UAssetManager::CallOrRegister_OnCompletedInitialScan(
		FSimpleMulticastDelegate::FDelegate::CreateWeakLambda(this, [this]()
			{
				bNeedsMeshBuild = false;
				RebuildAllSections();
			}));
}

TPair<FIntVector, FIntVector> AVoxelTerrainActor::WorldCoordToChunkLocalCoord(const FIntVector& WorldCoord)
{
	// 体素坐标 -> Section 坐标 + Section 内体素坐标，FloorDivide 保证负坐标向 -inf 取整
	const FIntVector SectionCoord{
		Voxel::FloorDivide(WorldCoord.X, Voxel::LENGTH),
		Voxel::FloorDivide(WorldCoord.Y, Voxel::LENGTH),
		Voxel::FloorDivide(WorldCoord.Z, Voxel::LENGTH) };

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

	return Chunks.Emplace(ChunkCoord, FVoxelChunk{ ChunkCoord, MinHeight, MaxHeight });
}
