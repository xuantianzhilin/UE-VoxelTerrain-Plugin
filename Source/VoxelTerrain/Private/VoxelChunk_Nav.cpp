#include "VoxelChunk.h"
#include "VoxelTerrainActor.h"
#include "Engine/OverlapResult.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"

using namespace Voxel;

namespace
{
	/* 净空向上扫描的格数上限：净空按体素计，128 格远超任何 AI 体型，只是给离谱的 MaxNavHeight 兜个底 */
	constexpr int32 MaxNavClearanceSpan = 128;

	/* 体素格闭区间：Min 是起始角点那一格，Size 是各轴的格数 */
	struct FNavRegion
	{
		FIntVector Min;
		FIntVector Size;

		bool Contains(const FIntVector& Coord) const
		{
			const FIntVector Rel = Coord - Min;
			return Rel.X >= 0 && Rel.Y >= 0 && Rel.Z >= 0
				&& Rel.X < Size.X && Rel.Y < Size.Y && Rel.Z < Size.Z;
		}
	};

	/* 按全局体素坐标读体素占据。净空要沿竖列向上扫，逐格回 Actor 查 Chunk/Section 表太浪费，
	   这里沿途缓存各 Section 指针（不存在的也缓存空指针，免得反复查同一张表） */
	class FNavVoxelReader
	{
	public:
		FNavVoxelReader(const AVoxelTerrainActor* InTerrain, const FVoxelSection* InSelf, const FIntVector& InSelfSection)
			: Terrain(InTerrain)
			, Self(InSelf)
			, SelfSection(InSelfSection)
		{}

		bool IsSolid(const FIntVector& Coord) const
		{
			const FIntVector Section{
				FloorDivide(Coord.X, LENGTH), FloorDivide(Coord.Y, LENGTH), FloorDivide(Coord.Z, LENGTH) };
			const FVoxelSection* Target = Section == SelfSection ? Self : FindSection(Section);
			return Target && !Target->GetVoxel(Coord - Section * LENGTH).IsNone();
		}

	private:
		const FVoxelSection* FindSection(const FIntVector& Section) const
		{
			if (const FVoxelSection* const* Cached = Cache.Find(Section))
			{
				return *Cached;
			}
			return Cache.Add(Section, Terrain->GetChunkSection(Section));
		}

		const AVoxelTerrainActor* Terrain;
		const FVoxelSection* Self;
		FIntVector SelfSection;
		mutable TMap<FIntVector, const FVoxelSection*> Cache;
	};

	/* 落脚判定用的窄带栅格：1 = 该格被体素或静态网格体等外部阻碍占据 */
	struct FNavSolidGrid
	{
		explicit FNavSolidGrid(const FNavRegion& InRegion)
			: Region(InRegion)
		{
			Solid.SetNumZeroed(Region.Size.X * Region.Size.Y * Region.Size.Z);
		}

		bool Contains(const FIntVector& Coord) const { return Region.Contains(Coord); }

		/* 栅格外一律当空处理：外扩量已保证落脚判定不会查到栅格外，真越界说明逻辑有误，
		   宁可漏格也不要凭空挡路 */
		bool IsSolid(const FIntVector& Coord) const
		{
			return Contains(Coord) && Solid[IndexOf(Coord)] != 0;
		}

		void SetSolid(const FIntVector& Coord)
		{
			if (Contains(Coord))
			{
				Solid[IndexOf(Coord)] = 1;
			}
		}

		int32 IndexOf(const FIntVector& Coord) const
		{
			const FIntVector Rel = Coord - Region.Min;
			return (Rel.Z * Region.Size.Y + Rel.Y) * Region.Size.X + Rel.X;
		}

		FNavRegion Region;
		TArray<uint8> Solid;
	};

	/**
	 * 世界 AABB -> 体素坐标闭区间（已夹到查询区域内，与区域不相交时返回空区间）。
	 * 取 [floor(Min), ceil(Max)-1]：边界正好落在格线上时不算进下一格，避免贴地的网格把上方一格也堵死；
	 * 退化包围盒（零厚度平面）仍占据它所在的那一格，免得薄墙面漏成通道。
	 */
	void WorldBoxToCellRange(const AVoxelTerrainActor* Terrain, const FNavRegion& Query, const FBox& Box, FIntVector& OutMin, FIntVector& OutMax)
	{
		const FVector& VoxelSize = Terrain->GetVoxelSize();
		const FTransform& TerrainTransform = Terrain->GetActorTransform();

		// 有旋转时把 8 个角点都换到局部再取包围盒：比只换 Min/Max 保守，但不会漏格
		FVector LocalMin(TNumericLimits<double>::Max());
		FVector LocalMax(TNumericLimits<double>::Lowest());
		for (int32 i = 0; i < 8; ++i)
		{
			const FVector Corner{
				(i & 1) ? Box.Max.X : Box.Min.X,
				(i & 2) ? Box.Max.Y : Box.Min.Y,
				(i & 4) ? Box.Max.Z : Box.Min.Z };
			const FVector Local = TerrainTransform.InverseTransformPosition(Corner) / VoxelSize;
			LocalMin = LocalMin.ComponentMin(Local);
			LocalMax = LocalMax.ComponentMax(Local);
		}

		const FVector QueryLo{ FVector(Query.Min) };
		const FVector QueryHi{ FVector(Query.Min + Query.Size) };
		if (LocalMax.X <= QueryLo.X || LocalMax.Y <= QueryLo.Y || LocalMax.Z <= QueryLo.Z ||
			LocalMin.X >= QueryHi.X || LocalMin.Y >= QueryHi.Y || LocalMin.Z >= QueryHi.Z)
		{
			OutMin = FIntVector(1, 1, 1);
			OutMax = FIntVector(0, 0, 0);		// 空区间：与本区域无关
			return;
		}

		// 先夹到查询区域再取整，超大包围盒（巨物/退化数据）不会溢出，也不会白白遍历整片区域
		LocalMin = LocalMin.ComponentMax(QueryLo);
		LocalMax = LocalMax.ComponentMin(QueryHi);

		OutMin = FIntVector(FMath::FloorToInt(LocalMin.X), FMath::FloorToInt(LocalMin.Y), FMath::FloorToInt(LocalMin.Z));
		OutMax = FIntVector(
			FMath::Max(OutMin.X, FMath::CeilToInt(LocalMax.X - KINDA_SMALL_NUMBER) - 1),
			FMath::Max(OutMin.Y, FMath::CeilToInt(LocalMax.Y - KINDA_SMALL_NUMBER) - 1),
			FMath::Max(OutMin.Z, FMath::CeilToInt(LocalMax.Z - KINDA_SMALL_NUMBER) - 1));
	}

	/**
	 * 把与查询区域相交的静态网格体等阻挡组件光栅化成被占据的体素格。
	 * 骨骼网格体按需求忽略；Pawn 自身的身体/挂件与体素地形自己的 ProceduralMesh 也不算障碍
	 * （否则角色路过就把格子堵住、地形把自己整片堵死）。
	 * 结果只装被挡住的格：净空要一路查到导航天花板，那一整段绝大多数是空的，逐格铺满太浪费。
	 * 用组件（ISM/HISM 用单个实例）的世界 AABB 近似，旋转/凹体网格会偏保守，多占几格。
	 */
	void GatherNavObstacles(const AVoxelTerrainActor* Terrain, const FNavRegion& Query, TSet<FIntVector>& OutCells)
	{
		const UWorld* World = Terrain ? Terrain->GetWorld() : nullptr;
		if (!World)
		{
			return;
		}

		const FVector& VoxelSize = Terrain->GetVoxelSize();
		const FBox RegionBox = FBox(FVector(Query.Min) * VoxelSize, FVector(Query.Min + Query.Size) * VoxelSize)
			.TransformBy(Terrain->GetActorTransform());

		TArray<FOverlapResult> Hits;
		// AllObjects：静态网格体多为 WorldStatic，但阻挡体/其他对象通道也要覆盖，只查一个通道会漏
		World->OverlapMultiByObjectType(Hits, RegionBox.GetCenter(), FQuat::Identity,
			FCollisionObjectQueryParams(FCollisionObjectQueryParams::InitType::AllObjects),
			FCollisionShape::MakeBox(RegionBox.GetExtent()));

		for (const FOverlapResult& Hit : Hits)
		{
			const UPrimitiveComponent* Comp = Hit.Component.Get();
			if (!Comp || Comp->IsA<UProceduralMeshComponent>() || Comp->GetOwner() == Terrain)
			{
				continue;		// 体素地形自己的派生网格永远是 ProceduralMesh，这里再兜一层防类过滤被放大
			}
			if (Comp->IsA<USkeletalMeshComponent>())
			{
				continue;		// 骨骼网格体不参与烘焙
			}
			const AActor* Owner = Comp->GetOwner();
			if (Owner && Owner->IsA<APawn>())
			{
				continue;		// Pawn/Character 的身体与挂件不算静态障碍
			}
			if (!Comp->IsQueryCollisionEnabled() || Comp->GetCollisionResponseToChannel(ECC_Pawn) != ECR_Block)
			{
				continue;		// 无查询碰撞或对 Pawn 只重叠/忽略的组件（触发器、装饰）不挡路
			}

			FBox Body;
			const int32 ItemIndex = Hit.GetItemIndex();
			if (const UInstancedStaticMeshComponent* Ism = Cast<UInstancedStaticMeshComponent>(Comp);
				Ism && Ism->GetStaticMesh() && ItemIndex != INDEX_NONE)
			{
				// ISM/HISM 一次命中一个实例：只光栅化该实例的包围盒，避免整个组件的大 AABB 糊住一整片
				FTransform InstanceTransform;
				if (Ism->GetInstanceTransform(ItemIndex, InstanceTransform, /*bWorldSpace*/ true))
				{
					Body = Ism->GetStaticMesh()->GetBounds().TransformBy(InstanceTransform).GetBox();
				}
				else
				{
					Body = Comp->Bounds.GetBox();
				}
			}
			else
			{
				Body = Comp->Bounds.GetBox();
			}
			if (!Body.IsValid)
			{
				continue;
			}

			FIntVector Lo, Hi;
			WorldBoxToCellRange(Terrain, Query, Body, Lo, Hi);
			for (int32 z = Lo.Z; z <= Hi.Z; ++z)
			{
				for (int32 y = Lo.Y; y <= Hi.Y; ++y)
				{
					for (int32 x = Lo.X; x <= Hi.X; ++x)
					{
						OutCells.Add(FIntVector(x, y, z));
					}
				}
			}
		}
	}

	/**
	 * 从这格起向上数连续的净空格数 = 站进这格的 AI 最高能有多高，数到第一个实心格为止。
	 * 窄带之内查栅格，窄带之外按需查体素与阻碍：只有落脚格要算净空，所以不预先铺一块高栅格再整片扫描。
	 * TopZ 之上没有数据可查，净空就此封顶。
	 */
	int32 ComputeNavClearance(const FNavSolidGrid& Grid, const FNavVoxelReader& Reader,
		const TSet<FIntVector>& Obstacles, const FIntVector& Coord, int32 TopZ)
	{
		int32 Height = 0;
		for (int32 z = Coord.Z; z <= TopZ; ++z)
		{
			const FIntVector Up(Coord.X, Coord.Y, z);
			const bool bBlocked = Grid.Contains(Up)
				? Grid.IsSolid(Up)
				: Reader.IsSolid(Up) || Obstacles.Contains(Up);
			if (bBlocked)
			{
				break;
			}
			++Height;
		}
		return Height;
	}
}

void FVoxelSection::BuildNavData(const AVoxelTerrainActor* Terrain)
{
	NavData.Reset();
	if (!Terrain)
	{
		return;
	}

	// 落脚判定用的窄带：本 Section 外扩一圈，X/Y 各 ±1 覆盖水平 4 邻格，
	// Z 下扩 LinkHeight+1 覆盖邻格的支撑格（判断“可站立”要往下看一格），上扩 LinkHeight 覆盖台阶。
	const FIntVector SectionOrigin = SectionCoord * LENGTH;
	const int32 BottomZ = SectionOrigin.Z - (LinkHeight + 1);
	const int32 HighestStandableZ = SectionOrigin.Z + LENGTH + LinkHeight;	// 邻格可能站到的最高一层
	const int32 TopZ = FMath::Clamp(FMath::Max(Terrain->GetMaxNavHeight(), HighestStandableZ),
		HighestStandableZ, HighestStandableZ + MaxNavClearanceSpan);			// 净空扫描的天花板

	const int32 XYSize = LENGTH + 2;
	const FIntVector PaddedMin(SectionOrigin.X - 1, SectionOrigin.Y - 1, BottomZ);
	const FNavRegion StandRegion{ PaddedMin, FIntVector(XYSize, XYSize, HighestStandableZ - BottomZ + 1) };
	// 阻碍查询要一直铺到天花板：头顶的桥、屋檐会压低下方落脚格的净空
	const FNavRegion QueryRegion{ PaddedMin, FIntVector(XYSize, XYSize, TopZ - BottomZ + 1) };

	// 1) 静态网格体等外部阻碍（只记下被挡住的格）
	TSet<FIntVector> Obstacles;
	GatherNavObstacles(Terrain, QueryRegion, Obstacles);

	// 2) 体素 + 阻碍 -> 窄带实心栅格
	const FNavVoxelReader Reader(Terrain, this, SectionCoord);
	FNavSolidGrid Grid(StandRegion);
	for (int32 z = 0; z < StandRegion.Size.Z; ++z)
	{
		for (int32 y = 0; y < StandRegion.Size.Y; ++y)
		{
			for (int32 x = 0; x < StandRegion.Size.X; ++x)
			{
				const FIntVector Coord = StandRegion.Min + FIntVector(x, y, z);
				if (Reader.IsSolid(Coord) || Obstacles.Contains(Coord))
				{
					Grid.SetSolid(Coord);
				}
			}
		}
	}

	// 落脚格：自身为空（无体素也无外部阻碍），下方有支撑（体素或被网格占据的格都算）。
	// 一条竖直通道里的每一层地面（地表、洞穴地板）都会各自成为一个节点。
	auto IsStandable = [&Grid](const FIntVector& Coord)
		{
			return !Grid.IsSolid(Coord) && Grid.IsSolid(Coord - FIntVector(0, 0, 1));
		};

	// 3) 每个落脚格只连接水平 4 邻：邻格与自己的高度差在 LinkHeight 内即视为相邻（可以上/下一格台阶）
	static const FIntVector Directions[4] = {
		FIntVector(1, 0, 0), FIntVector(-1, 0, 0), FIntVector(0, 1, 0), FIntVector(0, -1, 0) };

	for (int32 z = 0; z < LENGTH; ++z)
	{
		for (int32 y = 0; y < LENGTH; ++y)
		{
			for (int32 x = 0; x < LENGTH; ++x)
			{
				const FIntVector LocalCoord(x, y, z);
				const FIntVector Coord = SectionOrigin + LocalCoord;
				if (!IsStandable(Coord))
				{
					continue;
				}

				// 可站但没有邻居的格也要留一条记录：Key 存在即代表这一格能站
				FVoxelNavCell& Cell = NavData.Add(LocalCoord);
				Cell.AllowHeight = ComputeNavClearance(Grid, Reader, Obstacles, Coord, TopZ);

				for (const FIntVector& Direction : Directions)
				{
					for (int32 DeltaZ = -LinkHeight; DeltaZ <= LinkHeight; ++DeltaZ)
					{
						const FIntVector Neighbor = Coord + Direction + FIntVector(0, 0, DeltaZ);
						if (!IsStandable(Neighbor))
						{
							continue;
						}

						// 邻格可能落在相邻 Section：用 (所属 Section, 该 Section 内的局部坐标) 定位它，
						// 它自己的净空读它那条记录（NavData[LinkCoord].AllowHeight）
						const FIntVector NeighborSection{
							FloorDivide(Neighbor.X, LENGTH),
							FloorDivide(Neighbor.Y, LENGTH),
							FloorDivide(Neighbor.Z, LENGTH) };

						FVoxelNavLink& Link = Cell.Links.AddDefaulted_GetRef();
						Link.OwnnerSection = NeighborSection;
						Link.LinkCoord = Neighbor - NeighborSection * LENGTH;
					}
				}
			}
		}
	}
}
