#include "VoxelChunk.h"
#include "VoxelTerrainActor.h"
#include "Containers/Deque.h"		// 滑窗 min 的单调队列
#include "Engine/OverlapResult.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/PrimitiveComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Pawn.h"

using namespace Voxel;

namespace
{
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
	 * 最多数到 MaxAllowHeight 格就封顶（"至少这么高"）：封顶值不超过一层，改一体素时受影响的下层
	 * 落脚格才一定落在紧邻的那一层里，脏区标记（MarkSectionDirty）才能跟着网格用同一套规则。
	 */
	int32 ComputeNavClearance(const FNavSolidGrid& Grid, const FNavVoxelReader& Reader,
		const TSet<FIntVector>& Obstacles, const FIntVector& Coord, int32 MaxAllowHeight)
	{
		int32 Height = 0;
		const int32 LastZ = Coord.Z + MaxAllowHeight - 1;
		for (int32 z = Coord.Z; z <= LastZ; ++z)
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

	/* 滑窗 min 用的一维单调队列：窗口 = [i - Half, i - Half + W - 1]，越出范围的输出处记 0（=不可站）。
	   Src/Dst 沿一条直线取元素，取索引方式是模板参数（横排连续、纵列跨行、层间跨层）。 */
	template <typename TGetter>
	void SlidingMin1D(TGetter Src, TArray<uint8>& Dst, int32 Count, int32 W, int32 Half)
	{
		Dst.SetNumUninitialized(Count);
		TDeque<int32> Window;	// 存下标，队首恒为窗口内最小值
		for (int32 i = 0; i < Count; ++i)
		{
			const int32 Right = i - Half + W - 1;
			if (Right >= 0 && Right < Count)
			{
				const uint8 Val = Src(Right);
				while (!Window.IsEmpty() && Src(Window.Last()) >= Val)
				{
					Window.PopLast();
				}
				Window.PushLast(Right);
			}
			const int32 Left = i - Half;
			while (!Window.IsEmpty() && Window.First() < Left)
			{
				Window.PopFirst();		// 队首即窗口左沿之外的最旧下标
			}
			Dst[i] = (Left < 0 || Right >= Count || Window.IsEmpty()) ? 0 : Src(Window.First());
		}
	}

	/**
	 * 按体型 W×W 对一层「落脚净空图」做 min 侵蚀（两遍一维滑窗 min，先横后纵）。
	 * 结果 = footprint 以 (x,y) 为 anchor、覆盖 [x-W/2, x-W/2+W-1] × 同 Y 时每列净空的 min；
	 * 任一覆盖列为 0（不可站）或窗口越出平面则结果为 0。读写可指向同一对大数组里的第 Base 层。
	 */
	void ErodePlane(const TArray<uint8>& In, TArray<uint8>& Out, TArray<uint8>& Temp,
		int32 Base, int32 SizeX, int32 SizeY, int32 W, int32 Half)
	{
		// 横向：逐行滑窗，结果按行写回 Out 同一层（纵向遍以此为输入）
		for (int32 y = 0; y < SizeY; ++y)
		{
			const int32 Row = Base + y * SizeX;
			SlidingMin1D([Row, &In](int32 Idx) { return In[Row + Idx]; }, Temp, SizeX, W, Half);
			FMemory::Memcpy(Out.GetData() + Row, Temp.GetData(), SizeX);
		}
		// 纵向：逐列对 Out 再滑一次（列内先收进缓冲再写回，读写不互相踩）
		TArray<uint8> Col;
		Col.SetNumUninitialized(SizeY);
		for (int32 x = 0; x < SizeX; ++x)
		{
			SlidingMin1D([x, Base, SizeX, &Out](int32 Idx) { return Out[Base + Idx * SizeX + x]; }, Col, SizeY, W, Half);
			for (int32 y = 0; y < SizeY; ++y)
			{
				Out[Base + y * SizeX + x] = Col[y];
			}
		}
	}
}

void FVoxelSection::BuildNavData(const AVoxelTerrainActor* Terrain)
{
	NavDataByTier.Reset();	// 先清表：烘焙中途提前 return 时也不留旧档表
	if (!Terrain)
	{
		return;
	}

	/* 连接分两类，正好对应移动的两套驱动：
	   - 平面连接：同一层、水平相邻。UVoxelPathFollowingComponent 自己插值走过去。
	   - 非平面连接：高度不同或不相邻。这类必须挂一个 UVoxelNavLinkProxy，由它把人挪到对面去。
	   自动连接（bAutoSpawNavLink）负责把「高差在 NavLinkMaxHeightDiff 内的相邻格」用
	   AutoLinkProxy 连起来；手动连接走 AddLinkProxy，两端距离不限。 */
	const int32 MaxAllowHeight = Terrain->GetMaxAllowHeight();
	const bool bAutoLinks = Terrain->ShouldAutoSpawnNavLinks();
	const int32 LinkReachZ = bAutoLinks ? Terrain->GetNavLinkMaxHeightDiff() : 0;
	const TSubclassOf<UVoxelNavLinkProxy> AutoProxyClass = bAutoLinks ? Terrain->GetAutoNavLinkProxyClass() : nullptr;

	/* 体型档表（升序、去重、恒含 1，见 AVoxelTerrainActor::GetNavFootprintWidths）。
	   宽档 = 把单列净空图按 W×W 滑窗 min 侵蚀；档 0 与旧数据完全一致。
	   footprint 以落脚格为 anchor：向 -X/-Y 覆盖 W/2 格、向 +X/+Y 覆盖 (W-1)-W/2 格。 */
	const TArray<int32>& TierWidths = Terrain->GetNavFootprintWidths();
	const int32 MaxW = TierWidths.Num() > 0 ? TierWidths.Last() : 1;

	const FIntVector SectionOrigin = SectionCoord * LENGTH;
	// 落脚判定用的窄带：本 Section 在 X/Y 外扩 max(1, MaxW-1) —— 水平邻格与宽档 footprint 都会越出本层边界，
	// 外扩量保证「本层 anchor 的整块 footprint」都落在窄带内；
	// Z 下扩 LinkReachZ+1 覆盖邻格的支撑格（判断“可站立”要往下看一格），上扩 LinkReachZ 覆盖台阶。
	// 净空只从落脚格向上数 MaxAllowHeight 格，所以查询区域只要铺到最高落脚格之上这么多格即可。
	const int32 BottomZ = SectionOrigin.Z - (LinkReachZ + 1);
	const int32 HighestStandableZ = SectionOrigin.Z + LENGTH + LinkReachZ;					// 邻格可能站到的最高一层
	const int32 TopZ = FMath::Max(HighestStandableZ, SectionOrigin.Z + LENGTH - 1 + MaxAllowHeight - 1);

	const int32 XYHalfPad = FMath::Max(1, MaxW - 1);
	const int32 XYSize = LENGTH + 2 * XYHalfPad;
	const int32 StandLayers = HighestStandableZ - BottomZ + 1;
	const FIntVector PaddedMin(SectionOrigin.X - XYHalfPad, SectionOrigin.Y - XYHalfPad, BottomZ);
	const FNavRegion StandRegion{ PaddedMin, FIntVector(XYSize, XYSize, StandLayers) };
	// 阻碍查询要盖到净空看得到的最高处：头顶的桥、屋檐会压低下方落脚格的净空
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

	// 3) 基础净空图：窄带内每个可站格的「单列净空」（封顶 MaxAllowHeight），不可站记 0。
	//    宽档直接在这张图上做 min 侵蚀、不再回查体素，所以整带都要铺（不只本 Section 那 16 层）。
	const int32 PlaneSize = XYSize * XYSize;
	TArray<uint8> FootClear;
	FootClear.SetNumZeroed(PlaneSize * StandLayers);
	for (int32 z = 0; z < StandLayers; ++z)
	{
		for (int32 y = 0; y < XYSize; ++y)
		{
			for (int32 x = 0; x < XYSize; ++x)
			{
				const FIntVector Coord = StandRegion.Min + FIntVector(x, y, z);
				if (IsStandable(Coord))
				{
					FootClear[(z * XYSize + y) * XYSize + x] =
						static_cast<uint8>(ComputeNavClearance(Grid, Reader, Obstacles, Coord, MaxAllowHeight));
				}
			}
		}
	}

	NavDataByTier.SetNum(TierWidths.Num());

	auto GlobalToLocalSection = [](const FIntVector& Coord)
		{
			return FIntVector(FloorDivide(Coord.X, LENGTH), FloorDivide(Coord.Y, LENGTH), FloorDivide(Coord.Z, LENGTH));
		};

	// 写一条连接（同一条重复写会被忽略：手动连接可能与自动连接落在同一对格子上）
	auto AddLink = [&GlobalToLocalSection](FVoxelNavCell& Cell, const FIntVector& Target, TSubclassOf<UVoxelNavLinkProxy> ProxyClass)
		{
			const FIntVector TargetSection = GlobalToLocalSection(Target);
			const FIntVector TargetLocal = Target - TargetSection * LENGTH;
			for (const FVoxelNavLink& Existing : Cell.Links)
			{
				if (Existing.OwnnerSection == TargetSection && Existing.LinkCoord == TargetLocal)
				{
					return;
				}
			}
			FVoxelNavLink& Link = Cell.Links.AddDefaulted_GetRef();
			Link.OwnnerSection = TargetSection;
			Link.LinkCoord = TargetLocal;
			Link.ProxyClass = ProxyClass;
		};

	// 4) 手动连接先按「端点是否落在本 Section」分两堆：作为起点（正着连）与作为终点（非单向时反向补一条）
	auto IsInThisSection = [SectionOrigin](const FIntVector& Coord)
		{
			return Coord.X >= SectionOrigin.X && Coord.X < SectionOrigin.X + LENGTH
				&& Coord.Y >= SectionOrigin.Y && Coord.Y < SectionOrigin.Y + LENGTH
				&& Coord.Z >= SectionOrigin.Z && Coord.Z < SectionOrigin.Z + LENGTH;
		};
	auto IsOneWay = [](TSubclassOf<UVoxelNavLinkProxy> ProxyClass)
		{
			const UVoxelNavLinkProxy* CDO = ProxyClass ? ProxyClass->GetDefaultObject<UVoxelNavLinkProxy>() : nullptr;
			return CDO && CDO->bOneWay;
		};

	const TArray<FVoxelNavLinkProxyData>& AllManualLinks = Terrain->GetLinkProxyData();
	TMap<FIntVector, TArray<const FVoxelNavLinkProxyData*>> ByStart;
	TMap<FIntVector, TArray<const FVoxelNavLinkProxyData*>> ByDestination;
	for (const FVoxelNavLinkProxyData& Data : AllManualLinks)
	{
		if (!Data.ProxyClass)
		{
			// 没有代理类的非平面连接没法走：不写，免得寻路以为走得通
			UE_LOG(LogTemp, Warning, TEXT("[Voxel] 手动连接 (%d,%d,%d)->(%d,%d,%d) 没填 ProxyClass，已忽略"),
				Data.StartCoord.X, Data.StartCoord.Y, Data.StartCoord.Z, Data.Destination.X, Data.Destination.Y, Data.Destination.Z);
			continue;
		}
		if (IsInThisSection(Data.StartCoord))
		{
			ByStart.FindOrAdd(Data.StartCoord).Add(&Data);
		}
		if (IsInThisSection(Data.Destination))
		{
			ByDestination.FindOrAdd(Data.Destination).Add(&Data);
		}
	}

	// 5) 逐档出节点与连接。每档一张「footprint 净空图」TierClear：非 0 即整块 footprint 可站，
	//    值就是覆盖列净空的 min；节点与连接判定只看这张图。4 邻移动扫过的位置并集恰为两端
	//    footprint（格子图只有轴向迈步，没有斜步，拐角天然安全），所以「两端都是本档节点」
	//    就是平面连接对本档成立的充要判据；台阶连接取保守判据（两端整块可站，不查抬脚瞬间）。
	static const FIntVector Directions[4] = {
		FIntVector(1, 0, 0), FIntVector(-1, 0, 0), FIntVector(0, 1, 0), FIntVector(0, -1, 0) };

	auto EmitTier = [&](int32 Tier, const TArray<uint8>& TierClear)
	{
		auto ClearAt = [&](const FIntVector& C) -> uint8
		{
			const int32 RelX = C.X - PaddedMin.X;
			const int32 RelY = C.Y - PaddedMin.Y;
			const int32 RelZ = C.Z - BottomZ;
			if (RelX < 0 || RelY < 0 || RelZ < 0 || RelZ >= StandLayers || RelX >= XYSize || RelY >= XYSize)
			{
				return 0;		// 窄带外 = 不可站（外扩量保证本层 anchor 与各档 footprint 不会真越出去）
			}
			return TierClear[(RelZ * XYSize + RelY) * XYSize + RelX];
		};

		TMap<FIntVector, FVoxelNavCell>& TierMap = NavDataByTier[Tier];
		for (int32 z = 0; z < LENGTH; ++z)
		{
			for (int32 y = 0; y < LENGTH; ++y)
			{
				for (int32 x = 0; x < LENGTH; ++x)
				{
					const FIntVector LocalCoord(x, y, z);
					const FIntVector Coord = SectionOrigin + LocalCoord;
					const uint8 SelfClear = ClearAt(Coord);
					if (SelfClear == 0)
					{
						continue;		// 本档站不下
					}

					// 可站但没有邻居的格也要留一条记录：Key 存在即代表这一格能站
					FVoxelNavCell& Cell = TierMap.Add(LocalCoord);
					Cell.AllowHeight = SelfClear;

					for (const FIntVector& Direction : Directions)
					{
						// 平面连接：同层的水平 4 邻，不挂代理
						const FIntVector Side = Coord + Direction;
						if (ClearAt(Side) > 0)
						{
							AddLink(Cell, Side, nullptr);
						}

						// 自动连接：高差在 NavLinkMaxHeightDiff 内的上下台阶，挂自动代理
						if (bAutoLinks)
						{
							for (int32 DeltaZ = 1; DeltaZ <= LinkReachZ; ++DeltaZ)
							{
								const FIntVector Up = Side + FIntVector(0, 0, DeltaZ);
								if (ClearAt(Up) > 0)
								{
									AddLink(Cell, Up, AutoProxyClass);
								}
								const FIntVector Down = Side - FIntVector(0, 0, DeltaZ);
								if (ClearAt(Down) > 0)
								{
									AddLink(Cell, Down, AutoProxyClass);
								}
							}
						}
					}

					// 手动连接：另一端可能在本 Section 之外任意远，所以不校验对方的可站性 ——
					// 对方没被烘成落脚格时，A* 查询会因为查不到它的记录而自然忽略这条连接
					if (const TArray<const FVoxelNavLinkProxyData*>* FromHere = ByStart.Find(Coord))
					{
						for (const FVoxelNavLinkProxyData* Data : *FromHere)
						{
							AddLink(Cell, Data->Destination, Data->ProxyClass);
						}
					}
					if (const TArray<const FVoxelNavLinkProxyData*>* ToHere = ByDestination.Find(Coord))
					{
						for (const FVoxelNavLinkProxyData* Data : *ToHere)
						{
							if (!IsOneWay(Data->ProxyClass))
							{
								AddLink(Cell, Data->StartCoord, Data->ProxyClass);
							}
						}
					}
				}
			}
		}
	};

	TArray<uint8> TierClear;
	TArray<uint8> ErodeTemp;
	for (int32 Tier = 0; Tier < TierWidths.Num(); ++Tier)
	{
		const int32 W = TierWidths[Tier];
		if (W <= 1)
		{
			EmitTier(Tier, FootClear);		// 档 0：footprint 就是单列本身，净空图直接用
			continue;
		}
		TierClear = FootClear;
		for (int32 z = 0; z < StandLayers; ++z)
		{
			ErodePlane(FootClear, TierClear, ErodeTemp, z * PlaneSize, XYSize, XYSize, W, W / 2);
		}
		EmitTier(Tier, TierClear);
	}
}

const TMap<FIntVector, FVoxelNavCell>& FVoxelSection::GetNavTierData(int32 Tier) const
{
	static const TMap<FIntVector, FVoxelNavCell> Empty;
	return NavDataByTier.IsValidIndex(Tier) ? NavDataByTier[Tier] : Empty;
}

void FVoxelChunk::BuildNavData(AVoxelTerrainActor* Terrain, int32 CoordZ)
{
	const int32 Index = GetSectionIndex(CoordZ);
	if (!Sections.IsValidIndex(Index))
	{
		return;		// 该高度不在本 Chunk 的范围内（脏区里合法地会混进越界的相邻 Section），与 BuildMesh 保持一致
	}
	Sections[Index].BuildNavData(Terrain);
}

void FVoxelChunk::BuildAllNavData(AVoxelTerrainActor* Terrain)
{
	for (FVoxelSection& Section : Sections)
	{
		Section.BuildNavData(Terrain);
	}
}

void FVoxelChunk::ClearAllNavData()
{
	for (FVoxelSection& Section : Sections)
	{
		Section.ClearNavData();
	}
}