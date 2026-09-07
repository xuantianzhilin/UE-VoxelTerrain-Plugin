// VoxelTerrain 表面寻路：不依赖 NavMesh，直接在体素数据上跑 A*。
// 模式 A（FindVoxelPath）：严格以体素为单位的格路径。
// 模式 B（FindSurfacePath）：世界坐标端点 + 贴面拉紧平滑，允许任意方向斜着走。
// 通行规则、静态网格占据烘焙、区域代价权重见各小节注释。

#include "VoxelPath.h"
#include "VoxelTerrainActor.h"
#include "VoxelChunk.h"
#include "Engine/World.h"
#include "Engine/EngineTypes.h"
#include "Engine/OverlapResult.h"
#include "Components/PrimitiveComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "GameFramework/Pawn.h"
#include "ProceduralMeshComponent.h"
#include "CollisionShape.h"
#include "CollisionQueryParams.h"
#include "DrawDebugHelpers.h"
#include "Algo/Reverse.h"

/* ===================== 常量与工具 ===================== */

namespace
{
	/* XY 搜索区（含外扩）的格数上限，超过判 AreaTooLarge，防一次查询拖死游戏线程 */
	constexpr int64 MaxSearchXYCells = 256LL * 256;
	/* 体素坐标合法范围（1M 格 × 100cm = 10 万公里，远超任何关卡；挡掉蓝图里手滑的极端坐标） */
	constexpr int64 MaxSearchCoord = 1LL << 20;
	/* 水平斜步与正交步的基础代价比 */
	constexpr float DiagCost = 1.41421356f;
	/* 每 1 格高差的附加代价（偏好平路） */
	constexpr float ClimbPenalty = 0.1f;
	/* 超过该节点数就跳过拉紧平滑，直接输出格心折线 */
	constexpr int32 MaxSmoothNodes = 1024;
	/* 单次查询静态网格占据烘焙的格数上限 */
	constexpr int32 MaxBakedMeshCells = 128 * 1024;
	/* DebugDraw 各类标记的点上限 */
	constexpr int32 MaxDebugMeshPoints = 2000;
	constexpr int32 MaxDebugWeightPoints = 5000;
}

/* A* 开放表节点：FOpenNodeLess 构成按 F 弹出的最小堆（UE 5.8 已移除 TPriorityQueue） */
struct FOpenNode
{
	float F = 0.f;
	FIntVector Coord;
};
struct FOpenNodeLess
{
	bool operator()(const FOpenNode& A, const FOpenNode& B) const { return A.F < B.F; }
};

/* ===================== 求解器 ===================== */

struct FVoxelPathSolver
{
	FVoxelPathSolver(const AVoxelTerrainActor* InTerrain, FVoxelPathParams InParams);

	FVoxelPathResult SolveGrid(const FIntVector& Start, const FIntVector& End);
	FVoxelPathResult SolveSurface(const FVector& StartWorld, const FVector& EndWorld);

private:
	/* ---------- 查询上下文 ---------- */
	const AVoxelTerrainActor* Terrain;
	const UWorld* World;
	FVoxelPathParams Params;
	FVector VoxelSize;
	FVector ActorLocation;
	int32 HeightMin = 0;			// AVoxelTerrainActor::MinHeight
	int32 HeightMax = 0;			// AVoxelTerrainActor::MaxHeight
	bool bSurfaceMode = false;
	int32 LastExpansions = 0;

	/* 搜索区（体素坐标，闭区间）；Z 取 [HeightMin-1, HeightMax]，最高层允许站在地形顶面 */
	FIntVector RegionMin{ 0, 0, 0 };
	FIntVector RegionMax{ 0, 0, 0 };
	bool bRegionValid = false;

	/* ---------- 每查询缓存 ---------- */
	TSet<FIntVector> MeshCells;			// 静态网格占据烘焙结果
	TMap<FIntVector, bool> SolidCache;	// 体素是否非空
	TMap<FIntVector, bool> StandableCache;
	float MinWeightInRange = 1.f;		// 搜索区内最小正权重（含 1.0），供启发式保持可采纳性

	/* ---------- 基础判定 ---------- */
	FIntVector FloorCoord(const FVector& WorldPos) const;
	bool InRegion(const FIntVector& C) const;
	bool IsSolid(const FIntVector& C);
	bool IsStandable(const FIntVector& C);
	float WeightOf(const FIntVector& C) const;
	FString DescribeStandableFailure(const FIntVector& C);

	/* ---------- 查询准备 ---------- */
	void PrepareRegion(const FIntVector& A, const FIntVector& B, int32 Pad);
	void BakeMeshCells();
	void BakeWeightRange();
	bool SnapToSurface(const FVector& P, FIntVector& OutCell);

	/* ---------- A* ---------- */
	float Heuristic(const FIntVector& C, const FIntVector& Goal) const;
	EVoxelPathResult RunAStar(const FIntVector& Start, const FIntVector& Goal, TArray<FIntVector>& OutPath, float& OutCost);

	/* ---------- Surface 平滑 ---------- */
	bool SurfaceCellAt(const FVector& SamplePos, FIntVector& OutCell);
	bool LineOfSight(const FIntVector& A, const FIntVector& B);
	FVector EndWaypoint(const FVector& Requested, const FIntVector& Cell) const;
	void SmoothPath(const TArray<FIntVector>& Raw, const FVector& StartWorld, const FVector& EndWorld, TArray<FVector>& OutPath, TArray<FVector>& OutControl);

	void DebugDraw(const FVoxelPathResult& Result, const FVector& StartWorld, const FVector& EndWorld) const;
};

FVoxelPathSolver::FVoxelPathSolver(const AVoxelTerrainActor* InTerrain, FVoxelPathParams InParams)
	: Terrain(InTerrain)
	, World(InTerrain->GetWorld())
	, Params(InParams)
	, VoxelSize(InTerrain->GetVoxelSize())
	, ActorLocation(InTerrain->GetActorLocation())
	, HeightMin(InTerrain->MinHeight)
	, HeightMax(InTerrain->MaxHeight)
{
	const int32 HeightSpan = FMath::Max(1, HeightMax - HeightMin);
	Params.JumpHeight = FMath::Clamp(Params.JumpHeight, 0, HeightSpan);
	Params.AgentHeight = FMath::Clamp(Params.AgentHeight, 1, HeightSpan);
	Params.MaxExpansions = FMath::Clamp(Params.MaxExpansions, 1, 1000000);
	Params.SearchPadding = FMath::Clamp(Params.SearchPadding, 0, 256);
	Params.SnapRadiusXY = FMath::Clamp(Params.SnapRadiusXY, 0, 64);
	Params.SnapUpTolerance = FMath::Clamp(Params.SnapUpTolerance, 0, 256);
	Params.SnapDownTolerance = FMath::Clamp(Params.SnapDownTolerance, 0, 256);
}

FIntVector FVoxelPathSolver::FloorCoord(const FVector& WorldPos) const
{
	// 与 AVoxelTerrainActor::WorldLocationToCoord 同一套换算：假设 Actor 不旋转
	return FIntVector(
		FMath::FloorToInt((WorldPos.X - ActorLocation.X) / VoxelSize.X),
		FMath::FloorToInt((WorldPos.Y - ActorLocation.Y) / VoxelSize.Y),
		FMath::FloorToInt((WorldPos.Z - ActorLocation.Z) / VoxelSize.Z));
}

bool FVoxelPathSolver::InRegion(const FIntVector& C) const
{
	return C.X >= RegionMin.X && C.X <= RegionMax.X
		&& C.Y >= RegionMin.Y && C.Y <= RegionMax.Y
		&& C.Z >= RegionMin.Z && C.Z <= RegionMax.Z;
}

bool FVoxelPathSolver::IsSolid(const FIntVector& C)
{
	if (MeshCells.Contains(C))
	{
		return true;
	}
	if (const bool* Cached = SolidCache.Find(C))
	{
		return *Cached;
	}
	const bool bSolid = !Terrain->GetVoxel(C).IsNone();
	SolidCache.Add(C, bSolid);
	return bSolid;
}

bool FVoxelPathSolver::IsStandable(const FIntVector& C)
{
	if (!InRegion(C))
	{
		return false;
	}
	if (const bool* Cached = StandableCache.Find(C))
	{
		return *Cached;
	}
	bool bOk = true;
	for (int32 i = 0; i < Params.AgentHeight; ++i)
	{
		if (IsSolid(FIntVector(C.X, C.Y, C.Z + i)))
		{
			bOk = false;
			break;
		}
	}
	if (bOk)
	{
		// 脚下必须有支撑（体素或被网格占据的格都算）——防止凭空走在未生成区域上
		bOk = IsSolid(FIntVector(C.X, C.Y, C.Z - 1));
	}
	StandableCache.Add(C, bOk);
	return bOk;
}

float FVoxelPathSolver::WeightOf(const FIntVector& C) const
{
	if (!Params.bUseCostWeights)
	{
		return 1.f;
	}
	const float* Found = Terrain->PathCostWeights.Find(C);
	// 写入侧已挡负数，这里再兜一层（手改存档/旧数据）：负权会破坏 A* 的单调性，按 0 处理
	return Found ? FMath::Max(*Found, 0.f) : 1.f;
}

FString FVoxelPathSolver::DescribeStandableFailure(const FIntVector& C)
{
	if (!InRegion(C))
	{
		return FString::Printf(TEXT("不在搜索/高度区（Z 可站范围 [%d,%d]）"), RegionMin.Z, RegionMax.Z);
	}
	for (int32 i = 0; i < Params.AgentHeight; ++i)
	{
		const FIntVector A(C.X, C.Y, C.Z + i);
		if (MeshCells.Contains(A))
		{
			return FString::Printf(TEXT("净空格 (%d,%d,%d) 被静态网格占据"), A.X, A.Y, A.Z);
		}
		if (!Terrain->GetVoxel(A).IsNone())
		{
			return FString::Printf(TEXT("净空格 (%d,%d,%d) 被体素占据"), A.X, A.Y, A.Z);
		}
	}
	if (!IsSolid(FIntVector(C.X, C.Y, C.Z - 1)))
	{
		return TEXT("脚下无支撑（角色悬空，或地形网格/碰撞还没建好）");
	}
	if (WeightOf(C) <= 0.f)
	{
		return TEXT("权重为 0（软墙）");
	}
	return TEXT("未知");
}

void FVoxelPathSolver::PrepareRegion(const FIntVector& A, const FIntVector& B, int32 Pad)
{
	const int64 AX = A.X, AY = A.Y, BX = B.X, BY = B.Y;
	if (FMath::Abs(AX) > MaxSearchCoord || FMath::Abs(BX) > MaxSearchCoord ||
		FMath::Abs(AY) > MaxSearchCoord || FMath::Abs(BY) > MaxSearchCoord)
	{
		bRegionValid = false;
		return;
	}
	const int64 LowX = FMath::Min(AX, BX) - Pad;
	const int64 HighX = FMath::Max(AX, BX) + Pad;
	const int64 LowY = FMath::Min(AY, BY) - Pad;
	const int64 HighY = FMath::Max(AY, BY) + Pad;

	bRegionValid = (HighX - LowX + 1) * (HighY - LowY + 1) <= MaxSearchXYCells;
	RegionMin = FIntVector(static_cast<int32>(LowX), static_cast<int32>(LowY), HeightMin - 1);
	RegionMax = FIntVector(static_cast<int32>(HighX), static_cast<int32>(HighY), HeightMax);
}

void FVoxelPathSolver::BakeMeshCells()
{
	if (!Params.bCheckStaticMeshes || !World)
	{
		return;
	}
	const FVector BoxMin(
		ActorLocation.X + RegionMin.X * VoxelSize.X,
		ActorLocation.Y + RegionMin.Y * VoxelSize.Y,
		ActorLocation.Z + RegionMin.Z * VoxelSize.Z);
	const FVector BoxMax(
		ActorLocation.X + (RegionMax.X + 1) * VoxelSize.X,
		ActorLocation.Y + (RegionMax.Y + 1) * VoxelSize.Y,
		ActorLocation.Z + (RegionMax.Z + 1) * VoxelSize.Z);

	TArray<FOverlapResult> Hits;
	// AllObjects：骨骼网格多为 QueryOnly/非 WorldStatic 对象通道，只按 WorldStatic 一个通道查会漏掉
	World->OverlapMultiByObjectType(Hits, (BoxMin + BoxMax) * 0.5, FQuat::Identity,
		FCollisionObjectQueryParams(FCollisionObjectQueryParams::InitType::AllObjects),
		FCollisionShape::MakeBox((BoxMax - BoxMin) * 0.5));

	UClass* BlockerClass = Params.BlockerComponentClass.Get();
	if (!BlockerClass)
	{
		BlockerClass = UStaticMeshComponent::StaticClass();
	}
	bool bCapWarned = false;
	for (const FOverlapResult& Hit : Hits)
	{
		const UPrimitiveComponent* Comp = Hit.Component.Get();
		const AActor* Owner = Comp ? Comp->GetOwner() : nullptr;
		if (!Comp || Comp->IsA<UProceduralMeshComponent>() || Owner == Terrain)
		{
			// 体素地形自己的派生网格永远是 ProceduralMesh，被类过滤挡掉；这里再兜一层防用户把过滤类放大到基类
			continue;
		}
		if (Owner && Owner->IsA<APawn>())
		{
			continue;		// Pawn/Character 的身体或挂件不算地面障碍，避免“走动的角色把格子堵住”
		}
		const bool bMatchedBlocker = Comp->IsA(BlockerClass)
			|| (Params.bCheckSkeletalMeshes && Comp->IsA<USkeletalMeshComponent>());
		if (!bMatchedBlocker)
		{
			continue;
		}

		FBox Body;
		bool bBodyValid = false;
		FTransform InstanceXform;
		const int32 ItemIndex = Hit.GetItemIndex();
		const UInstancedStaticMeshComponent* Ism = Cast<UInstancedStaticMeshComponent>(Comp);
		if (Ism && Ism->GetStaticMesh() && ItemIndex != INDEX_NONE && Ism->GetInstanceTransform(ItemIndex, InstanceXform, true))
		{
			// ISM/HISM 一次命中一个实例：只光栅化该实例的包围盒，避免整个组件的大 AABB 糊住全区
			Body = Ism->GetStaticMesh()->GetBounds().TransformBy(InstanceXform).GetBox();
			bBodyValid = true;
		}
		if (!bBodyValid)
		{
			// 注意：世界 AABB 近似，旋转/凹体网格会偏保守（多占一些格）
			Body = Comp->Bounds.GetBox();
		}

		FIntVector Lo = FloorCoord(Body.Min);
		FIntVector Hi = FloorCoord(Body.Max);
		Lo.X = FMath::Clamp(Lo.X, RegionMin.X, RegionMax.X);
		Hi.X = FMath::Clamp(Hi.X, RegionMin.X, RegionMax.X);
		Lo.Y = FMath::Clamp(Lo.Y, RegionMin.Y, RegionMax.Y);
		Hi.Y = FMath::Clamp(Hi.Y, RegionMin.Y, RegionMax.Y);
		Lo.Z = FMath::Clamp(Lo.Z, RegionMin.Z, RegionMax.Z);
		Hi.Z = FMath::Clamp(Hi.Z, RegionMin.Z, RegionMax.Z);
		if (Lo.X > Hi.X || Lo.Y > Hi.Y || Lo.Z > Hi.Z)
		{
			continue;
		}

		const int64 Add = static_cast<int64>(Hi.X - Lo.X + 1) * (Hi.Y - Lo.Y + 1) * (Hi.Z - Lo.Z + 1);
		if (Add + MeshCells.Num() > MaxBakedMeshCells)
		{
			if (!bCapWarned)
			{
				bCapWarned = true;
				UE_LOG(LogTemp, Warning, TEXT("%s：寻路烘焙静态网格占据超过上限 %d 格，后续超预算的网格已跳过（占据判定可能偏宽松）"),
					*Terrain->GetName(), MaxBakedMeshCells);
			}
			continue;
		}
		for (int32 z = Lo.Z; z <= Hi.Z; ++z)
		{
			for (int32 y = Lo.Y; y <= Hi.Y; ++y)
			{
				for (int32 x = Lo.X; x <= Hi.X; ++x)
				{
					MeshCells.Add(FIntVector(x, y, z));
				}
			}
		}
	}
}

void FVoxelPathSolver::BakeWeightRange()
{
	MinWeightInRange = 1.f;
	if (!Params.bUseCostWeights)
	{
		return;
	}
	for (const TPair<FIntVector, float>& Pair : Terrain->PathCostWeights)
	{
		const float W = Pair.Value;
		if (W > 0.f && W < MinWeightInRange && InRegion(Pair.Key))
		{
			MinWeightInRange = W;
		}
	}
}

bool FVoxelPathSolver::SnapToSurface(const FVector& P, FIntVector& OutCell)
{
	const FIntVector Base = FloorCoord(P);
	const int32 R = Params.SnapRadiusXY;
	float BestScore = TNumericLimits<float>::Max();
	FIntVector Best = Base;
	bool bFound = false;

	const int32 HiZ = FMath::Min(Base.Z + Params.SnapUpTolerance, RegionMax.Z);
	const int32 LoZ = FMath::Max(Base.Z - Params.SnapDownTolerance, RegionMin.Z);
	for (int32 dy = -R; dy <= R; ++dy)
	{
		for (int32 dx = -R; dx <= R; ++dx)
		{
			const float DistXY = FMath::Sqrt(static_cast<float>(dx * dx + dy * dy));
			if (DistXY > R + 0.001f)
			{
				continue;
			}
			for (int32 z = HiZ; z >= LoZ; --z)
			{
				const FIntVector C(Base.X + dx, Base.Y + dy, z);
				if (!IsStandable(C) || WeightOf(C) <= 0.f)
				{
					continue;
				}
				// XY 距离优先、其次尽量贴近请求高度
				const float Score = DistXY * 4.f + 0.1f * FMath::Abs(z - Base.Z);
				if (Score < BestScore)
				{
					BestScore = Score;
					Best = C;
					bFound = true;
				}
				break;		// 一列只取最高的可站面
			}
		}
	}
	if (bFound)
	{
		OutCell = Best;
	}
	return bFound;
}

float FVoxelPathSolver::Heuristic(const FIntVector& C, const FIntVector& Goal) const
{
	const float dx = FMath::Abs(static_cast<float>(C.X - Goal.X));
	const float dy = FMath::Abs(static_cast<float>(C.Y - Goal.Y));
	const float Smaller = FMath::Min(dx, dy);
	const float Larger = FMath::Max(dx, dy);
	// 每步 XY 最多推进 1（正交）/ √2（斜向），纵向免费（一步内 dz ≤ JumpHeight），乘最小权重后仍可采纳
	return (Larger + (DiagCost - 1.f) * Smaller) * MinWeightInRange;
}

EVoxelPathResult FVoxelPathSolver::RunAStar(const FIntVector& Start, const FIntVector& Goal, TArray<FIntVector>& OutPath, float& OutCost)
{
	OutPath.Reset();
	OutCost = 0.f;
	LastExpansions = 0;

	if (Start == Goal)
	{
		OutPath.Add(Start);
		return EVoxelPathResult::Found;
	}

	const bool bDiagAllowed = bSurfaceMode || Params.bAllowDiagonal;
	const int32 J = Params.JumpHeight;

	TMap<FIntVector, float> GScore;
	TMap<FIntVector, FIntVector> Parent;
	TSet<FIntVector> Closed;
	TArray<FOpenNode> Open;

	GScore.Add(Start, 0.f);
	Open.HeapPush(FOpenNode{ Heuristic(Start, Goal), Start }, FOpenNodeLess());

	FOpenNodeLess Less;
	while (!Open.IsEmpty())
	{
		if (LastExpansions >= Params.MaxExpansions)
		{
			return EVoxelPathResult::BudgetExceeded;
		}
		FOpenNode Current;
		Open.HeapPop(Current, Less, EAllowShrinking::No);
		if (Closed.Contains(Current.Coord))
		{
			continue;		// 允许重复入堆，出堆时丢弃旧记录
		}
		Closed.Add(Current.Coord);
		++LastExpansions;

		if (Current.Coord == Goal)
		{
			FIntVector C = Goal;
			while (true)
			{
				OutPath.Add(C);
				const FIntVector* From = Parent.Find(C);
				if (!From)
				{
					break;
				}
				C = *From;
			}
			Algo::Reverse(OutPath);
			OutCost = GScore.FindRef(Goal);
			return EVoxelPathResult::Found;
		}

		const float BaseG = GScore.FindRef(Current.Coord);
		for (int32 dz = -J; dz <= J; ++dz)
		{
			for (int32 dx = -1; dx <= 1; ++dx)
			{
				for (int32 dy = -1; dy <= 1; ++dy)
				{
					if (dx == 0 && dy == 0)
					{
						continue;		// 不做原地垂直跳
					}
					const bool bDiag = dx != 0 && dy != 0;
					if (bDiag && !bDiagAllowed)
					{
						continue;
					}
					const FIntVector& A = Current.Coord;
					const FIntVector Next(A.X + dx, A.Y + dy, A.Z + dz);
					if (Closed.Contains(Next) || !IsStandable(Next))
					{
						continue;
					}
					const float W = WeightOf(Next);
					if (W <= 0.f)
					{
						continue;		// 权重 0 = 软墙，不可进入
					}
					if (bDiag && (IsSolid(FIntVector(Next.X, A.Y, Next.Z)) || IsSolid(FIntVector(A.X, Next.Y, Next.Z))))
					{
						continue;		// 斜步防穿墙角：两条直角中间格都得是空的
					}
					const float NewG = BaseG + (bDiag ? DiagCost : 1.f) * W + ClimbPenalty * FMath::Abs(dz);
					if (const float* Existing = GScore.Find(Next))
					{
						if (*Existing <= NewG)
						{
							continue;
						}
					}
					GScore.Add(Next, NewG);
					Parent.Add(Next, A);
					Open.HeapPush(FOpenNode{ NewG + Heuristic(Next, Goal), Next }, Less);
				}
			}
		}
	}
	return EVoxelPathResult::NoPath;
}

bool FVoxelPathSolver::SurfaceCellAt(const FVector& SamplePos, FIntVector& OutCell)
{
	// 该 XY 列上，找采样高度 ±JumpHeight 窗口内最高的可站格——"脚能落在窗口里的面上"才算可见
	const FIntVector Col = FloorCoord(SamplePos);
	const int32 HiZ = FMath::Min(Col.Z + Params.JumpHeight, RegionMax.Z);
	const int32 LoZ = FMath::Max(Col.Z - Params.JumpHeight, RegionMin.Z);
	for (int32 z = HiZ; z >= LoZ; --z)
	{
		const FIntVector C(Col.X, Col.Y, z);
		if (IsStandable(C) && WeightOf(C) > 0.f)
		{
			OutCell = C;
			return true;
		}
	}
	return false;
}

bool FVoxelPathSolver::LineOfSight(const FIntVector& A, const FIntVector& B)
{
	if (A == B)
	{
		return true;
	}
	const FVector PA = Terrain->CoordToWorldLocation(A);
	const FVector PB = Terrain->CoordToWorldLocation(B);
	const double Step = FMath::Min(VoxelSize.X, VoxelSize.Y) * 0.5;
	const int32 Samples = FMath::Max(1, FMath::CeilToInt(FVector::Dist2D(PA, PB) / Step));
	for (int32 i = 1; i < Samples; ++i)
	{
		const FVector S = FMath::Lerp(PA, PB, static_cast<double>(i) / Samples);
		FIntVector Cell;
		if (!SurfaceCellAt(S, Cell))
		{
			return false;
		}
	}
	return true;
}

FVector FVoxelPathSolver::EndWaypoint(const FVector& Requested, const FIntVector& Cell) const
{
	// 请求点与吸附格同列时用请求 XY（更贴合"从哪儿出发"），否则退回格心
	const FVector Center = Terrain->CoordToWorldLocation(Cell);
	const FIntVector Col = FloorCoord(Requested);
	if (Col.X == Cell.X && Col.Y == Cell.Y)
	{
		return FVector(Requested.X, Requested.Y, Center.Z);
	}
	return Center;
}

void FVoxelPathSolver::SmoothPath(const TArray<FIntVector>& Raw, const FVector& StartWorld, const FVector& EndWorld, TArray<FVector>& OutPath, TArray<FVector>& OutControl)
{
	const int32 N = Raw.Num();
	if (N == 0)
	{
		return;
	}
	if (N > MaxSmoothNodes)
	{
		// 超长路径不值得做 O(n²) 拉紧，退化为格心折线（正确但带锯齿）
		for (const FIntVector& C : Raw)
		{
			const FVector P = Terrain->CoordToWorldLocation(C);
			OutPath.Add(P);
			OutControl.Add(P);
		}
		return;
	}

	// 1) 字符串拉紧：每个控制点尽量跳过能直达的远点，得到最短的折角骨架
	TArray<FIntVector> Nodes = Raw;
	TArray<int32> Ctrl;
	Ctrl.Add(0);
	int32 i = 0;
	while (i < N - 1)
	{
		int32 Best = i + 1;
		for (int32 j = N - 1; j > i + 1; --j)
		{
			if (LineOfSight(Nodes[i], Nodes[j]))
			{
				Best = j;
				break;
			}
		}
		Ctrl.Add(Best);
		i = Best;
	}

	// 2) 梯度微调：把中间控制点往两邻点连线中点推（仍必须是合法落脚格），让斜线更圆
	for (int32 Round = 0; Round < 2; ++Round)
	{
		bool bMoved = false;
		for (int32 k = 1; k + 1 < Ctrl.Num(); ++k)
		{
			const FIntVector& A = Nodes[Ctrl[k - 1]];
			const FIntVector& B = Nodes[Ctrl[k]];
			const FIntVector& C = Nodes[Ctrl[k + 1]];
			const FVector PA = Terrain->CoordToWorldLocation(A);
			const FVector PC = Terrain->CoordToWorldLocation(C);
			const FVector MidPos((PA.X + PC.X) * 0.5, (PA.Y + PC.Y) * 0.5, Terrain->CoordToWorldLocation(B).Z);
			FIntVector Candidate;
			if (!SurfaceCellAt(MidPos, Candidate) || Candidate == B)
			{
				continue;
			}
			if (LineOfSight(A, Candidate) && LineOfSight(Candidate, C))
			{
				Nodes[Ctrl[k]] = Candidate;
				bMoved = true;
			}
		}
		if (!bMoved)
		{
			break;
		}
	}

	// 3) 稀疏控制折线：拉紧后的转弯点（首尾吸附到请求 XY），供移动组件直连跟随
	auto AddControl = [&OutControl](const FVector& Point)
	{
		if (!OutControl.IsEmpty() && FVector::DistSquared2D(OutControl.Last(), Point) < 1.0)
		{
			return;
		}
		OutControl.Add(Point);
	};
	for (int32 s = 0; s < Ctrl.Num(); ++s)
	{
		const FIntVector& Cell = Nodes[Ctrl[s]];
		const FVector Point = (s == 0) ? EndWaypoint(StartWorld, Cell)
			: (s == Ctrl.Num() - 1) ? EndWaypoint(EndWorld, Cell)
			: Terrain->CoordToWorldLocation(Cell);
		AddControl(Point);
	}
	if (Ctrl.Num() == 1)
	{
		// 单节点路径（起点即终点吸附格）：补上终点姿态
		AddControl(EndWaypoint(EndWorld, Nodes[Ctrl[0]]));
	}

	// 4) 贴面重采样：控制折线按 1/4 格密采，逐点吸附到落脚面高度；输出 XY 连续、不受格线限制
	const double OutStep = FMath::Min(VoxelSize.X, VoxelSize.Y) * 0.25;
	auto AddPoint = [&OutPath](const FVector& Point)
	{
		if (!OutPath.IsEmpty() && FVector::DistSquared2D(OutPath.Last(), Point) < 1.0)
		{
			return;		// 去掉近重复点（1cm² 以内）
		}
		OutPath.Add(Point);
	};

	AddPoint(EndWaypoint(StartWorld, Nodes[Ctrl[0]]));
	for (int32 s = 0; s + 1 < Ctrl.Num(); ++s)
	{
		const FVector PA = Terrain->CoordToWorldLocation(Nodes[Ctrl[s]]);
		const FVector PB = Terrain->CoordToWorldLocation(Nodes[Ctrl[s + 1]]);
		const int32 Samples = FMath::Max(1, FMath::CeilToInt(FVector::Dist2D(PA, PB) / OutStep));
		for (int32 k = 1; k < Samples; ++k)
		{
			const FVector S = FMath::Lerp(PA, PB, static_cast<double>(k) / Samples);
			FIntVector Cell;
			if (SurfaceCellAt(S, Cell))
			{
				AddPoint(FVector(S.X, S.Y, Terrain->CoordToWorldLocation(Cell).Z));
			}
			// 吸附失败就跳过该密采点：段 LOS 已验证通过，最多少一个中间点，不影响正确性
		}
		AddPoint(PB);
	}
	AddPoint(EndWaypoint(EndWorld, Nodes[Ctrl.Last()]));
}

void FVoxelPathSolver::DebugDraw(const FVoxelPathResult& Result, const FVector& StartWorld, const FVector& EndWorld) const
{
	if (!World || !World->IsGameWorld())
	{
		return;
	}
	constexpr float LifeTime = 5.f;

	for (int32 i = 0; i + 1 < Result.VoxelPath.Num(); ++i)
	{
		DrawDebugLine(World, Terrain->CoordToWorldLocation(Result.VoxelPath[i]), Terrain->CoordToWorldLocation(Result.VoxelPath[i + 1]),
			FColor::Blue, false, LifeTime, 0, 2.f);
	}
	for (int32 i = 0; i + 1 < Result.WorldPath.Num(); ++i)
	{
		DrawDebugLine(World, Result.WorldPath[i], Result.WorldPath[i + 1], FColor::Orange, false, LifeTime, 1, 6.f);
	}

	int32 Drawn = 0;
	const float PointSize = static_cast<float>(FVector(VoxelSize.X, VoxelSize.Y, 0.f).Size()) * 0.03f;
	for (const FIntVector& C : MeshCells)
	{
		if (Drawn++ >= MaxDebugMeshPoints)
		{
			break;
		}
		DrawDebugPoint(World, Terrain->CoordToWorldLocation(C), PointSize, FColor::Red, false, LifeTime);
	}

	if (Params.bUseCostWeights)
	{
		const float WeightSize = static_cast<float>(FVector(VoxelSize.X, VoxelSize.Y, 0.f).Size()) * 0.05f;
		Drawn = 0;
		for (const TPair<FIntVector, float>& Pair : Terrain->PathCostWeights)
		{
			if (Drawn++ >= MaxDebugWeightPoints || !InRegion(Pair.Key))
			{
				continue;
			}
			const FColor Color = Pair.Value <= 0.f ? FColor::Red : (Pair.Value < 1.f ? FColor::Green : FColor::Cyan);
			DrawDebugPoint(World, Terrain->CoordToWorldLocation(Pair.Key), WeightSize, Color, false, LifeTime);
		}
	}

	const float EndSize = static_cast<float>(FVector(VoxelSize.X, VoxelSize.Y, 0.f).Size()) * 0.3f;
	DrawDebugSphere(World, StartWorld, EndSize, 8, FColor::Turquoise, false, LifeTime);
	DrawDebugSphere(World, EndWorld, EndSize, 8, FColor::Yellow, false, LifeTime);
}

FVoxelPathResult FVoxelPathSolver::SolveGrid(const FIntVector& Start, const FIntVector& End)
{
	FVoxelPathResult Result;
	bSurfaceMode = false;
	PrepareRegion(Start, End, Params.SearchPadding);
	if (!bRegionValid)
	{
		Result.Result = EVoxelPathResult::AreaTooLarge;
		return Result;
	}
	BakeMeshCells();
	BakeWeightRange();

	if (!IsStandable(Start) || WeightOf(Start) <= 0.f)
	{
		UE_LOG(LogTemp, Warning, TEXT("%s：Grid 寻路起点 (%d,%d,%d) 不可站：%s"),
			*Terrain->GetName(), Start.X, Start.Y, Start.Z, *DescribeStandableFailure(Start));
		Result.Result = EVoxelPathResult::StartInvalid;
		return Result;
	}
	if (!IsStandable(End) || WeightOf(End) <= 0.f)
	{
		UE_LOG(LogTemp, Warning, TEXT("%s：Grid 寻路终点 (%d,%d,%d) 不可站：%s"),
			*Terrain->GetName(), End.X, End.Y, End.Z, *DescribeStandableFailure(End));
		Result.Result = EVoxelPathResult::GoalInvalid;
		return Result;
	}

	TArray<FIntVector> Path;
	float Cost = 0.f;
	Result.Result = RunAStar(Start, End, Path, Cost);
	if (Result.Result == EVoxelPathResult::Found)
	{
		Result.VoxelPath = Path;
		Result.WorldPath.Reserve(Path.Num());
		for (const FIntVector& C : Path)
		{
			// Z 下移半格：世界路径点的语义是“落脚面”（脚底要去的位置），不是格中心
			Result.WorldPath.Add(Terrain->CoordToWorldLocation(C) - FVector(0.0, 0.0, VoxelSize.Z * 0.5));
		}
		Result.ControlPath = Result.WorldPath;		// Grid 模式无平滑，控制折线即格心折线
		Result.TotalCost = Cost;
	}
	Result.Expansions = LastExpansions;
	if (Params.bDebugDraw)
	{
		DebugDraw(Result, Terrain->CoordToWorldLocation(Start), Terrain->CoordToWorldLocation(End));
	}
	return Result;
}

FVoxelPathResult FVoxelPathSolver::SolveSurface(const FVector& StartWorld, const FVector& EndWorld)
{
	FVoxelPathResult Result;
	bSurfaceMode = true;
	const FIntVector RawStart = FloorCoord(StartWorld);
	const FIntVector RawEnd = FloorCoord(EndWorld);
	// 外扩至少盖住吸附半径，否则吸附结果会被 IsStandable 的区域闸口拒掉
	PrepareRegion(RawStart, RawEnd, FMath::Max(Params.SearchPadding, Params.SnapRadiusXY));
	if (!bRegionValid)
	{
		Result.Result = EVoxelPathResult::AreaTooLarge;
		return Result;
	}
	BakeMeshCells();
	BakeWeightRange();

	FIntVector Start, End;
	if (!SnapToSurface(StartWorld, Start))
	{
		UE_LOG(LogTemp, Warning, TEXT("%s：Surface 寻路起点 %s（格 %d,%d,%d）在半径 %d、上 %d 层/下 %d 层窗口内找不到可站格：%s"),
			*Terrain->GetName(), *StartWorld.ToString(), RawStart.X, RawStart.Y, RawStart.Z,
			Params.SnapRadiusXY, Params.SnapUpTolerance, Params.SnapDownTolerance, *DescribeStandableFailure(RawStart));
		Result.Result = EVoxelPathResult::StartInvalid;
		return Result;
	}
	if (!SnapToSurface(EndWorld, End))
	{
		UE_LOG(LogTemp, Warning, TEXT("%s：Surface 寻路终点 %s（格 %d,%d,%d）在半径 %d、上 %d 层/下 %d 层窗口内找不到可站格：%s"),
			*Terrain->GetName(), *EndWorld.ToString(), RawEnd.X, RawEnd.Y, RawEnd.Z,
			Params.SnapRadiusXY, Params.SnapUpTolerance, Params.SnapDownTolerance, *DescribeStandableFailure(RawEnd));
		Result.Result = EVoxelPathResult::GoalInvalid;
		return Result;
	}

	TArray<FIntVector> Path;
	float Cost = 0.f;
	Result.Result = RunAStar(Start, End, Path, Cost);
	if (Result.Result == EVoxelPathResult::Found)
	{
		Result.VoxelPath = Path;
		SmoothPath(Path, StartWorld, EndWorld, Result.WorldPath, Result.ControlPath);
		// 同 SolveGrid：输出世界路径的 Z 平移到落脚面（内部几何都按格心算，最后统一减半格）
		const FVector FootDrop(0.0, 0.0, VoxelSize.Z * 0.5);
		for (FVector& P : Result.WorldPath)
		{
			P -= FootDrop;
		}
		for (FVector& P : Result.ControlPath)
		{
			P -= FootDrop;
		}
		Result.TotalCost = Cost;
	}
	Result.Expansions = LastExpansions;
	if (Params.bDebugDraw)
	{
		DebugDraw(Result, StartWorld, EndWorld);
	}
	return Result;
}

/* ===================== AVoxelTerrainActor 公开入口 ===================== */

namespace
{
	bool IsValidVoxelSize(const AVoxelTerrainActor* Terrain)
	{
		const FVector& Size = Terrain->GetVoxelSize();
		if (Size.X <= 0.0 || Size.Y <= 0.0 || Size.Z <= 0.0)
		{
			UE_LOG(LogTemp, Warning, TEXT("%s：VoxelSize 存在非正分量（%s），寻路查询已忽略"), *Terrain->GetName(), *Size.ToString());
			return false;
		}
		return true;
	}
}

FVoxelPathResult AVoxelTerrainActor::FindVoxelPath(FIntVector Start, FIntVector End, const FVoxelPathParams& Params) const
{
	FVoxelPathResult Result;
	if (!IsValidVoxelSize(this))
	{
		return Result;
	}
	FVoxelPathSolver Solver(this, Params);
	return Solver.SolveGrid(Start, End);
}

FVoxelPathResult AVoxelTerrainActor::FindSurfacePath(const FVector& StartWorld, const FVector& EndWorld, const FVoxelPathParams& Params) const
{
	FVoxelPathResult Result;
	if (!IsValidVoxelSize(this))
	{
		return Result;
	}
	FVoxelPathSolver Solver(this, Params);
	return Solver.SolveSurface(StartWorld, EndWorld);
}

bool AVoxelTerrainActor::IsVoxelStandable(FIntVector Coord, int32 AgentHeight, bool bCheckStaticMeshes) const
{
	if (!IsValidVoxelSize(this) || GetVoxelPathWeight(Coord) <= 0.f)
	{
		return false;
	}
	AgentHeight = FMath::Clamp(AgentHeight, 1, 64);

	const auto VoxelSolidAt = [this](const FIntVector& C)
	{
		return !GetVoxel(C).IsNone();
	};
	// 与 FVoxelPathSolver 的规则保持一致：格内净空 + 脚下支撑；单格版用略收缩的盒子避免"贴面误判占据"
	const auto CellBlockedByMesh = [this](const FIntVector& C)
	{
		const UWorld* World = GetWorld();
		if (!World)
		{
			return false;
		}
		TArray<FOverlapResult> Hits;
		World->OverlapMultiByObjectType(Hits, CoordToWorldLocation(C), FQuat::Identity,
			FCollisionObjectQueryParams(FCollisionObjectQueryParams::InitType::AllObjects), FCollisionShape::MakeBox(VoxelSize * 0.45f));
		for (const FOverlapResult& Hit : Hits)
		{
			const UPrimitiveComponent* Comp = Hit.Component.Get();
			const AActor* Owner = Comp ? Comp->GetOwner() : nullptr;
			if (!Comp || (Owner && Owner->IsA<APawn>()))
			{
				continue;		// Pawn/Character 身体或挂件不算地面障碍
			}
			if (Comp->IsA<UStaticMeshComponent>() || Comp->IsA<USkeletalMeshComponent>())
			{
				return true;
			}
		}
		return false;
	};

	for (int32 i = 0; i < AgentHeight; ++i)
	{
		const FIntVector C(Coord.X, Coord.Y, Coord.Z + i);
		if (VoxelSolidAt(C) || (bCheckStaticMeshes && CellBlockedByMesh(C)))
		{
			return false;
		}
	}
	const FIntVector Ground(Coord.X, Coord.Y, Coord.Z - 1);
	return VoxelSolidAt(Ground) || (bCheckStaticMeshes && CellBlockedByMesh(Ground));
}

void AVoxelTerrainActor::SetVoxelPathWeight(FIntVector Coord, float Weight)
{
	if (FMath::IsNaN(Weight) || Weight < 0.f)
	{
		UE_LOG(LogTemp, Warning, TEXT("%s：SetVoxelPathWeight 权重 (%f) 非法（只允许 >=0），已忽略"), *GetName(), Weight);
		return;
	}
	if (Weight == 1.f)
	{
		PathCostWeights.Remove(Coord);
	}
	else
	{
		PathCostWeights.Add(Coord, Weight);
	}
}

void AVoxelTerrainActor::ClearVoxelPathWeight(FIntVector Coord)
{
	PathCostWeights.Remove(Coord);
}

float AVoxelTerrainActor::GetVoxelPathWeight(FIntVector Coord) const
{
	const float* Found = PathCostWeights.Find(Coord);
	return Found ? *Found : 1.f;
}

void AVoxelTerrainActor::SetVoxelPathWeightBox(FIntVector Min, FIntVector Max, float Weight)
{
	if (FMath::IsNaN(Weight) || Weight < 0.f)
	{
		UE_LOG(LogTemp, Warning, TEXT("%s：SetVoxelPathWeightBox 权重 (%f) 非法（只允许 >=0），已忽略"), *GetName(), Weight);
		return;
	}
	const int32 LowX = FMath::Min(Min.X, Max.X), HighX = FMath::Max(Min.X, Max.X);
	const int32 LowY = FMath::Min(Min.Y, Max.Y), HighY = FMath::Max(Min.Y, Max.Y);
	const int32 LowZ = FMath::Min(Min.Z, Max.Z), HighZ = FMath::Max(Min.Z, Max.Z);

	const int64 Volume = (static_cast<int64>(HighX) - LowX + 1) * (static_cast<int64>(HighY) - LowY + 1) * (static_cast<int64>(HighZ) - LowZ + 1);
	if (Volume > MaxBulkWeightCells)
	{
		UE_LOG(LogTemp, Warning, TEXT("%s：SetVoxelPathWeightBox 请求 %lld 格，超过单次上限 %d，已忽略"),
			*GetName(), Volume, MaxBulkWeightCells);
		return;
	}
	// Z 不裁到 [MinHeight, MaxHeight)：越界权重永远命中不到可站格，无害；省去每格校验
	for (int32 z = LowZ; z <= HighZ; ++z)
	{
		for (int32 y = LowY; y <= HighY; ++y)
		{
			for (int32 x = LowX; x <= HighX; ++x)
			{
				const FIntVector C(x, y, z);
				if (Weight == 1.f)
				{
					PathCostWeights.Remove(C);
				}
				else
				{
					PathCostWeights.Add(C, Weight);
				}
			}
		}
	}
}
