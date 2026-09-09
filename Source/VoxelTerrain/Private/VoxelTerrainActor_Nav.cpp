// 导航数据的运行期侧：全量烘焙、区域通行权重、A* 查询。
// 烘焙只决定"能不能走"，代价（权重）是查询期按格乘上去的，两者互不依赖。

#include "VoxelTerrainActor.h"

#include "Algo/Reverse.h"
#include "VoxelChunk.h"

namespace
{
	/* 权重合法性：NaN 与负数一律拒收 —— 负权会破坏 A* 的代价单调性，NaN 会把比较全部带崩 */
	bool IsValidPathWeight(float Weight)
	{
		return !FMath::IsNaN(Weight) && Weight >= 0.f;
	}
}

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

/* ===================== 区域通行权重 =====================
   权重是查询期数据：烘焙只决定“能不能走”，代价由寻路时按格乘上去，
   所以刷权重既不触发重烘、烘焙也不会读它。
   语义：1 = 正常，>1 = 更难走，0 = 软墙（格能站、但没人愿意绕过来）。
   表里只存与默认值 1 不同的格，等于 1 的写入按“清除”处理。 */

void AVoxelTerrainActor::SetVoxelPathWeight(FIntVector Coord, float Weight)
{
	if (!IsValidPathWeight(Weight))
	{
		UE_LOG(LogTemp, Warning, TEXT("Voxel (%d,%d,%d) 的导航权重 %f 非法（允许 >= 0），已忽略"),
			Coord.X, Coord.Y, Coord.Z, Weight);
		return;
	}
	if (Coord.Z < MinHeight || Coord.Z >= MaxHeight)
	{
		UE_LOG(LogTemp, Warning, TEXT("Voxel (%d,%d,%d) 超出高度范围 [%d,%d)，权重已忽略"),
			Coord.X, Coord.Y, Coord.Z, MinHeight, MaxHeight);
		return;
	}

	if (FMath::IsNearlyEqual(Weight, 1.f))
	{
		NavWeights.Remove(Coord);		// 和默认值一样 = 没刷过
		return;
	}
	if (!NavWeights.Contains(Coord) && NavWeights.Num() >= MaxBulkWeightCells)
	{
		UE_LOG(LogTemp, Warning, TEXT("导航权重条目已达上限 %d 格，本次写入被忽略"), MaxBulkWeightCells);
		return;
	}

	NavWeights.Add(Coord, Weight);
}

void AVoxelTerrainActor::ClearVoxelPathWeight(FIntVector Coord)
{
	NavWeights.Remove(Coord);
}

float AVoxelTerrainActor::GetVoxelPathWeight(FIntVector Coord) const
{
	const float* Found = NavWeights.Find(Coord);
	return Found ? *Found : 1.f;
}

void AVoxelTerrainActor::SetVoxelPathWeightBox(FIntVector Min, FIntVector Max, float Weight)
{
	if (!IsValidPathWeight(Weight))
	{
		UE_LOG(LogTemp, Warning, TEXT("导航权重 %f 非法（允许 >= 0），批量写入已忽略"), Weight);
		return;
	}

	const FIntVector Lo(FMath::Min(Min.X, Max.X), FMath::Min(Min.Y, Max.Y), FMath::Min(Min.Z, Max.Z));
	const FIntVector Hi(FMath::Max(Min.X, Max.X), FMath::Max(Min.Y, Max.Y), FMath::Max(Min.Z, Max.Z));

	// Z 与有效高度范围求交集（越界的部分永远成不了落脚格，写进去只是白占内存）；XY 不裁
	const int32 LowZ = FMath::Max(Lo.Z, MinHeight);
	const int32 HighZ = FMath::Min(Hi.Z, MaxHeight - 1);
	if (LowZ > HighZ)
	{
		return;
	}

	const int64 Cells = static_cast<int64>(Hi.X - Lo.X + 1) * (Hi.Y - Lo.Y + 1) * (HighZ - LowZ + 1);
	const bool bClearAsDefault = FMath::IsNearlyEqual(Weight, 1.f);	// 刷 1 = 把这一片恢复成默认

	// 清除只减不加，不看上限；写入前先把总量拦住（估算偏保守：区间里已有的格会被覆盖而不是新增）
	if (!bClearAsDefault && static_cast<int64>(NavWeights.Num()) + Cells > MaxBulkWeightCells)
	{
		UE_LOG(LogTemp, Warning, TEXT("SetVoxelPathWeightBox 要写 %lld 格，加上已有的 %d 条会超过上限 %d，本次批量写入已忽略"),
			Cells, NavWeights.Num(), MaxBulkWeightCells);
		return;
	}

	for (int32 z = LowZ; z <= HighZ; ++z)
	{
		for (int32 y = Lo.Y; y <= Hi.Y; ++y)
		{
			for (int32 x = Lo.X; x <= Hi.X; ++x)
			{
				const FIntVector Coord(x, y, z);
				if (bClearAsDefault)
				{
					NavWeights.Remove(Coord);
				}
				else
				{
					NavWeights.Add(Coord, Weight);
				}
			}
		}
	}
}

/* ===================== A* 寻路 =====================
   只走烘焙好的导航图（FVoxelSection::NavData 里的正交 4 邻 Link），不涉及 NavMesh，查询期也不现查体素。
   图是无向的 —— 烘焙时对每一对走得通的邻格双向都写了 Link，所以只顺着 Links 前进就能到任意可达格。 */

namespace
{
	/** 单次查询最多展开多少个节点。导航图按 Chunk 无界铺开，绕不到终点时不能把主线程跑死 */
	constexpr int32 MaxAStarExpansions = 50000;

	/** FindNearbyNavCoord 允许的吸附半径上限：吸附是暴力扫立方体，半径 16 已经 4913 次查表了 */
	constexpr int32 MaxSnapRadius = 16;

	/**
	 * 可采纳启发式：一步只能在 X 或 Y 之一上前进 1 格（Z 最多跟着挪 1 格，垂直步可以和水平步重合），
	 * 于是“水平曼哈顿距离”和“垂直落差”各自都是步数下界，取二者较大仍不会高估真实代价。
	 * 单步代价 >= 1，所以它同时也是一致的（相邻两格的启发式差 <= 1），不需要重开已关闭的节点。
	 */
	float NavHeuristic(const FIntVector& From, const FIntVector& To)
	{
		const FIntVector D = To - From;
		const float Horizontal = static_cast<float>(FMath::Abs(D.X) + FMath::Abs(D.Y));
		const float Vertical = static_cast<float>(FMath::Abs(D.Z));
		return FMath::Max(Horizontal, Vertical);
	}

	/** 开放表里的一项。同一坐标允许重复入堆（找到更短路线时会再推一次），弹出时靠 Closed 去重 */
	struct FOpenNode
	{
		float FScore = 0.f;
		FIntVector Coord;
	};

	/** 二叉小顶堆。引擎 Core 里没有现成的优先队列（5.8 连 Algo 的堆操作都没有），这里手搓一个最小的：不支持改键，靠惰性删除 */
	class FOpenHeap
	{
	public:
		void Push(const FOpenNode& Node)
		{
			Nodes.Add(Node);
			int32 Index = Nodes.Num() - 1;
			while (Index > 0)
			{
				const int32 Parent = (Index - 1) / 2;
				if (Nodes[Parent].FScore <= Nodes[Index].FScore)
				{
					break;
				}
				Nodes.Swap(Index, Parent);
				Index = Parent;
			}
		}

		bool IsEmpty() const { return Nodes.Num() == 0; }

		bool Pop(FOpenNode& OutNode)
		{
			const int32 Count = Nodes.Num();
			if (Count == 0)
			{
				return false;
			}
			OutNode = MoveTemp(Nodes[0]);
			if (Count > 1)
			{
				// 末位补到堆顶再下沉（Count - 1 个元素参与调整，被移走的那个随后截掉）
				Nodes[0] = MoveTemp(Nodes[Count - 1]);
				const int32 Remain = Count - 1;
				int32 Index = 0;
				while (true)
				{
					const int32 Left = Index * 2 + 1;
					const int32 Right = Left + 1;
					int32 Smallest = Index;
					if (Left < Remain && Nodes[Left].FScore < Nodes[Smallest].FScore)
					{
						Smallest = Left;
					}
					if (Right < Remain && Nodes[Right].FScore < Nodes[Smallest].FScore)
					{
						Smallest = Right;
					}
					if (Smallest == Index)
					{
						break;
					}
					Nodes.Swap(Index, Smallest);
					Index = Smallest;
				}
			}
			Nodes.SetNum(Count - 1);
			return true;
		}

	private:
		TArray<FOpenNode> Nodes;
	};
}

const FVoxelNavCell* AVoxelTerrainActor::FindNavCell(const FIntVector& GlobalCoord) const
{
	const TPair<FIntVector, FIntVector> Split = WorldCoordToChunkLocalCoord(GlobalCoord);
	const FVoxelSection* Section = GetChunkSection(Split.Key);
	return Section ? Section->GetNavData().Find(Split.Value) : nullptr;
}

bool AVoxelTerrainActor::FindNearbyNavCoord(FIntVector Coord, const int32 AgentHeight, const int32 Radius, FIntVector& OutCoord) const
{
	OutCoord = FIntVector::ZeroValue;

	const int32 Height = FMath::Max(AgentHeight, 1);
	const int32 Range = FMath::Clamp(Radius, 0, MaxSnapRadius);

	int64 BestDistSq = -1;
	for (int32 dz = -Range; dz <= Range; ++dz)
	{
		for (int32 dy = -Range; dy <= Range; ++dy)
		{
			for (int32 dx = -Range; dx <= Range; ++dx)
			{
				const FIntVector Candidate(Coord.X + dx, Coord.Y + dy, Coord.Z + dz);
				const FVoxelNavCell* Cell = FindNavCell(Candidate);
				if (!Cell || Cell->AllowHeight < Height)
				{
					continue;
				}
				const int64 DistSq = static_cast<int64>(dx) * dx + static_cast<int64>(dy) * dy + static_cast<int64>(dz) * dz;
				if (BestDistSq < 0 || DistSq < BestDistSq)
				{
					BestDistSq = DistSq;
					OutCoord = Candidate;
				}
			}
		}
	}
	return BestDistSq >= 0;
}

bool AVoxelTerrainActor::FindPath(FIntVector StartCoord, FIntVector EndCoord, const int32 AgentHeight, TArray<FIntVector>& OutPath) const
{
	OutPath.Reset();

	const int32 Height = FMath::Max(AgentHeight, 1);

	const FVoxelNavCell* StartCell = FindNavCell(StartCoord);
	const FVoxelNavCell* EndCell = FindNavCell(EndCoord);
	if (!StartCell || !EndCell)
	{
		// 有一端压根不是落脚格：这不是“绕不过去”，而是它自己就站不住，该由调用方先去吸附
		return false;
	}
	if (StartCell->AllowHeight < Height || EndCell->AllowHeight < Height)
	{
		return false;
	}
	if (StartCoord == EndCoord)
	{
		OutPath.Add(StartCoord);
		return true;
	}

	TMap<FIntVector, float> GScore;
	TMap<FIntVector, FIntVector> CameFrom;
	TSet<FIntVector> Closed;
	FOpenHeap Open;

	GScore.Add(StartCoord, 0.f);
	Open.Push({ NavHeuristic(StartCoord, EndCoord), StartCoord });

	int32 Expansions = 0;
	FOpenNode Current;
	while (Open.Pop(Current))
	{
		if (Closed.Contains(Current.Coord))
		{
			continue; // 惰性删除：这是同一个格子留下的旧副本
		}
		if (Current.Coord == EndCoord)
		{
			// 终点在“弹出”时确认，配合一致性启发式，此时的 g 已经是到它的最短代价
			for (FIntVector Node = EndCoord; ; )
			{
				OutPath.Add(Node);
				const FIntVector* Prev = CameFrom.Find(Node);
				if (!Prev)
				{
					break; // 回到起点了
				}
				Node = *Prev;
			}
			Algo::Reverse(OutPath);
			return true;
		}
		Closed.Add(Current.Coord);

		if (++Expansions > MaxAStarExpansions)
		{
			UE_LOG(LogTemp, Warning, TEXT("[Voxel] 寻路 (%d,%d,%d)->(%d,%d,%d) 展开超过 %d 个节点仍未到达，已放弃（两点大概率不连通，或距离过远）"),
				StartCoord.X, StartCoord.Y, StartCoord.Z, EndCoord.X, EndCoord.Y, EndCoord.Z, MaxAStarExpansions);
			OutPath.Reset();
			return false;
		}

		const float CurrentG = GScore[Current.Coord];
		const FVoxelNavCell* Cell = FindNavCell(Current.Coord);
		if (!Cell)
		{
			continue; // 理论上不会发生：入堆前都确认过是落脚格
		}

		for (const FVoxelNavLink& Link : Cell->Links)
		{
			const FIntVector Next(
				Link.OwnnerSection.X * Voxel::LENGTH + Link.LinkCoord.X,
				Link.OwnnerSection.Y * Voxel::LENGTH + Link.LinkCoord.Y,
				Link.OwnnerSection.Z * Voxel::LENGTH + Link.LinkCoord.Z);

			if (Closed.Contains(Next))
			{
				continue;
			}

			const FVoxelNavCell* NextCell = FindNavCell(Next);
			if (!NextCell || NextCell->AllowHeight < Height)
			{
				continue; // 对方站不下这个体型的 AI
			}

			// 权重按“进入这一格”计价，所以起点不计；0 是软墙，直接跳过
			const float Weight = GetVoxelPathWeight(Next);
			if (Weight <= 0.f)
			{
				continue;
			}
			// 只罚不奖：权重低于 1 也按 1 算，否则启发式“每步 >= 1”的下界就不成立了
			const float Tentative = CurrentG + FMath::Max(1.f, Weight);

			if (const float* Existing = GScore.Find(Next))
			{
				if (*Existing <= Tentative)
				{
					continue;
				}
			}
			GScore.Add(Next, Tentative);
			CameFrom.Add(Next, Current.Coord);
			Open.Push({ Tentative + NavHeuristic(Next, EndCoord), Next });
		}
	}

	return false; // 开放表空了：终点不在起点所在的连通块里
}

bool AVoxelTerrainActor::FindPathWorld(const FVector StartLocation, const FVector EndLocation, const int32 AgentHeight, TArray<FVector>& OutPath) const
{
	OutPath.Reset();

	TArray<FIntVector> CoordPath;
	if (!FindPath(WorldLocationToCoord(StartLocation), WorldLocationToCoord(EndLocation), AgentHeight, CoordPath))
	{
		return false;
	}

	OutPath.Reserve(CoordPath.Num());
	for (const FIntVector& Coord : CoordPath)
	{
		OutPath.Add(CoordToWorldLocation(Coord));
	}
	return true;
}
