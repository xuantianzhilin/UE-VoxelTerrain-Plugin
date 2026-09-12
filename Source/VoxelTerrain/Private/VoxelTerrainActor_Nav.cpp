// 导航数据的运行期侧：全量烘焙、区域通行权重、A* 查询。
// 烘焙只决定"能不能走"，代价（权重）是查询期按格乘上去的，两者互不依赖。

#include "VoxelTerrainActor.h"
#include "VoxelPathFollowingComponent.h"
#include "Algo/Reverse.h"
#include "VoxelChunk.h"

namespace
{
	/* 权重合法性：NaN 与负数一律拒收 —— 负权会破坏 A* 的代价单调性，NaN 会把比较全部带崩 */
	bool IsValidPathWeight(float Weight)
	{
		return !FMath::IsNaN(Weight) && Weight >= 0.f;
	}

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
		FVoxelPathPoint Point;
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

void AVoxelTerrainActor::BuildNavData()
{
	for (auto& [ChunkCoord, Chunk] : Chunks)
	{
		Chunk.BuildAllNavData(this);
	}
}

/* ===================== 区域通行权重 =====================
   权重是查询期数据：烘焙只决定“能不能走”，代价由寻路时按格乘上去，
   所以刷权重既不触发重烘、烘焙也不会读它。
   语义：1 = 正常，>1 = 更难走，0 = 软墙（格能站、但没人愿意绕过来）。
   表里只存与默认值 1 不同的格，等于 1 的写入按“清除”处理。 */

void AVoxelTerrainActor::SetCoordNavWeight(FIntVector Coord, float Weight)
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

void AVoxelTerrainActor::ClearCoordNavWeight(FIntVector Coord)
{
	NavWeights.Remove(Coord);
}

float AVoxelTerrainActor::GetCoordNavWeight(FIntVector Coord) const
{
	const float* Found = NavWeights.Find(Coord);
	return Found ? *Found : 1.f;
}

void AVoxelTerrainActor::SetCoordNavWeightBox(FIntVector Min, FIntVector Max, float Weight)
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

void AVoxelTerrainActor::ConfigureAutoNavLinks(bool bEnable, TSubclassOf<UVoxelNavLinkProxy> ProxyClass, const int32 MaxHeightDiff)
{
	if (bEnable && !ProxyClass)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Voxel] 打开自动连接但没给代理类，已忽略（连接会全部消失，寻路只能同层走）"));
		return;
	}

	bAutoSpawNavLink = bEnable;
	AutoLinkProxy = ProxyClass;
	NavLinkMaxHeightDiff = MaxHeightDiff;

	// 连接是烘在 NavData 里的，开关一改必须重烘才生效
	BuildNavData();
}

void AVoxelTerrainActor::AddLinkProxy(const FVoxelNavLinkProxyData& ProxyData, bool bRebuildNavData)
{
	LinkData.Add(ProxyData);
	if (bRebuildNavData)
	{
		RebuildLinksAround(ProxyData);
	}
}

void AVoxelTerrainActor::RemoveLinkProxy(const FVoxelNavLinkProxyData& ProxyData, bool bRebuildNavData)
{
	if (LinkData.Remove(ProxyData) > 0 && bRebuildNavData)
	{
		RebuildLinksAround(ProxyData);
	}
}

void AVoxelTerrainActor::RebuildLinksAround(const FVoxelNavLinkProxyData& ProxyData)
{
	if (!GetWorld()) return;
	if (!GetWorld()->IsGameWorld()) return;	// 编辑器里不烘导航，烘了也没用

	// 连接是烘在 NavData 里的，所以手动加/删之后要立刻把两端所在的那两格 Section 重烘一次，
	// 否则得等到有人改体素把它标脏才生效。这里只重烘导航，不动网格（网格跟连接无关）
	for (const FIntVector& EndPoint : { ProxyData.StartCoord, ProxyData.Destination })
	{
		const FIntVector SectionCoord = GetSectionCoordFromWorldCoord(EndPoint);
		if (FVoxelSection* Section = GetChunkSection(SectionCoord))
		{
			Section->BuildNavData(this);
		}
	}
}

/* ===================== AI 占地 =====================
   一张「格子 -> AI」的弱引用表，落实约定的「一个 AI 只占一格，一格同时只有一个 AI」。
   移动器在跨进下一格之前先把它登记下来（预约制），所以既不会出现两个 AI 挤同一格，
   也不会出现两个 AI 对穿（双方都想进对方那格时，慢的一方根本占不到，只能在原地等）。 */

bool AVoxelTerrainActor::TryOccupyCoord(FIntVector Coord, AActor* Occupant)
{
	if (GetCoordOccupant(Coord))
	{
		return false;
	}
	else
	{
		CoordRecords.Emplace(MoveTemp(Coord), Occupant);
		return true;
	}
}

void AVoxelTerrainActor::ReleaseCoord(const FIntVector& Coord, AActor* Occupant)
{
	if (GetCoordOccupant(Coord) == Occupant)
	{
		CoordRecords.Remove(Coord);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("[Voxel] %s 试图释放 (%d,%d,%d) 占地，但它并不是占用者，已忽略"),
			*Occupant->GetName(), Coord.X, Coord.Y, Coord.Z);
	}
}

AActor* AVoxelTerrainActor::GetCoordOccupant(const FIntVector& Coord) const
{
	if (auto* WeakActor = CoordRecords.Find(Coord))
	{
		if (AActor* Actor = WeakActor->Get())
		{
			return Actor;
		}
		else
		{
			CoordRecords.Remove(Coord);
			return nullptr;
		}
	}
	return nullptr;
}

const FVoxelNavCell* AVoxelTerrainActor::FindNavCell(const FIntVector& GlobalCoord) const
{
	const TPair<FIntVector, FIntVector> Split = WorldCoordToSectionLocalCoord(GlobalCoord);
	const FVoxelSection* Section = GetChunkSection(Split.Key);
	return Section ? Section->GetNavData().Find(Split.Value) : nullptr;
}

TArray<TPair<FIntVector, float>> AVoxelTerrainActor::FindFreeNearbyCoord(FIntVector Target, int32 AgentHeight, int32 Radius) const
{
	TArray<TPair<FIntVector, float>> Result;

	const int32 Height = FMath::Max(AgentHeight, 1);
	const int32 Range = FMath::Clamp(Radius, 0, MaxSnapRadius);

	for (int32 dz = -Range; dz <= Range; ++dz)
	{
		for (int32 dy = -Range; dy <= Range; ++dy)
		{
			for (int32 dx = -Range; dx <= Range; ++dx)
			{
				const FIntVector Candidate{ Target.X + dx, Target.Y + dy, Target.Z + dz };
				const FVoxelNavCell* Cell = FindNavCell(Candidate);
				if (Cell && Cell->AllowHeight >= AgentHeight && !GetCoordOccupant(Candidate))
				{
					Result.Emplace(Candidate, GetCoordNavWeight(Candidate));
				}
			}
		}
	}
	return Result;
}

/* ===================== A* 寻路 =====================
   只走烘焙好的导航图（FVoxelSection::NavData 里的正交 4 邻 Link），不涉及 NavMesh，查询期也不现查体素。
   图是无向的 —— 烘焙时对每一对走得通的邻格双向都写了 Link，所以只顺着 Links 前进就能到任意可达格。 */

TArray<FVoxelPathPoint> AVoxelTerrainActor::FindPath(FIntVector StartCoord, FIntVector EndCoord, int32 AgentHeight) const
{
	TArray<FVoxelPathPoint> Result;

	if (StartCoord == EndCoord)
	{
		Result.Emplace(StartCoord);
		return Result;
	}

	const int32 Height = FMath::Max(AgentHeight, 1);
	const FVoxelNavCell* StartCell = FindNavCell(StartCoord);
	const FVoxelNavCell* EndCell = FindNavCell(EndCoord);

	if (!StartCell || !EndCell)
	{
		// 有一端压根不是落脚格：这不是“绕不过去”，而是它自己就站不住，该由调用方先去吸附
		return Result;
	}
	if (StartCell->AllowHeight < Height || EndCell->AllowHeight < Height)
	{
		return Result;
	}

	TMap<FVoxelPathPoint, float> GScore;
	TMap<FVoxelPathPoint, FVoxelPathPoint> Parent;
	TSet<FVoxelPathPoint> Closed;
	FOpenHeap Open;

	GScore.Add(StartCoord, 0.f);
	Open.Push({ NavHeuristic(StartCoord, EndCoord), StartCoord });

	int32 Expansions = 0;
	FOpenNode Current;
	while (Open.Pop(Current))
	{
		if (Closed.Contains(Current.Point))
		{
			continue; // 惰性删除：这是同一个格子留下的旧副本
		}
		if (Current.Point.Coord == EndCoord)
		{
			// 终点在“弹出”时确认，配合一致性启发式，此时的 g 已经是到它的最短代价
			for (FVoxelPathPoint Node = Current.Point; ; )
			{
				Result.Emplace(Node);
				const FVoxelPathPoint* Prev = Parent.Find(Node);
				if (!Prev)
				{
					break; // 回到起点了
				}
				Node = *Prev;
			}
			Algo::Reverse(Result);
			return Result;
		}
		Closed.Add(Current.Point);

		if (++Expansions > MaxAStarExpansions)
		{
			UE_LOG(LogTemp, Warning, TEXT("[Voxel] 寻路 (%d,%d,%d)->(%d,%d,%d) 展开超过 %d 个节点仍未到达，已放弃（两点大概率不连通，或距离过远）"),
				StartCoord.X, StartCoord.Y, StartCoord.Z, EndCoord.X, EndCoord.Y, EndCoord.Z, MaxAStarExpansions);
			Result.Reset();
			return Result;
		}

		const float CurrentG = GScore[Current.Point];
		const FVoxelNavCell* Cell = FindNavCell(Current.Point.Coord);

		for (const FVoxelNavLink& Link : Cell->Links)
		{
			const FIntVector NextCoord = SectionLocalCoordToWorldCoord(Link.OwnnerSection, Link.LinkCoord);
			const FVoxelPathPoint NextPoint{ NextCoord, Link.ProxyClass };

			if (Closed.Contains(NextPoint))
			{
				continue;
			}

			const FVoxelNavCell* NextCell = FindNavCell(NextCoord);
			if (!NextCell || NextCell->AllowHeight < Height)
			{
				continue; // 对方站不下这个体型的 AI
			}

			if (GetCoordOccupant(NextCoord))
			{
				// 站着别的 AI。占地是随时间变的，所以只在查询期判，绝不写进烘焙数据；
				// 终点也走这条判定，所以「终点被人占了」会直接表现为找不到路，由调用方退到 4 邻
				continue;
			}

			// 权重按“进入这一格”计价，所以起点不计；0 是软墙，直接跳过
			const float Weight = GetCoordNavWeight(NextCoord);
			if (Weight <= 0.f)
			{
				continue;
			}
			// 只罚不奖：权重低于 1 也按 1 算，否则启发式“每步 >= 1”的下界就不成立了
			float StepCost = FMath::Max(1.f, Weight);

			if (Link.ProxyClass)
			{
				// 非平面连接：爬梯、跳台这些不该跟平地一个价，再乘一次代理自己的代价
				const UVoxelNavLinkProxy* ProxyCDO = Link.ProxyClass->GetDefaultObject<UVoxelNavLinkProxy>();
				if (!ProxyCDO || ProxyCDO->Weight <= 0.f)
				{
					continue;		// 类取不到，或代理把这条连接关掉了
				}
				StepCost *= FMath::Max(1.f, ProxyCDO->Weight);
			}
			const float Tentative = CurrentG + StepCost;

			if (const float* Existing = GScore.Find(NextPoint))
			{
				if (*Existing <= Tentative)
				{
					continue;
				}
			}
			GScore.Add(NextPoint, Tentative);
			Parent.Add(NextPoint, Current.Point);
			Open.Push({ Tentative + NavHeuristic(NextCoord, EndCoord), NextPoint });
		}
	}

	return Result; // 开放表空了：终点不在起点所在的连通块里
}
