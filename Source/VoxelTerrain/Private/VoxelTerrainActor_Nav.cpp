// 导航数据的运行期侧：全量烘焙、区域通行权重、AI 占地与时空预约、A* 查询（空间 / 时空两种模式）。
// 烘焙只决定"能不能走"（按体型档），代价（权重）与预约是查询期按格叠加的，三者互不依赖。

#include "VoxelTerrainActor.h"
#include "VoxelPathFollowingComponent.h"
#include "Algo/Reverse.h"
#include "VoxelChunk.h"
#include "Engine/World.h"

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
   语义：1 = 正常，>1 = 更难走，0 = 软墙（格能站但没人愿意绕过来）。
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

/* ===================== 体型档与 footprint 口径 =====================
   档表归一化（升序、去重、恒含 1）是烘焙、寻路、占地共同的口径来源。 */

TArray<int32> AVoxelTerrainActor::GetNavFootprintWidths() const
{
	TArray<int32> Sorted = AgentFootprintWidths;
	for (int32& W : Sorted)
	{
		W = FMath::Clamp(W, 1, 8);		// 上限 8 < Section 边长 16：footprint 牵连不越过相邻 Section
	}
	Sorted.Add(1);						// 档 0 恒为 1×1（旧图），没配也得有
	Sorted.Sort();
	TArray<int32> Result;
	Result.Reserve(Sorted.Num());
	for (const int32 W : Sorted)
	{
		if (Result.IsEmpty() || Result.Last() != W)		// 排序后线性去重（UE 的 TArray 没有 RemoveDuplicates）
		{
			Result.Add(W);
		}
	}
	return Result;
}

int32 AVoxelTerrainActor::ResolveNavTier(int32 Width) const
{
	// 不做「就近取档」：请求没烘的体型是配置错误，悄悄换成别的体型比找不到路更危险
	return GetNavFootprintWidths().IndexOfByKey(FMath::Clamp(Width, 1, 8));
}

int32 AVoxelTerrainActor::GetMaxAgentFootprintWidth() const
{
	const TArray<int32> Widths = GetNavFootprintWidths();
	return Widths.Last();				// 升序且恒含 1，末尾即最大
}

void AVoxelTerrainActor::ConfigureAgentSizes(const TArray<int32>& Widths)
{
	AgentFootprintWidths = Widths;
	// footprint 分档是烘在 NavData 里的，改表必须重烘才生效（量级同 ConfigureAutoNavLinks，不是廉价操作）
	BuildNavData();
}

FIntVector AVoxelTerrainActor::FootprintOrigin(const FIntVector& Anchor, int32 Width)
{
	const int32 Half = FMath::Max(Width, 1) / 2;
	return Anchor - FIntVector(Half, Half, 0);
}

FVector AVoxelTerrainActor::FootprintCenterToWorld(FIntVector Anchor, int32 AgentWidth) const
{
	FVector Loc = CoordToWorldLocation(Anchor);
	if ((FMath::Max(AgentWidth, 1) & 1) == 0)
	{
		// 偶数档：footprint 中心在 anchor 格心的 -X/-Y 各半格处（覆盖方向见 FootprintOrigin）
		Loc.X -= VoxelSize.X * 0.5;
		Loc.Y -= VoxelSize.Y * 0.5;
	}
	return Loc;
}

FIntVector AVoxelTerrainActor::WorldToFootprintAnchor(const FVector& WorldLocation, int32 AgentWidth) const
{
	FVector Local = GetActorTransform().InverseTransformPosition(WorldLocation) / VoxelSize;
	// FootprintCenterToWorld 的逆：奇数档中心在格心（Local = anchor + 0.5），偶数档正落在 anchor 整数位
	const double Bias = (FMath::Max(AgentWidth, 1) & 1) ? 0.5 : 0.0;
	return FIntVector(
		FMath::RoundToInt32(Local.X - Bias),
		FMath::RoundToInt32(Local.Y - Bias),
		FMath::FloorToInt32(Local.Z));
}

const FVoxelNavCell* AVoxelTerrainActor::FindNavCellAtTier(const FIntVector& GlobalCoord, int32 Tier) const
{
	const TPair<FIntVector, FIntVector> Split = WorldCoordToSectionLocalCoord(GlobalCoord);
	const FVoxelSection* Section = GetChunkSection(Split.Key);
	return Section ? Section->GetNavTierData(Tier).Find(Split.Value) : nullptr;
}

const FVoxelNavCell* AVoxelTerrainActor::FindNavCell(const FIntVector& GlobalCoord, int32 AgentWidth) const
{
	const int32 Tier = ResolveNavTier(AgentWidth);
	return Tier == INDEX_NONE ? nullptr : FindNavCellAtTier(GlobalCoord, Tier);
}

/* ===================== AI 占地 =====================
   一张「格子 -> AI」的弱引用表，落实「一格同时只有一个 AI」。宽体型的 AI 整体占住它的 footprint
   （覆盖的每一格都登记），所以别人的寻路/让行判定只看单格占用表就天然正确。
   移动器在跨进下一格之前先把它占下来（先占后走，见 UVoxelPathFollowingComponent），
   所以既不会出现两个 AI 挤同一格，也不会对穿（双方都想进对方那格时，谁都没拿到，只能原地等）。 */

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

bool AVoxelTerrainActor::TryOccupyFootprint(FIntVector Anchor, int32 AgentWidth, AActor* Occupant)
{
	const int32 W = FMath::Max(AgentWidth, 1);
	const FIntVector Origin = FootprintOrigin(Anchor, W);

	// 先整块校验再整块写入：任何一格被**别人**占着就一格都不写（同一 Occupant 已占的格视作已有）
	for (int32 y = 0; y < W; ++y)
	{
		for (int32 x = 0; x < W; ++x)
		{
			if (AActor* Holder = GetCoordOccupant(Origin + FIntVector(x, y, 0)))
			{
				if (Holder != Occupant)
				{
					return false;
				}
			}
		}
	}
	for (int32 y = 0; y < W; ++y)
	{
		for (int32 x = 0; x < W; ++x)
		{
			CoordRecords.Emplace(Origin + FIntVector(x, y, 0), Occupant);
		}
	}
	return true;
}

void AVoxelTerrainActor::ReleaseFootprint(FIntVector Anchor, int32 AgentWidth, AActor* Occupant)
{
	const int32 W = FMath::Max(AgentWidth, 1);
	const FIntVector Origin = FootprintOrigin(Anchor, W);
	for (int32 y = 0; y < W; ++y)
	{
		for (int32 x = 0; x < W; ++x)
		{
			const FIntVector Cell = Origin + FIntVector(x, y, 0);
			if (GetCoordOccupant(Cell) == Occupant)		// 不是自己占的（出生时就被挡的格）静默跳过，不打高频日志
			{
				CoordRecords.Remove(Cell);
			}
		}
	}
}

bool AVoxelTerrainActor::IsFootprintBlocked(FIntVector Anchor, int32 AgentWidth, const AActor* IgnoreAgent) const
{
	const int32 W = FMath::Max(AgentWidth, 1);
	const FIntVector Origin = FootprintOrigin(Anchor, W);
	for (int32 y = 0; y < W; ++y)
	{
		for (int32 x = 0; x < W; ++x)
		{
			const AActor* Holder = GetCoordOccupant(Origin + FIntVector(x, y, 0));
			if (Holder && Holder != IgnoreAgent)
			{
				return true;
			}
		}
	}
	return false;
}

TArray<TPair<FIntVector, float>> AVoxelTerrainActor::FindFreeNearbyCoord(FIntVector Target, int32 AgentHeight, int32 Radius, int32 AgentWidth) const
{
	TArray<TPair<FIntVector, float>> Result;

	const int32 Tier = ResolveNavTier(AgentWidth);
	if (Tier == INDEX_NONE)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Voxel] 吸附按体型 %d 格查询，但地形没烘这一档，返回空（检查 AgentFootprintWidths）"), FMath::Max(AgentWidth, 1));
		return Result;
	}

	const int32 Height = FMath::Max(AgentHeight, 1);
	const int32 Range = FMath::Clamp(Radius, 0, MaxSnapRadius);

	for (int32 dz = -Range; dz <= Range; ++dz)
	{
		for (int32 dy = -Range; dy <= Range; ++dy)
		{
			for (int32 dx = -Range; dx <= Range; ++dx)
			{
				const FIntVector Candidate{ Target.X + dx, Target.Y + dy, Target.Z + dz };
				const FVoxelNavCell* Cell = FindNavCellAtTier(Candidate, Tier);
				if (Cell && Cell->AllowHeight >= Height && !IsFootprintBlocked(Candidate, AgentWidth))
				{
					Result.Emplace(Candidate, GetCoordNavWeight(Candidate));
				}
			}
		}
	}
	return Result;
}

/* ===================== 时空预约（群体避让的软规划层） =====================
   预约记录「某 AI 预计何时占某格」。规划期（FindPathScheduled / CommitPathReservations）按时间窗绕开它；
   运行期的正确性由逐格硬占地（先占后走）兜底 —— 实际早到/晚到只会影响流畅度，不会造成重叠。 */

float AVoxelTerrainActor::HopDurationSeconds(const FVoxelPathPoint& ToPoint, float BaseSecondsPerStep) const
{
	const float Base = FMath::Max(BaseSecondsPerStep, 0.01f);
	const float Weight = FMath::Max(1.f, GetCoordNavWeight(ToPoint.Coord));	// 只罚不奖，与代价口径一致
	if (ToPoint.LinkClass)
	{
		const UVoxelNavLinkProxy* ProxyCDO = ToPoint.LinkClass->GetDefaultObject<UVoxelNavLinkProxy>();
		if (ProxyCDO && ProxyCDO->Weight > 0.f)
		{
			return FMath::Max(Base, ProxyCDO->ExpectedMoveDuration * FMath::Max(1.f, ProxyCDO->Weight) * Weight);
		}
	}
	return Base * Weight;
}

void AVoxelTerrainActor::PruneExpiredReservations(float NowSeconds)
{
	static constexpr float Grace = 2.f;		// 过窗后再留一会儿：正在被读的那条别在循环中途凭空消失
	for (auto It = Reservations.CreateIterator(); It; ++It)
	{
		It->Value.RemoveAll([NowSeconds](const FNavReservation& R)
			{
				return R.tExit + Grace < NowSeconds || !R.Agent.IsValid();
			});
		if (It->Value.IsEmpty())
		{
			It.RemoveCurrent();
		}
	}
}

bool AVoxelTerrainActor::IsCoordReservedAt(const FIntVector& Coord, float tEnter, float tExit, const AActor* IgnoreAgent, const FIntVector* SwapFrom) const
{
	// 窗口相接不算冲突：一条预约恰好在对方进入的瞬间结束（先出后进）是合法的接力
	auto ScanKey = [&](const FIntVector& Key, bool bSwapCheck)
	{
		const TArray<FNavReservation>* List = Reservations.Find(Key);
		if (!List)
		{
			return false;
		}
		for (const FNavReservation& R : *List)
		{
			if (R.tExit <= tEnter || R.tEnter >= tExit)
			{
				continue;						// 时间窗不重叠
			}
			if (bSwapCheck && R.From != Coord)	// 对穿判定：对方的移动是 Coord -> Key，我方是 Key -> Coord
			{
				continue;
			}
			const AActor* Owner = R.Agent.Get();
			if (!Owner || (IgnoreAgent && Owner == IgnoreAgent))
			{
				continue;						// 已失效（等着被清扫）或自己的预约
			}
			return true;
		}
		return false;
	};

	if (ScanKey(Coord, false))
	{
		return true;
	}
	return SwapFrom && ScanKey(*SwapFrom, true);
}

bool AVoxelTerrainActor::CommitPathReservations(AActor* Agent, const TArray<FVoxelPathPoint>& Path, int32 AgentWidth,
	float SecondsPerStep, float NowSeconds, float& OutTravelTime)
{
	OutTravelTime = 0.f;
	if (!Agent || Path.Num() <= 1)
	{
		return false;		// 零跳（原地对齐）不必登记
	}

	const float Base = FMath::Max(SecondsPerStep, 0.01f);
	const int32 W = FMath::Max(AgentWidth, 1);
	// 终点格的尾窗：到位后还会站一会儿（别人规划时要把这段时间也算成占用）
	const float Tail = FMath::Max(Base, 0.5f);

	// 1) 先排时刻表：Times[i] = 预计到达第 i 个点的时刻（与 FindPathScheduled 共享 HopDurationSeconds 口径）
	TArray<float> Times;
	Times.SetNumUninitialized(Path.Num());
	Times[0] = NowSeconds;
	for (int32 i = 1; i < Path.Num(); ++i)
	{
		Times[i] = Times[i - 1] + HopDurationSeconds(Path[i], Base);
	}
	OutTravelTime = Times.Last() - NowSeconds;

	auto WindowOfNode = [&](int32 i, float& OutEnter, float& OutExit)
	{
		OutEnter = Times[i];
		OutExit = (i + 1 < Path.Num()) ? Times[i + 1] : Times[i] + Tail;
	};

	// 2) 整条全部冲突检测通过才写入（原子提交）：宁可这次不登记（退回纯空间避让），也不登记半条误导别人
	for (int32 i = 0; i < Path.Num(); ++i)
	{
		float tEnter = 0.f, tExit = 0.f;
		WindowOfNode(i, tEnter, tExit);
		const bool bHasHop = i > 0;
		const FIntVector Prev = bHasHop ? Path[i - 1].Coord : FIntVector::ZeroValue;
		const FIntVector Origin = FootprintOrigin(Path[i].Coord, W);
		for (int32 y = 0; y < W; ++y)
		{
			for (int32 x = 0; x < W; ++x)
			{
				const FIntVector Cell = Origin + FIntVector(x, y, 0);
				if (IsCoordReservedAt(Cell, tEnter, tExit, Agent, bHasHop ? &Prev : nullptr))
				{
					return false;
				}
			}
		}
	}

	// 3) 登记：footprint 展开到覆盖的每一格
	for (int32 i = 0; i < Path.Num(); ++i)
	{
		float tEnter = 0.f, tExit = 0.f;
		WindowOfNode(i, tEnter, tExit);
		const FIntVector From = i > 0 ? Path[i - 1].Coord : Path[0].Coord;
		const FIntVector Origin = FootprintOrigin(Path[i].Coord, W);
		for (int32 y = 0; y < W; ++y)
		{
			for (int32 x = 0; x < W; ++x)
			{
				FNavReservation& Slot = Reservations.FindOrAdd(Origin + FIntVector(x, y, 0)).AddDefaulted_GetRef();
				Slot.Agent = Agent;
				Slot.tEnter = tEnter;
				Slot.tExit = tExit;
				Slot.From = From;
			}
		}
	}
	return true;
}

void AVoxelTerrainActor::RemoveReservationsFor(const AActor* Agent)
{
	if (!Agent)
	{
		return;
	}
	for (auto It = Reservations.CreateIterator(); It; ++It)
	{
		It->Value.RemoveAll([Agent](const FNavReservation& R) { return R.Agent.Get() == Agent; });
		if (It->Value.IsEmpty())
		{
			It.RemoveCurrent();
		}
	}
}

/* ===================== A* 寻路 =====================
   只走烘焙好的导航图（按体型档取 FVoxelSection::NavDataByTier 里的正交 4 邻 Link），不涉及 NavMesh，
   查询期也不现查体素。图是无向的 —— 烘焙时对每一对走得通的邻格双向都写了 Link，只顺着 Links 就能到任意可达格。
   两种模式共享同一套展开逻辑：
     - 空间模式（FindPath）：代价是抽象单位（权重只罚不奖，启发式按「一步至少一个单位」），避让只看逐格硬占用；
     - 时空模式（FindPathScheduled）：代价就是秒（G 值 = 预计到达时刻），在硬占用之外再躲开预约表的时间窗与对穿。
   时空模式不安排「原地等待」（那要给状态加时间维）；真撞上时由跟随组件的先占后走/让行兜底。 */

TArray<FVoxelPathPoint> AVoxelTerrainActor::FindPath(FIntVector StartCoord, FIntVector EndCoord, int32 AgentHeight, int32 AgentWidth, const AActor* IgnoreAgent) const
{
	float Unused = 0.f;
	return FindPathInternal(StartCoord, EndCoord, AgentHeight, AgentWidth, IgnoreAgent, false, 0.f, 0.f, Unused);
}

TArray<FVoxelPathPoint> AVoxelTerrainActor::FindPathScheduled(FIntVector StartCoord, FIntVector EndCoord, int32 AgentHeight, int32 AgentWidth,
	const AActor* Agent, float SecondsPerStep, float StartDelay, float& OutTravelTime) const
{
	return FindPathInternal(StartCoord, EndCoord, AgentHeight, AgentWidth, Agent, true, SecondsPerStep, StartDelay, OutTravelTime);
}

TArray<FVoxelPathPoint> AVoxelTerrainActor::FindPathInternal(FIntVector StartCoord, FIntVector EndCoord, int32 AgentHeight, int32 AgentWidth,
	const AActor* IgnoreAgent, bool bScheduled, float SecondsPerStep, float StartDelay, float& OutTravelTime) const
{
	TArray<FVoxelPathPoint> Result;

	const int32 Tier = ResolveNavTier(AgentWidth);
	if (Tier == INDEX_NONE)
	{
		UE_LOG(LogTemp, Warning, TEXT("[Voxel] 寻路按体型 %d 格查询，但地形没烘这一档，返回空（检查 AgentFootprintWidths）"), FMath::Max(AgentWidth, 1));
		return Result;
	}
	const int32 Width = FMath::Max(AgentWidth, 1);

	if (StartCoord == EndCoord)
	{
		Result.Emplace(StartCoord);
		return Result;
	}

	// 时空模式：代价口径就是「秒」，启发式按基准步时缩放下界；空间模式维持「一步至少一个单位」的旧口径
	const float BaseSeconds = FMath::Max(SecondsPerStep, 0.01f);
	const float StepUnit = bScheduled ? BaseSeconds : 1.f;

	const int32 Height = FMath::Max(AgentHeight, 1);
	const FVoxelNavCell* StartCell = FindNavCellAtTier(StartCoord, Tier);
	const FVoxelNavCell* EndCell = FindNavCellAtTier(EndCoord, Tier);

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

	const float StartG = bScheduled ? StartDelay : 0.f;
	GScore.Add(StartCoord, StartG);
	Open.Push({ StartG + NavHeuristic(StartCoord, EndCoord) * StepUnit, StartCoord });

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
			if (bScheduled)
			{
				OutTravelTime = GScore[Current.Point] - StartDelay;
			}
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
		const FVoxelNavCell* Cell = FindNavCellAtTier(Current.Point.Coord, Tier);

		for (const FVoxelNavLink& Link : Cell->Links)
		{
			const FIntVector NextCoord = SectionLocalCoordToWorldCoord(Link.OwnnerSection, Link.LinkCoord);
			const FVoxelPathPoint NextPoint{ NextCoord, Link.ProxyClass };

			if (Closed.Contains(NextPoint))
			{
				continue;
			}

			const FVoxelNavCell* NextCell = FindNavCellAtTier(NextCoord, Tier);
			if (!NextCell || NextCell->AllowHeight < Height)
			{
				continue; // 这一档站不下这个身高的 AI
			}

			if (IsFootprintBlocked(NextCoord, Width, IgnoreAgent))
			{
				// footprint 里有别的 AI 占着的格（终点同理：「终点被人占了」表现为找不到路，由调用方退到邻格吸附）。
				// 占地与预约都是随时间变的，只在查询期判，绝不写进烘焙数据
				continue;
			}

			// 权重按“进入这一格”计价，所以起点不计；0 是软墙，直接跳过
			const float Weight = GetCoordNavWeight(NextCoord);
			if (Weight <= 0.f)
			{
				continue;
			}
			// 只罚不奖：权重低于 1 也按 1 算，否则启发式“每步 >= 一个单位”的下界就不成立了
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
			if (bScheduled)
			{
				// 时空模式换成秒计：与 CommitPathReservations 共用同一份 HopDurationSeconds 口径
				StepCost = HopDurationSeconds(NextPoint, SecondsPerStep);
			}
			const float Tentative = CurrentG + StepCost;

			if (bScheduled && IgnoreAgent)
			{
				// 预约冲突：预计到达时刻落在别人登记的窗口里（或对穿），这一跳按走不得处理。
				// 展开期只能量到「到达后至少一步」的最短驻留；完整窗口的裁决在 Commit 时做
				const FIntVector FromCoord = Current.Point.Coord;
				if (IsCoordReservedAt(NextCoord, Tentative, Tentative + BaseSeconds, IgnoreAgent, &FromCoord))
				{
					continue;
				}
			}

			if (const float* Existing = GScore.Find(NextPoint))
			{
				if (*Existing <= Tentative)
				{
					continue;
				}
			}
			GScore.Add(NextPoint, Tentative);
			Parent.Add(NextPoint, Current.Point);
			Open.Push({ Tentative + NavHeuristic(NextCoord, EndCoord) * StepUnit, NextPoint });
		}
	}

	return Result; // 开放表空了：终点不在起点所在的连通块里
}
