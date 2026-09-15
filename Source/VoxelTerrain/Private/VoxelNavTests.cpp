// VoxelTerrain 导航回归/诊断测试：体型分档烘焙、时空预约、先占后走、footprint 占地。
// 纯代码搭地形（不依赖关卡与资产），手动喂 AdvanceFollowing，不跑世界 Tick。

#if WITH_AUTOMATION_TESTS

#include "CoreMinimal.h"
#include "Misc/AutomationTest.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Components/SceneComponent.h"
#include "VoxelTerrainActor.h"
#include "VoxelChunk.h"
#include "VoxelPathFollowingComponent.h"

#define VOXEL_NAV_TEST(Flags) (EAutomationTestFlags::EditorContext | EAutomationTestFlags::ProductFilter)

namespace VoxelNavTest
{
	/** RAII：建一个游戏世界（不 BeginPlay），析构时整体销毁 */
	struct FTestWorld
	{
		UWorld* World = nullptr;

		bool Create(const FString& Name, FAutomationTestBase& Test)
		{
			World = UWorld::CreateWorld(EWorldType::Game, /*bFromEditor*/false, FName(*Name));
			if (!World)
			{
				Test.AddError(TEXT("CreateWorld 失败"));
				return false;
			}
			return true;
		}

		~FTestWorld()
		{
			if (World && GEngine)
			{
				GEngine->DestroyWorldContext(World);
				World->DestroyWorld(false);
			}
		}
	};

	/** 铺一块 16×16 以内的平坦场地（地面 z=0，落脚层 z=1），并按需挖掉若干落脚列 */
	static AVoxelTerrainActor* SpawnFlatTerrain(UWorld* World, FAutomationTestBase& Test, const TArray<int32>& Sizes)
	{
		AVoxelTerrainActor* Terrain = World->SpawnActor<AVoxelTerrainActor>(FVector::ZeroVector, FRotator::ZeroRotator);
		if (!Terrain)
		{
			Test.AddError(TEXT("SpawnActor<AVoxelTerrainActor> 失败"));
			return nullptr;
		}
		if (Sizes.Num() > 0)
		{
			Terrain->ConfigureAgentSizes(Sizes);		// 改档表并全量重烘（此时还没有体素，实际烘在下面）
		}
		Terrain->FillBox({ -8, -8, 0 }, { 7, 7, 0 }, TEXT("Rock"));
		Terrain->BuildNavData();
		return Terrain;
	}

	static AActor* SpawnAgentAt(UWorld* World, AVoxelTerrainActor* Terrain, const FIntVector& At, int32 Width, UVoxelPathFollowingComponent** OutComp)
	{
		AActor* Actor = World->SpawnActor<AActor>();
		if (!Actor)
		{
			return nullptr;
		}
		// 裸 AActor 没有根组件，SetActorLocation 会静默失败 —— 给它一个最小的场景根
		USceneComponent* Root = NewObject<USceneComponent>(Actor);
		Actor->SetRootComponent(Root);
		Root->RegisterComponent();
		Actor->SetActorLocation(Terrain->FootprintCenterToWorld(At, Width));

		UVoxelPathFollowingComponent* Comp = NewObject<UVoxelPathFollowingComponent>(Actor);
		Comp->RegisterComponent();
		Comp->Terrain = Terrain;
		Comp->AgentWidth = Width;
		Comp->MoveSpeed = 1000.f;			// VoxelSize=100 → 喂 dt=0.1 就是一步一格，时序完全确定
		if (OutComp)
		{
			*OutComp = Comp;
		}
		return Actor;
	}
}

/* ===================== 体型分档烘焙 ===================== */

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVoxelNavTierBakeTest, "VoxelTerrain.Nav.TierBake", VOXEL_NAV_TEST())

bool FVoxelNavTierBakeTest::RunTest(const FString& Parameters)
{
	using namespace VoxelNavTest;
	FTestWorld TW;
	if (!TW.Create(TEXT("VoxelNavTierBake"), *this))
	{
		return false;
	}
	AVoxelTerrainActor* Terrain = SpawnFlatTerrain(TW.World, *this, { 1, 2 });
	if (!Terrain)
	{
		return false;
	}

	// 隔墙：x=3、y 只留 [-2..0]，y∈[1..7] 处有洞 → 通道（y=-2..0 一侧）只有 3 格宽，2×2 能绕；
	// 改成只留一格开口（y=0），另一格（y=1 以下全封）→ 1 格隘口
	for (int32 y = -8; y <= 7; ++y)
	{
		if (y != 0)
		{
			Terrain->SetVoxel({ 3, y, 1 }, TEXT("Rock"));		// 占掉落脚层：该列不可站
		}
	}
	// 隘口上方再压一格，顺带验证竖直净空
	Terrain->SetVoxel({ 3, 0, 2 }, TEXT("Rock"));
	Terrain->BuildNavData();

	// 1 格隘口：宽 1 的落脚点存在（(3,0,1) 两侧连通），宽 2 的 anchor 覆盖到被封列，不存在
	TestNotNull(TEXT("隘口落脚格(宽1)"), const_cast<const FVoxelNavCell*>(Terrain->FindNavCell({ 3, 0, 1 }, 1)));
	TestNull(TEXT("隘口落脚格(宽2)"), const_cast<const FVoxelNavCell*>(Terrain->FindNavCell({ 3, 1, 1 }, 2)));
	TestNull(TEXT("隘口另一侧 anchor(宽2)"), const_cast<const FVoxelNavCell*>(Terrain->FindNavCell({ 3, 0, 1 }, 2)));

	// 隘口两侧各放一个能走的点：宽 1 寻路通，宽 2 寻路断
	const int32 Height1 = 1;
	const TArray<FVoxelPathPoint> PathW1 = Terrain->FindPath({ 0, 0, 1 }, { 6, 0, 1 }, Height1, 1);
	TestTrue(TEXT("宽1 穿过隘口"), PathW1.Num() > 3);
	const TArray<FVoxelPathPoint> PathW2 = Terrain->FindPath({ 0, 0, 1 }, { 6, 0, 1 }, Height1, 2);
	TestTrue(TEXT("宽2 被 1 格隘口挡住（空路径）"), PathW2.IsEmpty());

	// 开阔地：宽 2 应能走，且净空是覆盖列 min —— 压一格的顶（(2,2) 列上方放石头），
	// 覆盖它的 2×2 anchor（如 (2,3)：覆盖 (1,2)(2,2)(1,3)(2,3)）净空应被拉到 1
	Terrain->SetVoxel({ 2, 2, 2 }, TEXT("Rock"));
	Terrain->BuildNavData();
	const FVoxelNavCell* OpenW2 = Terrain->FindNavCell({ 5, 5, 1 }, 2);
	TestNotNull(TEXT("开阔地 2×2 落脚点"), OpenW2);
	if (OpenW2)
	{
		TestEqual(TEXT("开阔地净空封顶"), OpenW2->AllowHeight, Terrain->GetMaxAllowHeight());
	}
	const FVoxelNavCell* NearLow = Terrain->FindNavCell({ 2, 3, 1 }, 2);
	TestNotNull(TEXT("压顶列附近的 2×2 落脚点"), NearLow);
	if (NearLow)
	{
		TestEqual(TEXT("宽2 净空 = 覆盖列 min（被压顶列拉到 1）"), NearLow->AllowHeight, 1);
	}
	// 身高 2 的宽 2 AI：min 净空 1 < 2 → 连开阔绕路都迈不开（墙本身也拦一切，这里断言为空即可）
	const TArray<FVoxelPathPoint> PathW2Tall = Terrain->FindPath({ 0, 0, 1 }, { 6, 0, 1 }, 2, 2);
	TestTrue(TEXT("宽2 身高2 无路"), PathW2Tall.IsEmpty());
	// 宽 1 走 (3,0) 上方净空 1：身高 2 也过不了隘口，但绕不开（墙）→ 空
	const TArray<FVoxelPathPoint> PathW1Tall = Terrain->FindPath({ 0, 0, 1 }, { 6, 0, 1 }, 2, 1);
	TestTrue(TEXT("宽1 身高2 也被压顶隘口挡住"), PathW1Tall.IsEmpty());

	// 没烘的档位明确拒绝
	const TArray<FVoxelPathPoint> PathNoTier = Terrain->FindPath({ 0, 0, 1 }, { 6, 0, 1 }, 1, 4);
	TestTrue(TEXT("未配置体型 4 返回空"), PathNoTier.IsEmpty());

	// anchor/站位互逆：偶数档偏移半格
	const FVector Center2 = Terrain->FootprintCenterToWorld({ 4, 1, 1 }, 2);
	const FIntVector Back2 = Terrain->WorldToFootprintAnchor(Center2, 2);
	TestEqual(TEXT("宽2 anchor↔站位互逆 X"), Back2.X, 4);
	TestEqual(TEXT("宽2 anchor↔站位互逆 Y"), Back2.Y, 1);
	const FVector Center3 = Terrain->FootprintCenterToWorld({ 4, 1, 1 }, 3);
	TestEqual(TEXT("宽3 站位=格心"), Center3.X, Terrain->CoordToWorldLocation({ 4, 1, 1 }).X);
	const FIntVector Back3 = Terrain->WorldToFootprintAnchor(Center3, 3);
	TestEqual(TEXT("宽3 anchor↔站位互逆"), Back3.Y, 1);
	return true;
}

/* ===================== 档 0 回归（与旧 1×1 图逐点一致） ===================== */

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVoxelNavTierRegressionTest, "VoxelTerrain.Nav.TierRegression", VOXEL_NAV_TEST())

bool FVoxelNavTierRegressionTest::RunTest(const FString& Parameters)
{
	using namespace VoxelNavTest;
	FTestWorld TW;
	if (!TW.Create(TEXT("VoxelNavTierRegression"), *this))
	{
		return false;
	}
	// 默认体型档（只含 1）：一条直线走廊的寻路结果必须还是逐格直线，连接不带代理
	AVoxelTerrainActor* Terrain = SpawnFlatTerrain(TW.World, *this, {});
	if (!Terrain)
	{
		return false;
	}

	const TArray<FVoxelPathPoint> Path = Terrain->FindPath({ 0, 0, 1 }, { 3, 0, 1 }, 1);
	TestEqual(TEXT("路径点数"), Path.Num(), 4);
	if (Path.Num() == 4)
	{
		for (int32 i = 0; i < 4; ++i)
		{
			TestEqual(*FString::Printf(TEXT("路径第 %d 点 X"), i), Path[i].Coord.X, i);
			TestTrue(*FString::Printf(TEXT("路径第 %d 点无代理"), i), Path[i].LinkClass == nullptr);
		}
	}
	// 单格档的净空口径：开阔地 = 封顶值
	const FVoxelNavCell* Open = Terrain->FindNavCell({ 0, 0, 1 });
	TestNotNull(TEXT("落脚格存在"), const_cast<const FVoxelNavCell*>(Open));
	if (Open)
	{
		TestEqual(TEXT("净空封顶"), Open->AllowHeight, Terrain->GetMaxAllowHeight());
	}
	return true;
}

/* ===================== 时空预约 ===================== */

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVoxelNavReservationTest, "VoxelTerrain.Nav.Reservations", VOXEL_NAV_TEST())

bool FVoxelNavReservationTest::RunTest(const FString& Parameters)
{
	using namespace VoxelNavTest;
	FTestWorld TW;
	if (!TW.Create(TEXT("VoxelNavReservation"), *this))
	{
		return false;
	}
	AVoxelTerrainActor* Terrain = SpawnFlatTerrain(TW.World, *this, { 1 });
	if (!Terrain)
	{
		return false;
	}

	// 一格宽的桥：只剩 (0,0)-(1,0) 两格（两端之外也封死）
	for (int32 x = -8; x <= 7; ++x)
	{
		for (int32 y = -8; y <= 7; ++y)
		{
			if (y != 0 || x < 0 || x > 1)
			{
				Terrain->SetVoxel({ x, y, 1 }, TEXT("Rock"));
			}
		}
	}
	Terrain->BuildNavData();

	AActor* AgentA = TW.World->SpawnActor<AActor>();
	AActor* AgentB = TW.World->SpawnActor<AActor>();

	// A 登记 (0,0) -> (1,0) 的预约：1 秒/格
	TArray<FVoxelPathPoint> PathA;
	PathA.Emplace(FIntVector(0, 0, 1));
	PathA.Emplace(FIntVector(1, 0, 1));
	float TravelA = 0.f;
	TestTrue(TEXT("A 预约登记成功"), Terrain->CommitPathReservations(AgentA, PathA, 1, 1.f, 0.f, TravelA));
	TestEqual(TEXT("A 全程耗时"), TravelA, 1.f, 0.001f);

	// 窗口查询：(1,0) 在 [1,2) 被占；忽略 A 后不占
	TestTrue(TEXT("目标格窗口内被预约"), Terrain->IsCoordReservedAt({ 1, 0, 1 }, 1.5f, 1.6f, AgentB));
	TestFalse(TEXT("忽略预约者本人"), Terrain->IsCoordReservedAt({ 1, 0, 1 }, 1.5f, 1.6f, AgentA));
	TestFalse(TEXT("窗口外不冲突"), Terrain->IsCoordReservedAt({ 1, 0, 1 }, 2.5f, 3.f, AgentB));

	// B 在 (1,0) 要走到 (0,0)：桥只有两格，绕无可绕，且对穿被预约判定拦下 → 时空寻路给不出路
	float TravelB = 0.f;
	const TArray<FVoxelPathPoint> PathB = Terrain->FindPathScheduled({ 1, 0, 1 }, { 0, 0, 1 }, 1, 1, AgentB, 1.f, 0.f, TravelB);
	TestTrue(TEXT("对穿被时空寻路消解（无路）"), PathB.IsEmpty());

	// 原子性：B 试图登记与 A 重叠的预约 → 拒绝且不写入半条
	TArray<FVoxelPathPoint> PathB2;
	PathB2.Emplace(FIntVector(1, 0, 1));
	PathB2.Emplace(FIntVector(0, 0, 1));
	float TravelB2 = 0.f;
	TestFalse(TEXT("重叠预约整条拒绝"), Terrain->CommitPathReservations(AgentB, PathB2, 1, 1.f, 0.f, TravelB2));
	TestFalse(TEXT("B 没留下任何半截登记"), Terrain->IsCoordReservedAt({ 1, 0, 1 }, 1.2f, 1.3f, AgentA));

	// 释放后 B 的时空寻路恢复可行
	Terrain->RemoveReservationsFor(AgentA);
	const TArray<FVoxelPathPoint> PathB3 = Terrain->FindPathScheduled({ 1, 0, 1 }, { 0, 0, 1 }, 1, 1, AgentB, 1.f, 0.f, TravelB);
	TestTrue(TEXT("释放后 B 可达"), PathB3.Num() == 2);
	return true;
}

/* ===================== 先占后走 / 让行 / 对向死锁 ===================== */

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVoxelNavClaimBeforeMoveTest, "VoxelTerrain.Nav.ClaimBeforeMove", VOXEL_NAV_TEST())

bool FVoxelNavClaimBeforeMoveTest::RunTest(const FString& Parameters)
{
	using namespace VoxelNavTest;
	FTestWorld TW;
	if (!TW.Create(TEXT("VoxelNavClaim"), *this))
	{
		return false;
	}
	AVoxelTerrainActor* Terrain = SpawnFlatTerrain(TW.World, *this, { 1 });
	if (!Terrain)
	{
		return false;
	}
	// 1 格宽走廊：y=0 一行（其余列封死落脚层）
	for (int32 x = -8; x <= 7; ++x)
	{
		for (int32 y = -8; y <= 7; ++y)
		{
			if (y != 0)
			{
				Terrain->SetVoxel({ x, y, 1 }, TEXT("Rock"));
			}
		}
	}
	Terrain->BuildNavData();

	UVoxelPathFollowingComponent* CompA = nullptr;
	UVoxelPathFollowingComponent* CompB = nullptr;
	AActor* A = SpawnAgentAt(TW.World, Terrain, { -5, 0, 1 }, 1, &CompA);
	AActor* B = SpawnAgentAt(TW.World, Terrain, { 5, 0, 1 }, 1, &CompB);
	if (!A || !B || !CompA || !CompB)
	{
		AddError(TEXT("Agent/组件生成失败"));
		return false;
	}
	CompA->bReservePath = false;	// 本用例只看硬占地层（预约在 Reservation 用例单独验证）
	CompB->bReservePath = false;

	const TArray<FVoxelPathPoint> PathA = Terrain->FindPath({ -5, 0, 1 }, { 5, 0, 1 }, 1, 1, A);
	const TArray<FVoxelPathPoint> PathB = Terrain->FindPath({ 5, 0, 1 }, { -5, 0, 1 }, 1, 1, B);
	TestTrue(TEXT("A 有路"), PathA.Num() > 1);
	TestTrue(TEXT("B 有路（对向路径在迈步前无法被空间避让发现）"), PathB.Num() > 1);
	TestTrue(TEXT("A 出发"), CompA->RequestMove(PathA));
	TestTrue(TEXT("B 出发"), CompB->RequestMove(PathB));

	// 逐帧推进：每步一格。校验不变式：同一格不同时的两人登记 + 同帧对穿（换位）绝不发生
	FIntVector PrevA = CompA->GetCurrentCoord();
	FIntVector PrevB = CompB->GetCurrentCoord();
	const float Step = 0.1f;
	int32 Iter = 0;
	for (; Iter < 2000; ++Iter)
	{
		CompA->AdvanceFollowing(Step);
		CompB->AdvanceFollowing(Step);

		const FIntVector NowA = CompA->GetCurrentCoord();
		const FIntVector NowB = CompB->GetCurrentCoord();
		if (NowA == PrevB && NowB == PrevA && NowA != PrevA)
		{
			AddError(*FString::Printf(TEXT("第 %d 步发生了对穿换位：%s <-> %s"), Iter, *PrevA.ToString(), *PrevB.ToString()));
			break;
		}
		PrevA = NowA;
		PrevB = NowB;
		if (CompA->Status == EVoxelPathFollowingStatus::Idle && CompB->Status == EVoxelPathFollowingStatus::Idle)
		{
			break;
		}
	}
	TestTrue(TEXT("在预算步数内收敛"), Iter < 2000);

	// 两人最终都停在各自的落脚格里；让行/收尾状态机没有挂死
	TestFalse(TEXT("A 不再移动"), CompA->IsFollowingPath());
	TestFalse(TEXT("B 不再移动"), CompB->IsFollowingPath());
	// 单格走廊上对向而行：不可能都到达 —— 至少一人被 Blocked
	const bool bAnyBlocked = CompA->GetLastResult().Code == EVoxelPathFollowingResult::Blocked
		|| CompB->GetLastResult().Code == EVoxelPathFollowingResult::Blocked;
	TestTrue(TEXT("至少一人以 Blocked 收尾（对向死锁被让行超时解开）"), bAnyBlocked);
	// 两人没挤在同一格
	TestTrue(TEXT("终局不同格"), CompA->GetCurrentCoord() != CompB->GetCurrentCoord());
	return true;
}

/* ===================== 宽体型占地与重寻路 ===================== */

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVoxelNavWideAgentTest, "VoxelTerrain.Nav.WideAgent", VOXEL_NAV_TEST())

bool FVoxelNavWideAgentTest::RunTest(const FString& Parameters)
{
	using namespace VoxelNavTest;
	FTestWorld TW;
	if (!TW.Create(TEXT("VoxelNavWide"), *this))
	{
		return false;
	}
	AVoxelTerrainActor* Terrain = SpawnFlatTerrain(TW.World, *this, { 1, 2 });
	if (!Terrain)
	{
		return false;
	}
	// 2 格宽走廊（y∈{0,1}），两端寻路点用 anchor 口径：宽 2 的 anchor 在 y=1（覆盖 y∈{0,1}）
	for (int32 x = -8; x <= 7; ++x)
	{
		for (int32 y = -8; y <= 7; ++y)
		{
			if (y != 0 && y != 1)
			{
				Terrain->SetVoxel({ x, y, 1 }, TEXT("Rock"));
			}
		}
	}
	// 走廊只有一格宽的歪脖：x=0 处 y=0 封掉 → 宽 2 在 x=1 之前就没有合法 anchor（覆盖不到 y=0 列）
	Terrain->SetVoxel({ 0, 0, 1 }, TEXT("Rock"));
	Terrain->BuildNavData();

	const TArray<FVoxelPathPoint> PathW2 = Terrain->FindPath({ -4, 1, 1 }, { 4, 1, 1 }, 1, 2);
	TestTrue(TEXT("歪脖挡住宽2"), PathW2.IsEmpty());
	const TArray<FVoxelPathPoint> PathW1 = Terrain->FindPath({ -4, 1, 1 }, { 4, 1, 1 }, 1, 1);
	TestTrue(TEXT("宽1 可绕（走廊 y=1 一整行通）"), PathW1.Num() > 1);

	// footprint 占地：宽 2 的 AI 站 anchor (2,1) → 占 (1,0)(2,0)(1,1)(2,1) 四格，宽 1 寻路被这堵“人墙”拦停
	AActor* Big = SpawnAgentAt(TW.World, Terrain, { 2, 1, 1 }, 2, nullptr);
	AActor* Small = SpawnAgentAt(TW.World, Terrain, { -4, 1, 1 }, 1, nullptr);
	if (!Big || !Small)
	{
		AddError(TEXT("Agent 生成失败"));
		return false;
	}
	TestTrue(TEXT("宽2 整块占位"), Terrain->TryOccupyFootprint({ 2, 1, 1 }, 2, Big));
	TestFalse(TEXT("宽1 不能再占已被覆盖的 (1,1)"), Terrain->TryOccupyCoord({ 1, 1, 1 }, Small));

	// y=0 被封、y=1 被占两格：小 AI 无路可绕
	const TArray<FVoxelPathPoint> Blocked = Terrain->FindPath({ -4, 1, 1 }, { 4, 1, 1 }, 1, 1, Small);
	TestTrue(TEXT("人墙拦停宽1寻路"), Blocked.IsEmpty());

	Terrain->ReleaseFootprint({ 2, 1, 1 }, 2, Big);
	const TArray<FVoxelPathPoint> Reopened = Terrain->FindPath({ -4, 1, 1 }, { 4, 1, 1 }, 1, 1, Small);
	TestTrue(TEXT("释放后恢复可通"), Reopened.Num() > 1);
	return true;
}

/* ===================== 自动重寻路（RequestMoveToGoal） ===================== */

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVoxelNavAutoRepathTest, "VoxelTerrain.Nav.AutoRepath", VOXEL_NAV_TEST())

bool FVoxelNavAutoRepathTest::RunTest(const FString& Parameters)
{
	using namespace VoxelNavTest;
	FTestWorld TW;
	if (!TW.Create(TEXT("VoxelNavRepath"), *this))
	{
		return false;
	}
	AVoxelTerrainActor* Terrain = SpawnFlatTerrain(TW.World, *this, { 1 });
	if (!Terrain)
	{
		return false;
	}
	// 「口」形双通路：中间一段单走廊被占后，绕外圈可达
	// 外圈通道：y=0 行 x∈[-4..4]；上圈 y=2 行 x∈[-4..4]；两端以 x=-4/4 的 y=1 相连；其余全封
	for (int32 x = -8; x <= 7; ++x)
	{
		for (int32 y = -8; y <= 7; ++y)
		{
			const bool bKeepRow = (y == 0 || y == 2) && x >= -4 && x <= 4;
			const bool bKeepCap = (x == -4 || x == 4) && (y == 1);
			if (!bKeepRow && !bKeepCap)
			{
				Terrain->SetVoxel({ x, y, 1 }, TEXT("Rock"));
			}
		}
	}
	Terrain->BuildNavData();

	UVoxelPathFollowingComponent* CompA = nullptr;
	UVoxelPathFollowingComponent* CompB = nullptr;
	AActor* A = SpawnAgentAt(TW.World, Terrain, { -4, 1, 1 }, 1, &CompA);		// 上路走 y=2
	AActor* B = SpawnAgentAt(TW.World, Terrain, { 4, 1, 1 }, 1, &CompB);		// 下路走 y=0
	if (!A || !B || !CompA || !CompB)
	{
		AddError(TEXT("Agent/组件生成失败"));
		return false;
	}
	CompA->bReservePath = false;
	CompB->bReservePath = false;
	CompA->MaxYieldTime = 0.5f;		// 快速触发重寻路
	CompB->MaxYieldTime = 0.5f;

	// 两人都从同一个隘口 (0,1,1)?「口」字里 y=1 只有两端 —— 双方路径无共享格，
	// 改造共享：让 A 先占住 y=2 中段一格，B 的初始路径走 y=2 被占 → B 重寻路绕 y=0
	// 具体：B 先手动占住 (0,2,1)（假装是 A 停在上面），再让 B 用 RequestMoveToGoal 规划：
	TestTrue(TEXT("路人占住上圈中段"), Terrain->TryOccupyCoord({ 0, 2, 1 }, A));

	TestTrue(TEXT("B 目标式请求出发"), CompB->RequestMoveToGoal({ -4, 1, 1 }, 1));
	TestTrue(TEXT("B 的初始路径避开占格改走下圈（自动选路）"),
		CompB->GetPathPoints().Num() > 1 && CompB->GetPathPoints()[1].Coord.Y == 0);

	int32 Iter = 0;
	for (; Iter < 400 && CompB->IsFollowingPath(); ++Iter)
	{
		CompB->AdvanceFollowing(0.1f);
	}
	TestTrue(TEXT("B 走完（预算内）"), Iter < 400);
	TestTrue(TEXT("B 到达"), CompB->GetLastResult().Code == EVoxelPathFollowingResult::Success);
	TestTrue(TEXT("B 停在目标格"), CompB->GetCurrentCoord() == FIntVector(-4, 1, 1));
	return true;
}

#endif // WITH_AUTOMATION_TESTS
