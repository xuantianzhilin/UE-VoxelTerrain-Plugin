// 路径跟随的诊断/回归测试：不依赖关卡与资产，纯代码搭地形。
//
// 命令行跑法（见文件末尾注释）：
//   UnrealEditor-Cmd.exe <项目>.uproject -ExecCmds="Automation RunTests VoxelTerrain.PathFollowing;Quit"
//       -Unattended -NoSound -NullRHI -Stdout -AllowStdOutLogVerbosity -NoLoadingScreen

#include "Misc/AutomationTest.h"

#include "Components/SceneComponent.h"
#include "Containers/Ticker.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"

#include "VoxelTerrainActor.h"
#include "VoxelPathFollowingComponent.h"
#include "VoxelNavLinkJumpProxy.h"

#if WITH_DEV_AUTOMATION_TESTS

namespace VoxelPathTest
{
	/** 任何非 None 的体素类型在地形看来都是实的，测试不需要真的资产 */
	const FName GroundType(TEXT("VoxelPathTestGround"));

	/** 离场景内容远远的地方搭测试地形：烘焙会把世界里的静态网格体栅格化成阻碍 */
	const FVector IsolatedOrigin(500000.0, 500000.0, 0.0);

	struct FActorCleanup
	{
		TArray<TWeakObjectPtr<AActor>> Actors;

		~FActorCleanup()
		{
			for (const TWeakObjectPtr<AActor>& Actor : Actors)
			{
				if (Actor.IsValid())
				{
					Actor->Destroy();
				}
			}
		}
	};

	static UWorld* GetEditorWorld()
	{
		if (!GEngine)
		{
			return nullptr;
		}
		for (const FWorldContext& Context : GEngine->GetWorldContexts())
		{
			if (Context.WorldType == EWorldType::Editor)
			{
				if (UWorld* World = Context.World())
				{
					return World;
				}
			}
		}
		return nullptr;
	}

	/** 裸 Actor 没有根组件，SpawnActor 给的位置无处安放，补一个 */
	static AActor* SpawnDummyActor(UWorld* World, FActorCleanup& Cleanup, FVector Location)
	{
		FActorSpawnParameters Params;
		Params.ObjectFlags |= RF_Transient;
		AActor* Actor = World->SpawnActor<AActor>(Location, FRotator::ZeroRotator, Params);
		Cleanup.Actors.Add(Actor);
		if (Actor && !Actor->GetRootComponent())
		{
			USceneComponent* Root = NewObject<USceneComponent>(Actor);
			Actor->SetRootComponent(Root);
			Root->RegisterComponentWithWorld(World);
			Actor->SetActorLocation(Location);
		}
		return Actor;
	}

	static AVoxelTerrainActor* SpawnPlatform(UWorld* World, FActorCleanup& Cleanup, int32 Extent, int32 GroundZ)
	{
		FActorSpawnParameters Params;
		Params.ObjectFlags |= RF_Transient;
		AVoxelTerrainActor* Terrain = World->SpawnActor<AVoxelTerrainActor>(IsolatedOrigin, FRotator::ZeroRotator, Params);
		Cleanup.Actors.Add(Terrain);
		if (Terrain)
		{
			Terrain->FillBox(FIntVector(0, 0, GroundZ), FIntVector(Extent, Extent, GroundZ), GroundType);
		}
		return Terrain;
	}
}

/* ============================================================================================ */

/**
 * 用户报的场景：地面铺在 z=0，AI 站在 (0,0,1)，要走到 (19,19,1)。
 * 这条路径必须跨过 Section 边界（16 格一层），而旧的测试全都在单个 Section 内部，从没覆盖过。
 */
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVoxelPathAcrossSectionsTest, "VoxelTerrain.PathFollowing.WalkAcrossSections",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FVoxelPathAcrossSectionsTest::RunTest(const FString& Parameters)
{
	using namespace VoxelPathTest;

	UWorld* World = GetEditorWorld();
	if (!TestNotNull(TEXT("需要编辑器世界"), World))
	{
		return true;
	}

	FActorCleanup Cleanup;
	AVoxelTerrainActor* Terrain = SpawnPlatform(World, Cleanup, 19, 0);		// 地面 z=0 -> 落脚格 z=1
	if (!TestNotNull(TEXT("地形应能生成"), Terrain))
	{
		return true;
	}
	Terrain->BuildNavData();

	const FIntVector Start(0, 0, 1);
	const FIntVector Goal(19, 19, 1);

	AddInfo(FString::Printf(TEXT("起点可站=%d 终点可站=%d"),
		Terrain->FindNavCell(Start) ? 1 : 0, Terrain->FindNavCell(Goal) ? 1 : 0));

	const TArray<FVoxelPathPoint> Path = Terrain->FindPath(Start, Goal, 1);
	AddInfo(FString::Printf(TEXT("路径点数=%d"), Path.Num()));
	FString PathText;
	for (int32 i = 0; i < Path.Num(); ++i)
	{
		if (i > 0)
		{
			const FIntVector D = Path[i].Coord - Path[i - 1].Coord;
			PathText += FString::Printf(TEXT(" %s(%s%s)"), *Path[i].Coord.ToString(),
				(D.X + D.Y) > 0 ? TEXT("+") : TEXT(""), *(D.X != 0 ? FString::FromInt(D.X) : FString::FromInt(D.Y)));
		}
		else
		{
			PathText += FString::Printf(TEXT(" %s"), *Path[i].Coord.ToString());
		}
	}
	AddInfo(FString::Printf(TEXT("路径=%s"), *PathText));

	if (!TestTrue(TEXT("应该能找到路"), Path.Num() > 0))
	{
		return true;
	}
	TestTrue(TEXT("路径起点是出发格"), Path[0].Coord == Start);
	TestTrue(TEXT("路径终点是目标格"), Path.Last().Coord == Goal);

	// 逐格走一遍：记录它实际踩过的每一格
	AActor* Agent = SpawnDummyActor(World, Cleanup, Terrain->CoordToWorldLocation(Start));
	UVoxelPathFollowingComponent* Follower = NewObject<UVoxelPathFollowingComponent>(Agent);
	Follower->RegisterComponent();
	Follower->SetTerrain(Terrain);
	Follower->Drive = EVoxelMoveDrive::DirectLocation;		// 裸 Actor 没有移动组件，这里强制走直线插值
	Follower->MoveSpeed = 300.f;

	TestTrue(TEXT("RequestMove 应当接受这条路径"), Follower->RequestMove(Path));
	TestTrue(TEXT("接受后进入跟随状态"), Follower->IsFollowingPath());

	FIntVector Last = Follower->GetCurrentCoord();
	TArray<FIntVector> Visited;
	FString Trail;
	int32 Steps = 0;
	while (Follower->IsFollowingPath() && ++Steps < 20000)
	{
		Follower->AdvanceFollowing(0.05f);
		const FIntVector Now = Follower->GetCurrentCoord();
		if (Now != Last)
		{
			Visited.Add(Now);
			Trail += FString::Printf(TEXT(" %s"), *Now.ToString());
			Last = Now;
		}
	}
	AddInfo(FString::Printf(TEXT("走位轨迹(%d 步, %d 格)=%s"), Steps, Visited.Num(), *Trail));

	const FVoxelPathFollowingResultInfo Result = Follower->GetLastResult();
	AddInfo(FString::Printf(TEXT("结果码=%d 结束格=%s 终点=%s PathPoints=%d"),
		static_cast<int32>(Result.Code), *Result.FinalCoord.ToString(), *Result.GoalCoord.ToString(), Result.PathPoints));

	TestEqual(TEXT("结果应当是到达"), static_cast<int32>(Result.Code), static_cast<int32>(EVoxelPathFollowingResult::Success));
	TestTrue(TEXT("最终应当站在目标格上"), Follower->GetCurrentCoord() == Goal);

	// 实际踩过的每一格都必须在路径上（没有走反、没有穿到别处）
	for (const FIntVector& Coord : Visited)
	{
		const bool bOnPath = Path.ContainsByPredicate([&Coord](const FVoxelPathPoint& P) { return P.Coord == Coord; });
		if (!bOnPath)
		{
			AddError(FString::Printf(TEXT("踩到了路径之外的格 %s"), *Coord.ToString()));
			break;
		}
	}
	return true;
}

/* ============================================================================================ */

/** 同一条路，但起点换到 Section 边界上（15->16 那一跳），单独钉一下跨 Section 的相邻关系 */
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVoxelPathSectionEdgeTest, "VoxelTerrain.PathFollowing.CrossSectionEdge",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FVoxelPathSectionEdgeTest::RunTest(const FString& Parameters)
{
	using namespace VoxelPathTest;

	UWorld* World = GetEditorWorld();
	if (!TestNotNull(TEXT("需要编辑器世界"), World))
	{
		return true;
	}

	FActorCleanup Cleanup;
	AVoxelTerrainActor* Terrain = SpawnPlatform(World, Cleanup, 19, 0);
	if (!TestNotNull(TEXT("地形应能生成"), Terrain))
	{
		return true;
	}
	Terrain->BuildNavData();

	// 只走一格：15 -> 16，正是 Section 边界
	const TArray<FVoxelPathPoint> Path = Terrain->FindPath(FIntVector(15, 8, 1), FIntVector(16, 8, 1), 1);
	AddInfo(FString::Printf(TEXT("跨边界路径点数=%d"), Path.Num()));
	TestEqual(TEXT("跨 Section 的一步应当有 2 个点"), Path.Num(), 2);
	if (Path.Num() == 2)
	{
		AddInfo(FString::Printf(TEXT("  起点=%s 终点=%s"), *Path[0].Coord.ToString(), *Path[1].Coord.ToString()));
		TestTrue(TEXT("起点是 (15,8,1)"), Path[0].Coord == FIntVector(15, 8, 1));
		TestTrue(TEXT("终点是 (16,8,1)"), Path[1].Coord == FIntVector(16, 8, 1));
	}
	return true;
}

/* ============================================================================================ */

/**
 * 跳跃连接的回归测试：UVoxelNavLinkJumpProxy 把「被一堵墙切断的两块地」连起来，
 * 并且真的让 Agent 沿抛物线翻过去。
 *
 * 墙（x=8 的一列，高 1 格）把地面切成两半，岛上没有别的通路 —— 能走到终点，
 * 且腾空期间 Z 高过墙顶，说明这条链路是通的：
 *   手动连接烘进导航图 -> A* 带 LinkClass 跨过 -> WaitingLink 交接 ->
 *   代理借 FTSTicker 逐帧驱动弧线 -> 落位放行 -> 走完剩下的路。
 */
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVoxelJumpNavLinkTest, "VoxelTerrain.PathFollowing.JumpLink",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FVoxelJumpNavLinkTest::RunTest(const FString& Parameters)
{
	using namespace VoxelPathTest;

	UWorld* World = GetEditorWorld();
	if (!TestNotNull(TEXT("需要编辑器世界"), World))
	{
		return true;
	}

	FActorCleanup Cleanup;
	AVoxelTerrainActor* Terrain = SpawnPlatform(World, Cleanup, 19, 0);		// 地面 z=0 -> 落脚格 z=1
	if (!TestNotNull(TEXT("地形应能生成"), Terrain))
	{
		return true;
	}
	Terrain->FillBox(FIntVector(8, 0, 1), FIntVector(8, 19, 1), GroundType);	// 一列墙横贯全场，切断两侧
	Terrain->BuildNavData();

	FVoxelNavLinkProxyData Link;
	Link.StartCoord = FIntVector(7, 12, 1);
	Link.Destination = FIntVector(9, 12, 1);
	Link.ProxyClass = UVoxelNavLinkJumpProxy::StaticClass();
	Terrain->AddLinkProxy(Link);

	const FIntVector Start(0, 0, 1);
	const FIntVector Goal(12, 12, 1);

	const TArray<FVoxelPathPoint> Path = Terrain->FindPath(Start, Goal, 1);
	AddInfo(FString::Printf(TEXT("路径点数=%d"), Path.Num()));
	if (!TestTrue(TEXT("应能经跳跃连接找到跨过墙的路"), Path.Num() > 0))
	{
		return true;
	}
	TestTrue(TEXT("路径终点是目标格"), Path.Last().Coord == Goal);

	int32 LinkPoints = 0;
	for (const FVoxelPathPoint& Point : Path)
	{
		if (Point.LinkClass)
		{
			++LinkPoints;
			TestEqual(TEXT("非平面跳挂的是跳跃代理"),
				Point.LinkClass.Get(), static_cast<UClass*>(UVoxelNavLinkJumpProxy::StaticClass()));
		}
	}
	TestEqual(TEXT("路径里只该有这一跳非平面连接"), LinkPoints, 1);

	// 逐格走一遍：腾空期间代理靠全局 ticker 驱动，测试里手动替引擎推它
	AActor* Agent = SpawnDummyActor(World, Cleanup, Terrain->CoordToWorldLocation(Start));
	UVoxelPathFollowingComponent* Follower = NewObject<UVoxelPathFollowingComponent>(Agent);
	Follower->RegisterComponent();
	Follower->SetTerrain(Terrain);
	Follower->Drive = EVoxelMoveDrive::DirectLocation;
	Follower->MoveSpeed = 400.f;

	TestTrue(TEXT("RequestMove 应当接受这条路径"), Follower->RequestMove(Path));

	const double WallTopZ = Terrain->CoordToWorldLocation(FIntVector(8, 12, 1)).Z + Terrain->GetVoxelSize().Z * 0.5;
	const double GoalCenterZ = Terrain->CoordToWorldLocation(Goal).Z;
	double MaxZ = TNumericLimits<double>::Lowest();
	int32 Steps = 0;
	while (Follower->IsFollowingPath() && ++Steps < 20000)
	{
		Follower->AdvanceFollowing(1.f / 60.f);
		FTSTicker::GetCoreTicker().Tick(1.f / 60.f);
		MaxZ = FMath::Max(MaxZ, Agent->GetActorLocation().Z);
	}

	const FVoxelPathFollowingResultInfo Result = Follower->GetLastResult();
	AddInfo(FString::Printf(TEXT("结果码=%d 结束格=%s 步数=%d 腾空最高 Z=%.1f（墙顶 %.1f）"),
		static_cast<int32>(Result.Code), *Result.FinalCoord.ToString(), Steps, MaxZ, WallTopZ));

	TestEqual(TEXT("结果应当是到达"), static_cast<int32>(Result.Code), static_cast<int32>(EVoxelPathFollowingResult::Success));
	TestTrue(TEXT("最终应当站在目标格上"), Follower->GetCurrentCoord() == Goal);
	TestTrue(TEXT("腾空期间跳过了墙顶（弧线驱动确实在动）"), MaxZ > WallTopZ);
	TestTrue(TEXT("落地后回到目标格的格心高度"), FMath::Abs(Agent->GetActorLocation().Z - GoalCenterZ) < 1.0);

	Agent->Destroy();
	return true;
}

/* ============================================================================================ */

/** StopMovement 打断一次正在空中的跳跃：代理要自己收场（撤 ticker、放行交给组件的收尾），移动按 Aborted 结束 */
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FVoxelJumpNavLinkAbortTest, "VoxelTerrain.PathFollowing.JumpLinkAbort",
	EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FVoxelJumpNavLinkAbortTest::RunTest(const FString& Parameters)
{
	using namespace VoxelPathTest;

	UWorld* World = GetEditorWorld();
	if (!TestNotNull(TEXT("需要编辑器世界"), World))
	{
		return true;
	}

	FActorCleanup Cleanup;
	AVoxelTerrainActor* Terrain = SpawnPlatform(World, Cleanup, 19, 0);
	if (!TestNotNull(TEXT("地形应能生成"), Terrain))
	{
		return true;
	}
	Terrain->FillBox(FIntVector(8, 0, 1), FIntVector(8, 19, 1), GroundType);
	Terrain->BuildNavData();

	FVoxelNavLinkProxyData Link;
	Link.StartCoord = FIntVector(7, 12, 1);
	Link.Destination = FIntVector(9, 12, 1);
	Link.ProxyClass = UVoxelNavLinkJumpProxy::StaticClass();
	Terrain->AddLinkProxy(Link);

	// 起点直接站在跳台上风：一步就是跳跃
	const FIntVector Start(7, 12, 1);
	const FIntVector Goal(9, 12, 1);
	const TArray<FVoxelPathPoint> Path = Terrain->FindPath(Start, Goal, 1);
	if (!TestTrue(TEXT("应该能找到这一跳"), Path.Num() == 2))
	{
		return true;
	}

	AActor* Agent = SpawnDummyActor(World, Cleanup, Terrain->CoordToWorldLocation(Start));
	UVoxelPathFollowingComponent* Follower = NewObject<UVoxelPathFollowingComponent>(Agent);
	Follower->RegisterComponent();
	Follower->SetTerrain(Terrain);
	Follower->Drive = EVoxelMoveDrive::DirectLocation;

	TestTrue(TEXT("RequestMove 应当接受"), Follower->RequestMove(Path));

	// 推进到交接发生、弧线跳起来（先离地再打断，确保打断的是「空中」）
	int32 Steps = 0;
	while (Steps++ < 120 && Agent->GetActorLocation().Z <= Terrain->CoordToWorldLocation(Start).Z + 1.0)
	{
		Follower->AdvanceFollowing(1.f / 60.f);
		FTSTicker::GetCoreTicker().Tick(1.f / 60.f);
	}
	const bool bAirborne = Agent->GetActorLocation().Z > Terrain->CoordToWorldLocation(Start).Z + 1.0;
	TestTrue(TEXT("打断前 Agent 应已被弧线带离地面"), bAirborne);

	Follower->StopMovement();
	const double AbortedZ = Agent->GetActorLocation().Z;
	for (int32 i = 0; i < 120; ++i)
	{
		FTSTicker::GetCoreTicker().Tick(1.f / 60.f);	// 残留的弧线 ticker 若没被撤，这里会把它抓出来
	}

	TestEqual(TEXT("中止后结果码应是 Aborted"),
		static_cast<int32>(Follower->GetLastResult().Code), static_cast<int32>(EVoxelPathFollowingResult::Aborted));
	TestEqual(TEXT("中止后 ticker 已撤，位置不再被弧线驱动"),
		Agent->GetActorLocation().Z, AbortedZ);

	Agent->Destroy();
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
