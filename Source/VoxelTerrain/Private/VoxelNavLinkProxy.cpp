#include "VoxelNavLinkProxy.h"
#include "VoxelTerrainActor.h"
#include "VoxelPathFollowingComponent.h"
#include "GameFramework/Actor.h"

void UVoxelNavLinkProxy::ReceiveLinkReached_Implementation(AActor* Agent, const AVoxelTerrainActor* Terrain, FIntVector Destination)
{
	if (!Agent || !Terrain)
	{
		return;
	}

	// 默认实现：直接把 Agent 对齐到目标格。交给跟随组件来放，是因为「站在哪」是它的事 ——
	// 它知道角色原点相对格心的竖直偏移（碰撞柱体中心 vs 格心），直接按 Terrain->CoordToWorldLocation
	// 放会把角色按到格心里、陷进地板
	if (UVoxelPathFollowingComponent* Follower = Agent->FindComponentByClass<UVoxelPathFollowingComponent>())
	{
		Follower->SnapToCell(Destination);
	}
	else
	{
		Agent->SetActorLocation(Terrain->CoordToWorldLocation(Destination));
	}

	ResumePathFollowing(Agent);
}

void UVoxelNavLinkProxy::ResumePathFollowing(AActor* Agent)
{
	if (!Agent)
	{
		return;
	}

	// 跟随组件挂在「被挪动的那个 Actor」上：放行就是通知它这一跳做完了，可以接着走剩下的路
	if (UVoxelPathFollowingComponent* Follower = Agent->FindComponentByClass<UVoxelPathFollowingComponent>())
	{
		Follower->ResumeFromLink();
	}
}