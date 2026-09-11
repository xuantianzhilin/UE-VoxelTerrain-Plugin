#include "VoxelNavLinkProxy.h"
#include "VoxelTerrainActor.h"
#include "VoxelPathFollowingComponent.h"
#include "GameFramework/Actor.h"

void UVoxelNavLinkProxy::ReceiveLinkReached_Implementation(AActor* Agent, const AVoxelTerrainActor* Terrain, FIntVector Destination)
{
	Agent->SetActorLocation(Terrain->CoordToWorldLocation(Destination));
	ResumePathFollowing(Agent);
}

void UVoxelNavLinkProxy::ResumePathFollowing(AActor* Agent)
{
	if (Agent)
	{
		if (auto Follower = Agent->FindComponentByClass<UVoxelPathFollowingComponent>())
		{
			// To Do
		}
	}
}