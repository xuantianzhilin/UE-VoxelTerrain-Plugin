#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "VoxelNavLinkProxy.h"
#include "VoxelPathFollowingComponent.generated.h"

class UVoxelNavLinkProxy;

USTRUCT(BlueprintType)
struct FVoxelPathPoint
{
	GENERATED_BODY()

	FVoxelPathPoint() = default;
	FVoxelPathPoint(FIntVector InCoord) : Coord(MoveTemp(InCoord)) {}
	FVoxelPathPoint(FIntVector InCoord, TSubclassOf<UVoxelNavLinkProxy> Class)
		: Coord(MoveTemp(InCoord))
		, LinkClass(Class)
	{}

	UPROPERTY()
	FIntVector Coord = FIntVector::ZeroValue;
	UPROPERTY()
	TSubclassOf<UVoxelNavLinkProxy> LinkClass;

	bool operator==(const FVoxelPathPoint& Other) const
	{
		return Coord == Other.Coord && LinkClass == Other.LinkClass;
	}
};

FORCEINLINE uint32 GetTypeHash(const FVoxelPathPoint& Key)
{
	return HashCombine(GetTypeHash(Key.Coord), GetTypeHash(Key.LinkClass));
}

// To Do
UCLASS(ClassGroup = (Voxel), meta = (BlueprintSpawnableComponent))
class VOXELTERRAIN_API UVoxelPathFollowingComponent : public UActorComponent
{
	GENERATED_BODY()

public:

};
