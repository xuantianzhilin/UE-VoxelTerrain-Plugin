#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "VoxelGenerator.generated.h"

class AVoxelTerrainActor;

/**
 * 
 */
UCLASS(Abstract, Blueprintable)
class VOXELTERRAIN_API UVoxelGenerator : public UObject
{
	GENERATED_BODY()
	
public:
	
	UFUNCTION(BlueprintNativeEvent, Category = "Voxel")
	void GenerateVoxel(AVoxelTerrainActor* Terrain) const;
	
};
