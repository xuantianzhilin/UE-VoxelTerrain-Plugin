#include "VoxelGenerator.h"

// 声明是 const 的（BlueprintNativeEvent 会按原样生成 _Implementation），这里必须带上 const
void UVoxelGenerator::GenerateVoxel_Implementation(AVoxelTerrainActor* Terrain) const
{}
