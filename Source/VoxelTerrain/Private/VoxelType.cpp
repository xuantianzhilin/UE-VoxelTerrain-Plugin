#include "VoxelType.h"

FPrimaryAssetId UVoxelType::GetPrimaryAssetId() const
{
	if (HasAnyFlags(RF_ClassDefaultObject))
	{
		return FPrimaryAssetId{};
	}

	return FPrimaryAssetId{ AssetType, TypeName };
}
