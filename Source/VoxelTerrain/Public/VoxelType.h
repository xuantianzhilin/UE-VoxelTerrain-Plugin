#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "VoxelType.generated.h"


UCLASS(BlueprintType)
class UVoxelType : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VoxelType")
	FName TypeName;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VoxelType")
	TSoftObjectPtr<UMaterialInterface> Material;

	/** 是否开启碰撞 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VoxelType")
	bool bCollides = true;

	/* 是否透明 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VoxelType")
	bool bTranslucent = false;

	/**
	 * 旋转是否影响外观（材质槽解析与 UV 朝向）。
	 * 设为 false 的对称方块（纯色/六面同纹）会在贪心合并时忽略朝向，合并更充分。
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "VoxelType")
	bool bOrientationSensitive = true;

	inline static const FPrimaryAssetType AssetType{ TEXT("Voxel") };
	virtual FPrimaryAssetId GetPrimaryAssetId() const override;
};
