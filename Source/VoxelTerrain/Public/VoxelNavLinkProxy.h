#pragma once

#include "CoreMinimal.h"
#include "UObject/NoExportTypes.h"
#include "VoxelNavLinkProxy.generated.h"

class AVoxelTerrainActor;

USTRUCT(BlueprintType)
struct FVoxelNavLinkProxyData
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere)
	FIntVector StartCoord;
	UPROPERTY(EditAnywhere)
	FIntVector Destination;
	UPROPERTY(EditAnywhere)
	TSubclassOf<UVoxelNavLinkProxy> ProxyClass;

	bool operator==(const FVoxelNavLinkProxyData& Other) const
	{
		return StartCoord == Other.StartCoord && Destination == Other.Destination && ProxyClass == Other.ProxyClass;
	}
};

FORCEINLINE uint32 GetTypeHash(const FVoxelNavLinkProxyData& Key)
{
	return HashCombine(HashCombine(GetTypeHash(Key.StartCoord), GetTypeHash(Key.Destination)), GetTypeHash(Key.ProxyClass));
}

/**
 * 一条「非平面导航连接」的驱动者。
 *
 * 为什么要它：烘焙只把「同一层、水平相邻」的格子直接连起来，那种路 UVoxelPathFollowingComponent
 * 自己插值就能走。高差不同、或者根本不相邻的两处落脚点（跳台、爬梯、传送点、跨沟），
 * 位移方式没法用逐格插值表达，就由这里的一个对象负责把 Agent 挪过去。
 *
 * 来源有两种：
 *   - 自动：地形的 bAutoSpawNavLink 会把高差在 NavLinkMaxHeightDiff 内的相邻格挂上 NavLinkProxyClass；
 *   - 手动：AVoxelTerrainActor::AddLinkProxy(FVoxelNavLinkProxyData) 连任意两点。
 *
 * 想做自己的动作（跳跃弧线、爬梯动画、传送特效）就继承这个类，重写 ReceiveLinkReached，
 * 动作做完再调 ResumePathFollowing 放行；不需要挪人的话默认实现会直接把 Agent 对齐到目标格。
 */
UCLASS(Blueprintable, BlueprintType)
class VOXELTERRAIN_API UVoxelNavLinkProxy : public UObject
{
	GENERATED_BODY()
	
public:

	/** Agent 已站到本连接的起点：请你负责把它送到 Destination。默认什么都不做，等 ResumePathFollowing 放行 */
	UFUNCTION(BlueprintNativeEvent, Category = "Voxel|Navigation")
	void ReceiveLinkReached(AActor* Agent, const AVoxelTerrainActor* Terrain, FIntVector Destination);
	/** 非平面移动结束，放行 Agent 的路径跟随：把它对齐到目标格并继续走剩下的路 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	void ResumePathFollowing(AActor* Agent);

	/** 这条连接在寻路里的代价倍率，与「进入目标格的代价」相乘；<=0 表示这条连接已关闭（比如上了锁）*/
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation", meta = (ClampMin = "0"))
	float Weight = 1.f;
	/** 单向：只允许 StartCoord -> Destination，反向不生成连接。读的是类默认值，在烘焙时生效 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation")
	bool bOneWay = false;
	
};
