#pragma once

#include "CoreMinimal.h"
#include "Containers/Ticker.h"		// FTSTicker / 句柄
#include "Engine/EngineTypes.h"		// EMovementMode（暂存角色的移动模式）
#include "VoxelNavLinkProxy.h"
#include "VoxelNavLinkJumpProxy.generated.h"

class UCharacterMovementComponent;
class UVoxelPathFollowingComponent;

/**
 * 跳跃弧线连接：UVoxelNavLinkProxy 的「会挪人」实现，把 Agent 沿抛物线跳到目标格。
 *
 * 用法：把它填到 AVoxelTerrainActor 的 DefaultLinkProxy（自动台阶连接），
 * 或塞进 AddLinkProxy(FVoxelNavLinkProxyData) 的 ProxyClass（手动连任意两点，
 * 配合 bOneWay 可以做单向跳台）。
 *
 * 两条飞行路线（ReceiveLinkReached 里自动选）：
 *  1) 真实弹道（Agent 是有控制器驱动的 ACharacter 且 bUseRealPhysicsTrajectory 开着）：
 *     把移动组件切成 MOVE_Falling、写入解出来的起跳速度（vz=√(2gH)，水平 Δxy/T，
 *     T 由「顶点比高的一端再抬 JumpHeight」反解 —— 轨迹正好落在目标格）。之后位移
 *     整个交给引擎：IsFalling() 为真、GetVelocity() 是真实抛物线速度、落地自动切回
 *     Walking 并广播 Landed —— 标准动画蓝图的跳跃/下落状态机不用改任何节点，
 *     途中还会真做碰撞扫描（不会穿墙）。代理只挂一条 FTSTicker 监护：发现不再
 *     Falling 就落位放行，超过兜底时限也照样落位。
 *  2) 运动学弧线（没有 CharacterMovement 的 Agent、或关掉上面开关/退化情形）：
 *     代理自己按 t 沿「起点 → 目标格站位」的抛物线 SetActorLocation，穿过中间几何体
 *     不做碰撞（想不蹭台阶把 JumpHeight 抬够），时长由 JumpDuration 决定；
 *     Agent 若有移动组件（如被关开关的角色），弧线的解析速度会每帧写进它的 Velocity，
 *     GetVelocity() 有真实读数；裸 Actor（无移动组件）没有速度可言，动画逻辑也无从挂起。
 *
 * 无论哪条路，最后一帧都会用 SnapToCell 按跟随组件的站位口径精确放好，再
 * ResumePathFollowing 放行 —— 「Actor 位置 ↔ 格坐标」始终一一对应。
 *
 * 想加跳跃音效 / 蒙太奇表现：继承它（或做个蓝图子类）重写 ReceiveJumpStarted；
 * 别在跳跃期间用 Root Motion 驱动 Actor，位置会和弧线驱动打架。
 */
UCLASS(Blueprintable, BlueprintType)
class VOXELTERRAIN_API UVoxelNavLinkJumpProxy : public UVoxelNavLinkProxy
{
	GENERATED_BODY()

public:

	/** 交接进来就开始跳；子类想加表现请优先重写 ReceiveJumpStarted，而不是把它整个替掉 */
	virtual void ReceiveLinkReached_Implementation(AActor* Agent, const AVoxelTerrainActor* Terrain, FIntVector Destination) override;

	/** 正在跳（逐帧监护/驱动挂在跑动中）。外部一般不用查 */
	UFUNCTION(BlueprintPure, Category = "Voxel|Navigation")
	bool IsJumping() const { return bJumping; }

	/** 这次是真实弹道（引擎驱动，IsFalling/GetVelocity/Landed 全真）还是运动学弧线（代理挪位置）。跳完/取消后为 false */
	UFUNCTION(BlueprintPure, Category = "Voxel|Navigation")
	bool IsUsingRealTrajectory() const { return bPhysicsJump; }

	/**
	 * 提前结束这次跳跃：立刻落到目标格并放行跟随组件（弹道模式连剩余空中过程一并跳过）。
	 * 跟随组件在 WaitingLink 期间拒绝 PauseMove（「要停得让代理停它自己的动作」），这里就是那个停。
	 * 没在跳时调用是空操作。
	 */
	UFUNCTION(BlueprintCallable, Category = "Voxel|Navigation")
	void InterruptJump();

	/* ===================== 配置 ===================== */

	/** 抛物线顶点比「起点/落点中较高一端」再抬高的厘米数。两条路线都用它定弧线高度；调太小会刮到中间台阶 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation", meta = (ClampMin = "0"))
	float JumpHeight = 80.f;

	/** 运动学路线的总时长（秒）。真实弹道路线不读它：角色的滞空由 JumpHeight 和重力决定（那样才像真跳） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation", meta = (ClampMin = "0.05"))
	float JumpDuration = 0.5f;

	/** Character 优先走引擎真实弹道（动画蓝图按 IsFalling/GetVelocity/Landed 就能播跳跃）；关掉则一律运动学 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation")
	bool bUseRealPhysicsTrajectory = true;

	/** 空中把朝向（只改 Yaw）平滑转向落点方向。角色由 AIController 管朝向时建议关掉，免得两边抢 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation")
	bool bFaceDestination = true;

	/** 转向角速度（度/秒），配 <=0 就是起跳瞬间硬转到位 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel|Navigation", meta = (EditCondition = "bFaceDestination", ClampMin = "0"))
	float TurnSpeed = 540.f;

	/** 起跳瞬间广播（默认空实现）。蓝图子类在这里挂跳跃音效、蒙太奇（非 Root Motion）、粒子等表现 */
	UFUNCTION(BlueprintNativeEvent, Category = "Voxel|Navigation")
	void ReceiveJumpStarted(AActor* Agent);

protected:

	/** 半路被 GC（移动被中止后代理会被地形丢出登记表）时兜底：撤掉 Ticker、恢复角色的移动模式 */
	virtual void BeginDestroy() override;

private:

	/** FTSTicker 回调：弹道模式做监护，运动学模式推进弧线。返回 false 表示本条 ticker 作废 */
	bool TickJump(float DeltaTime);

	/** 飞行结束：落位、恢复移动模式、放行跟随组件 */
	void CompleteJump();

	/** 选飞行路线：条件够就把角色挂成真实弹道（存下起跳前模式），否则交运动学（切 Flying 摘重力） */
	void BeginCharacterFlight(AActor* Agent);

	/** 把起跳前存下的模式恢复回去（幂等，没切过就是空操作） */
	void RestoreCharacterMovement();

	/** 按跳跃方向把 Yaw 平滑转向落点 */
	void FaceDestination(AActor* Agent, float DeltaTime) const;

	/* ---- 一次跳跃的运行期状态 ---- */

	UPROPERTY(Transient)
	TObjectPtr<UVoxelPathFollowingComponent> Follower;

	/** 交接给我们时那块地形（存着防悬挂，也保证弧线期间世界有效） */
	UPROPERTY(Transient)
	TObjectPtr<const AVoxelTerrainActor> TerrainRef;

	TWeakObjectPtr<AActor> JumpAgent;
	TWeakObjectPtr<UCharacterMovementComponent> CharMovement;

	/** 起跳前角色的 MovementMode / CustomMovementMode，落地后原样还回去 */
	TEnumAsByte<EMovementMode> SavedMovementMode = MOVE_None;
	uint8 SavedCustomMode = 0;

	FVector StartLocation = FVector::ZeroVector;
	/** 落点：目标格心 + 跟随组件口径的竖直站位偏移 */
	FVector DestinationLocation = FVector::ZeroVector;
	FIntVector DestinationCoord = FIntVector::ZeroValue;

	/** 真实弹道模式：水平起跳速度（每帧按它兜住刹车摩擦）与监护时限 */
	FVector LaunchXYVelocity = FVector::ZeroVector;
	bool bPhysicsJump = false;
	float PhysicsFlightLimit = 0.f;

	/** 注意：FTSTicker 的句柄是 TWeakPtr<FElement>，不是委托系统那个 FDelegateHandle */
	FTSTicker::FDelegateHandle TickerHandle;
	float Elapsed = 0.f;
	bool bJumping = false;
};
