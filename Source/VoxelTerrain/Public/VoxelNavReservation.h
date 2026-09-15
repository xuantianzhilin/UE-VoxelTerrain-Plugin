#pragma once

#include "CoreMinimal.h"
#include "UObject/WeakObjectPtr.h"

class AActor;

/**
 * 时空预约表里的一条记录：某 Agent 预计在 [tEnter, tExit]（世界秒）这段时间占用所在的那一格。
 *
 * 定位是「软规划层」：给 FindPathScheduled 与路径登记用的，别人规划路线时会绕开重叠窗口；
 * 真正的正确性由逐格硬占地（AVoxelTerrainActor::TryOccupyFootprint，先占后走）保证。
 * 角色实际早到/晚到只会影响流畅度，不会造成重叠。
 *
 * From 记的是这一跳的出发格：两条窗口重叠、方向正好相反的预约（A 的 From→To 对上 B 的 To→From）
 * 就是「对穿」，规划期按冲突处理。宽体型（footprint）在登记时展开成覆盖的每一格。
 */
struct FNavReservation
{
	TWeakObjectPtr<AActor> Agent;
	float tEnter = 0.f;
	float tExit = 0.f;
	/** 这一跳从哪一格来（起点驻留记录与 From 相同） */
	FIntVector From = FIntVector::ZeroValue;
};
