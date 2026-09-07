// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class VoxelTerrain : ModuleRules
{
	public VoxelTerrain(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicIncludePaths.AddRange(
			new string[] {
				// ... add public include paths required here ...
			}
			);
				
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
			);
			
		
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"ProceduralMeshComponent",	// VoxelChunk.h 按值存 FProcMeshTangent，公开头文件需要它
				"Engine",					// 公开头文件暴露 UBlueprintAsyncActionBase / AAIController 等类型
				"AIModule",					// VoxelPathAI.h：行为树任务与 AI 移动
				"GameplayTasks",			// BT 任务基类的传递依赖，显式列出更稳
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject"
			}
			);
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
