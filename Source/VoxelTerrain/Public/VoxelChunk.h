#pragma once

#include "CoreMinimal.h"
#include "ProceduralMeshComponent.h"
#include "VoxelChunk.generated.h"

class UVoxelType;
class AVoxelTerrainActor;

namespace Voxel
{
	constexpr int32 LENGTH = 16;
	constexpr int32 SQUARE = LENGTH * LENGTH;
	constexpr int32 VOLUME = LENGTH * LENGTH * LENGTH;

	/* VoxelType 解析失败计数：FVoxelState::GetTypeInstance 失败时 +1；
	   AVoxelTerrainActor 重建网格前清零、重建后检查，非 0 说明 AssetManager 初次扫描可能还没完成，
	   需要延迟自动重建（编辑器冷启动的竞态防护）。仅游戏线程访问。 */
	inline int32& VoxelTypeResolveFailures()
	{
		static int32 Failures = 0;
		return Failures;
	}
}

USTRUCT(BlueprintType)
struct FVoxelState
{
	GENERATED_BODY()

	FVoxelState() = default;
	FVoxelState(FName InType, const FRotator& Rotator);

	/** 必须是 UPROPERTY：FVoxelState 会被序列化进 FVoxelSection::Palettes，
	 *  没有 UPROPERTY 的字段不会写进包体，读回来就是 None（数组长度照常、内容全空） */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel")
	FName Type;
	/** 24向旋转 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel")
	uint8 Rotation = 0;

	bool IsNone() const { return Type.IsNone(); }
	void SetRotation(const FRotator& Rotator);
	const UVoxelType* GetTypeInstance() const;

	bool operator==(const FVoxelState& Other) const
	{
		return Type == Other.Type && Rotation == Other.Rotation;
	}
};

struct FVoxelMeshData
{
	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<FProcMeshTangent> Tangents;	// 切线基：TangentX 沿贴图 U 轴，bFlipTangentY 为 (TangentX, 贴图V轴, 法线) 的手性符号
	bool bCollides = false;
	UMaterialInterface* Material = nullptr;
};

/*方块数据的基本单位*/
USTRUCT()
struct FVoxelSection
{
public:

	GENERATED_BODY()

	FVoxelState GetVoxel(FIntVector Coord) const;
	bool SetVoxel(FIntVector Coord, FVoxelState State);
	void Compact();

	/**
	 * 贪心合并出本 Section 的网格数据。
	 * @param SectionCoord	本 Section 的坐标（Section 为单位，非体素坐标），用于向相邻 Section 查询遮挡
	 * @return	每个材质（Type + Rotation）一个 FVoxelMeshData，顶点均为 Section 局部坐标（原点在 Section 的 (0,0,0) 角点）
	 */
	TArray<FVoxelMeshData> BuildMeshData(const AVoxelTerrainActor* Terrain, FIntVector SectionCoord) const;

	/** 排查用：把本 Section 的存储特征拼成一行，方便对比保存前/载入后的差异 */
	FString ToDebugString() const;

private:

	static int32 ToIndex(FIntVector Coord);
	static int32 BitsForCount(int32 Count);
	static uint32 ReadBits(const TArray<uint32>& Data, int32 BitOffset, int32 BitCount);
	static void WriteBits(TArray<uint32>& Data, int32 BitOffset, int32 BitCount, uint32 Value);

	int32 PackedDataNeedSize() const;
	uint32 FindOrAddPalenteIndex(FVoxelState State);

	static bool IsFaceVisible(const UVoxelType* Source, const UVoxelType* Target);
	static FIntVector GetOffsetCoord(int32 Source, int32 Direction);

	UPROPERTY()
	int32 BitsPerVoxel = 0;			// 每个体素的位数，0表示单值模式，>0表示调色板模式
	UPROPERTY()
	FVoxelState SingleValue;		// 单值模式下的值
	UPROPERTY()
	TArray<FVoxelState> Palettes;	// 调色板
	UPROPERTY()
	TArray<uint32> PackedData;		// 位打包数据

};

USTRUCT()
struct FVoxelChunk
{
public:

	GENERATED_BODY()

	FVoxelChunk() = default;
	FVoxelChunk(int32 InMinHeight, int32 InMaxHeight);

	/** SectionZ 为全局 Section 索引（Chunk 只按 X/Y 划分，Z 由 MinHeight 换算），越界返回 nullptr */
	const FVoxelSection* GetSection(int32 SectionZ) const;
	FVoxelSection* GetSection(int32 SectionZ);

	/**
	 * 重建 SectionCoord 所在 Section 的网格（每个材质一个 mesh section），并挂到本 Chunk 的 ProceduralMeshComponent 上。
	 * @param SectionCoord	Section 坐标（Section 单位），X/Y 用于组件命名，Z 用于定位 Section
	 */
	void BuildMesh(AVoxelTerrainActor* Terrain, FIntVector SectionCoord);
	void BuildAllMeshes(AVoxelTerrainActor* Terrain, FIntVector2 ChunkCoord);

	void ClearMesh(int32 CoordZ);
	void ClearAllMeshes();

	/**
	 * 校验载入的存档与当前高度范围是否一致：BaseSectionZ 与 Section 数量都对得上才认为可用。
	 * 通过时会顺手把 MeshComponents 补齐到正确长度（存档里是 null，网格组件是运行时创建的）。
	 */
	bool SyncHeightRange(int32 InMinHeight, int32 InMaxHeight);

private:

	// 全局 Section Z 索引 -> 本 Chunk 的 Sections 下标（MinHeight 必须是 LENGTH 的整数倍）
	int32 GetSectionIndex(int32 SectionZ) const;
	UProceduralMeshComponent* FindOrAddChunkMeshComponent(AVoxelTerrainActor* Terrain, FIntVector SectionCoord, int32 Index);

	UPROPERTY()
	int32 BaseSectionZ = 0;			// 本 Chunk 第一个 Section 的全局 Z 索引 = MinHeight / LENGTH（默认构造必须归零，否则 LogClass 会报未初始化）
	UPROPERTY()
	TArray<FVoxelSection> Sections;
	// 网格组件是体素数据的派生结果，不入档也不参与对象复制：载入后由 Actor 统一重建一次
	UPROPERTY(Transient)
	TArray<UProceduralMeshComponent*> MeshComponents;

};
