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

/** 一条导航连接：从某个落脚格指向水平 4 邻里的另一个落脚格（邻格可能属于相邻 Section） */
struct FVoxelNavLink
{
	/** 目标格所属的 Section（Section 单位坐标） */
	FIntVector OwnnerSection;
	/** 目标格在该 Section 内的局部体素坐标（0 ~ Voxel::LENGTH-1） */
	FIntVector LinkCoord;
};

/** 一个落脚格：本格自己的净空 + 到水平 4 邻的连接 */
struct FVoxelNavCell
{
	/** 本格能通过的最高 AI（体素单位）：从本格起向上数连续的净空格数，与旧寻路的 AgentHeight 同义。
	 *  数到 AVoxelTerrainActor::GetMaxAllowHeight() 格就封顶，取到该值只表示“至少这么高” */
	int32 AllowHeight = 0;
	/** 走得通的邻格：只含水平 4 邻，且与本格高度差在 FVoxelSection::LinkHeight 内（净空由对方那条记录自己给） */
	TArray<FVoxelNavLink> Links;
};

/*方块数据的基本单位*/
USTRUCT()
struct FVoxelSection
{
public:

	GENERATED_BODY()

	FVoxelSection() = default;
	explicit FVoxelSection(FIntVector Coord) : SectionCoord(MoveTemp(Coord)) {}

	FVoxelState GetVoxel(FIntVector Coord) const;
	bool SetVoxel(FIntVector Coord, FVoxelState State);
	void Compact();

	/**
	 * 贪心合并出本 Section 的网格数据。
	 * @param SectionCoord	本 Section 的坐标（Section 为单位，非体素坐标），用于向相邻 Section 查询遮挡
	 * @return	每个材质（Type + Rotation）一个 FVoxelMeshData，顶点均为 Section 局部坐标（原点在 Section 的 (0,0,0) 角点）
	 */
	TArray<FVoxelMeshData> BuildMeshData(const AVoxelTerrainActor* Terrain) const;

	/**
	 * 烘焙本 Section 的导航数据到 NavData（每次调用整体重建本 Section 的部分）。
	 * 节点是"落脚格"：自身为空、下方有支撑（体素或被静态网格体等外部阻碍占据的格都算支撑）。
	 * 连接规则：只连水平 4 邻，且两格高度差在 LinkHeight 内；骨骼网格体不参与烘焙。
	 * Key = 落脚格在本 Section 内的局部坐标，Value 见 FVoxelNavCell；链接目标可能落在相邻 Section。
	 * 注意：外扩一圈只为判定边界格与邻格，本函数不会写入相邻 Section 的数据（那边烘焙自己的）。
	 */
	void BuildNavData(const AVoxelTerrainActor* Terrain);
	void ClearNavData() { NavData.Reset(); }
	const TMap<FIntVector, FVoxelNavCell>& GetNavData() const { return NavData; }

private:

	static int32 ToIndex(FIntVector Coord);
	static int32 BitsForCount(int32 Count);
	static uint32 ReadBits(const TArray<uint32>& Data, int32 BitOffset, int32 BitCount);
	static void WriteBits(TArray<uint32>& Data, int32 BitOffset, int32 BitCount, uint32 Value);

	int32 PackedDataNeedSize() const;
	uint32 FindOrAddPalenteIndex(FVoxelState State);

	UPROPERTY()
	FIntVector SectionCoord;
	UPROPERTY()
	int32 BitsPerVoxel = 0;			// 每个体素的位数，0表示单值模式，>0表示调色板模式
	UPROPERTY()
	FVoxelState SingleValue;		// 单值模式下的值
	UPROPERTY()
	TArray<FVoxelState> Palettes;	// 调色板
	UPROPERTY()
	TArray<uint32> PackedData;		// 位打包数据

	/* 导航数据：Key = 本 Section 内的落脚格局部坐标，Value = 该格的净空与到水平 4 邻的连接。
	   只由 BuildNavData 生成（未烘焙时为空），是派生数据，不参与序列化 */
	TMap<FIntVector, FVoxelNavCell> NavData;

	static constexpr int32 LinkHeight = 1;
};

USTRUCT()
struct FVoxelChunk
{
public:

	GENERATED_BODY()

	FVoxelChunk() = default;
	FVoxelChunk(FIntVector2 InChunkCoord, int32 InMinHeight, int32 InMaxHeight);

	/** SectionZ 为全局 Section 索引（Chunk 只按 X/Y 划分，Z 由 MinHeight 换算），越界返回 nullptr */
	const FVoxelSection* GetSection(int32 SectionZ) const;
	FVoxelSection* GetSection(int32 SectionZ);

	/**
	 * 重建 SectionCoord 所在 Section 的网格（每个材质一个 mesh section），并挂到本 Chunk 的 ProceduralMeshComponent 上。
	 * @param SectionCoord	Section 坐标（Section 单位），X/Y 用于组件命名，Z 用于定位 Section
	 */
	void BuildMesh(AVoxelTerrainActor* Terrain, int32 CoordZ);
	void BuildAllMeshes(AVoxelTerrainActor* Terrain);

	void ClearMesh(int32 CoordZ);
	void ClearAllMeshes();

	void BuildNavData(AVoxelTerrainActor* Terrain, int32 CoordZ);
	void BuildAllNavData(AVoxelTerrainActor* Terrain);
	void ClearAllNavData();

private:

	// 全局 Section Z 索引 -> 本 Chunk 的 Sections 下标（MinHeight 必须是 LENGTH 的整数倍）
	int32 GetSectionIndex(int32 SectionZ) const;
	UProceduralMeshComponent* FindOrAddChunkMeshComponent(AVoxelTerrainActor* Terrain, int32 Index);

	UPROPERTY()
	FIntVector2 ChunkCoord;
	UPROPERTY()
	int32 BaseSectionZ = 0;			// 本 Chunk 第一个 Section 的全局 Z 索引 = MinHeight / LENGTH（默认构造必须归零，否则 LogClass 会报未初始化）
	UPROPERTY()
	TArray<FVoxelSection> Sections;
	// 网格组件是体素数据的派生结果，不入档也不参与对象复制：载入后由 Actor 统一重建一次
	UPROPERTY(Transient)
	TArray<UProceduralMeshComponent*> MeshComponents;

};
