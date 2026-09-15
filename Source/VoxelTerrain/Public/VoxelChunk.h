#pragma once

#include "CoreMinimal.h"
#include "ProceduralMeshComponent.h"
#include "VoxelChunk.generated.h"

class UVoxelType;
class UVoxelNavLinkProxy;
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

/**
 * 一条导航连接：从某个落脚格指向另一个落脚格（目标可能在相邻 Section，手动连接甚至可以很远）。
 * 两种走法，ProxyClass 是唯一的分界：
 *   - ProxyClass 为空：平面连接，同一层且水平相邻，UVoxelPathFollowingComponent 自己插值走过去。
 *   - ProxyClass 非空：非平面连接（高差不同 / 不相邻），必须由这个代理把 Agent 挪到目标格。
 */
struct FVoxelNavLink
{
	/** 目标格所属的 Section（Section 单位坐标） */
	FIntVector OwnnerSection;
	/** 目标格在该 Section 内的局部体素坐标（0 ~ Voxel::LENGTH-1） */
	FIntVector LinkCoord;

	/** 非平面连接的驱动者；为空表示这是一条可以直接走的平面连接 */
	TSubclassOf<UVoxelNavLinkProxy> ProxyClass;
};

/**
 * 一个落脚格（属于某一个「体型档」）：footprint 净空 + 到水平 4 邻的连接。
 *
 * 导航图按体型分档烘焙（见 AVoxelTerrainActor::GetNavFootprintWidths）：档 W 的节点是
 * 「W×W 的方形 footprint 能以本格为 anchor 完整站下」的落脚位。footprint 覆盖规则见
 * AVoxelTerrainActor::FootprintOrigin：anchor 固定在格上，「Actor 位置 ↔ anchor 格」一一对应。
 * 4 邻移动扫过的位置并集恰为两端 footprint（格子图只做轴向迈步，没有斜步，拐角天然安全），
 * 所以「两端都是本档节点」就是平面连接对本档体型成立的充要判据，不需要别的横向检查。
 */
struct FVoxelNavCell
{
	/** 本格能通过的最高 AI（体素单位）。档 0（1×1）是从本格起向上数连续的净空格数、只看本格这一列；
	 *  宽档取 footprint 覆盖的各列净空的 min（封顶单调：min-of-cap == cap-of-min，查询期仍按 AgentHeight 过滤）。
	 *  数到 AVoxelTerrainActor::GetMaxAllowHeight() 格封顶，取到该值只表示“至少这么高”；0 = 该档站不下（不会有条目） */
	int32 AllowHeight = 0;
	/** 走得通的邻格：平面连接只含同层水平 4 邻；高差不同或不相邻的得靠代理连接（见 FVoxelNavLink） */
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
	 * 烘焙本 Section 的导航数据到 NavDataByTier（每次调用按地形的体型档表整体重建本 Section 的部分）。
	 * 节点是"落脚格"：档 0 要求自身为空、下方有支撑（体素或被静态网格体等外部阻碍占据的格都算支撑）；
	 * 宽档（W>1）要求以本格为 anchor 的 W×W footprint 覆盖的每一格都满足档 0 的落脚条件，
	 * AllowHeight 取覆盖列净空的 min（烘焙期滑窗算好，查询期不再做 footprint 展开）。
	 * 连接规则：平面连接只连同层水平 4 邻；高差在 NavLinkMaxHeightDiff 内的相邻格由 bAutoSpawNavLink 自动
	 * 挂上 NavLinkProxyClass；更远或跨层的连接由 AddLinkProxy 手动挂。骨骼网格体不参与烘焙。
	 * 烘出来的图按档分表：NavDataByTier[0] 恒为 1×1 档（与旧数据逐格一致），索引 i>0 对应
	 * AVoxelTerrainActor::GetNavFootprintWidths()[i]。链接目标可能落在相邻 Section。
	 * 注意：外扩一圈只为判定边界格与邻格（宽档按最大 footprint 放宽），本函数不会写入相邻 Section 的数据（那边烘焙自己的）。
	 * Key = 落脚格在本 Section 内的局部坐标，Value 见 FVoxelNavCell。
	 */
	void BuildNavData(const AVoxelTerrainActor* Terrain);
	void ClearNavData() { NavDataByTier.Reset(); }
	/** 档 0（1×1）的导航数据 */
	const TMap<FIntVector, FVoxelNavCell>& GetNavData() const { return GetNavTierData(0); }
	/** 指定体型档的导航数据；档不存在返回空表 */
	const TMap<FIntVector, FVoxelNavCell>& GetNavTierData(int32 Tier) const;
	int32 GetNavTierCount() const { return NavDataByTier.Num(); }

private:

	static int32 ToIndex(FIntVector Coord);
	static int32 BitsForCount(int32 Count);
	static uint32 ReadBits(const TArray<uint32>& Data, int32 BitOffset, int32 BitCount);
	static void WriteBits(TArray<uint32>& Data, int32 BitOffset, int32 BitCount, uint32 Value);

	int32 PackedDataNeedSize() const;
	uint32 FindOrAddPalenteIndex(FVoxelState State);

	UPROPERTY()
	FIntVector SectionCoord = FIntVector::ZeroValue;
	UPROPERTY()
	int32 BitsPerVoxel = 0;			// 每个体素的位数，0表示单值模式，>0表示调色板模式
	UPROPERTY()
	FVoxelState SingleValue;		// 单值模式下的值
	UPROPERTY()
	TArray<FVoxelState> Palettes;	// 调色板
	UPROPERTY()
	TArray<uint32> PackedData;		// 位打包数据

	/* 导航数据：按体型档分表（索引即档号，见 FVoxelNavCell 注释），每档一张
	   「本 Section 内落脚格局部坐标 -> FVoxelNavCell」的表。
	   只由 BuildNavData 生成（未烘焙时为空），是派生数据，不参与序列化 */
	TArray<TMap<FIntVector, FVoxelNavCell>> NavDataByTier;
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
	FIntVector2 ChunkCoord = FIntVector2::ZeroValue;
	UPROPERTY()
	int32 BaseSectionZ = 0;			// 本 Chunk 第一个 Section 的全局 Z 索引 = MinHeight / LENGTH（默认构造必须归零，否则 LogClass 会报未初始化）
	UPROPERTY()
	TArray<FVoxelSection> Sections;
	// 网格组件是体素数据的派生结果，不入档也不参与对象复制：载入后由 Actor 统一重建一次
	UPROPERTY(Transient)
	TArray<UProceduralMeshComponent*> MeshComponents;

};
