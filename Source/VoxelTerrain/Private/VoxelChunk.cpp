#include "VoxelChunk.h"
#include "VoxelTerrainActor.h"
#include "VoxelType.h"
#include "Engine/AssetManager.h"
#include "Math/RotationMatrix.h"
#include "ProceduralMeshComponent.h"

using namespace Voxel;

namespace
{

struct FCacheState
{
	constexpr FCacheState() = default;
	explicit FCacheState(uint64 Packed)
		: Type(reinterpret_cast<const UVoxelType*>(Packed & 0x01FFFFFFFFFFFFFFULL))// 低 57 位为指针地址
		, Rotation(static_cast<uint8>((Packed >> 57) & 0x3F))// 位 57~62 为 Rotation
	{}

	FCacheState(FVoxelState State)
		: Type(State.GetTypeInstance())
		, Rotation(Type ? Type->bOrientationSensitive ? State.Rotation : 0 : 0)
	{}

	const UVoxelType* Type = nullptr;
	uint8 Rotation = 0;

	uint64 Packed() const
	{
		// 确保用户态地址不超过 57 位（兼容 5 级页表）
		// Rotation 存储于位 57~62，位 63 保留为 0
		return (static_cast<uint64>(Rotation & 0x3F) << 57)
			| (reinterpret_cast<uint64>(Type) & 0x01FFFFFFFFFFFFFFULL);
	}
};

// 整数轴向量的叉积（FIntVector 没有提供 ^）
FIntVector CrossAxis(const FIntVector& A, const FIntVector& B)
{
	return FIntVector(
		A.Y * B.Z - A.Z * B.Y,
		A.Z * B.X - A.X * B.Z,
		A.X * B.Y - A.Y * B.X);
}

// 面方向的局部坐标系：体素层沿 Normal 递进（Plane），面内沿 U、V 两条正轴向展开（u 对应 w，v 对应 h）
// 面内轴一律取正轴向，索引即坐标，避免负轴向造成的索引/坐标换算歧义；绕序由 NeedsSwappedWinding 修正。
struct FFaceFrame
{
	FIntVector Normal;	// 面法线（带符号，指向体外）
	FIntVector U;		// 面内 U 轴（正单位向量），即矩形的 w 方向，也是切线空间的切线方向
	FIntVector V;		// 面内 V 轴（正单位向量），即矩形的 h 方向
};

const FFaceFrame& GetFaceFrame(int32 Direction)
{
	static const FFaceFrame FaceFrames[6] = {
		{ FIntVector(1, 0, 0), FIntVector(0, 1, 0), FIntVector(0, 0, 1) },		// +X
		{ FIntVector(-1, 0, 0), FIntVector(0, 1, 0), FIntVector(0, 0, 1) },		// -X
		{ FIntVector(0, 1, 0), FIntVector(1, 0, 0), FIntVector(0, 0, 1) },		// +Y
		{ FIntVector(0, -1, 0), FIntVector(1, 0, 0), FIntVector(0, 0, 1) },		// -Y
		{ FIntVector(0, 0, 1), FIntVector(1, 0, 0), FIntVector(0, 1, 0) },		// +Z
		{ FIntVector(0, 0, -1), FIntVector(1, 0, 0), FIntVector(0, 1, 0) } };	// -Z
	return FaceFrames[Direction];
}

// UE 是左手坐标系，但三角形正面按“从正面看为逆时针（CCW）”判定
//（见 StaticMeshOperations.cpp:45 的注释：“We have a left-handed coordinate system, but a counter-clockwise winding order”），
// 用叉积写出来就是要求 (V1-V0)^(V2-V0) 指向 -Normal。
// 面内 (U,V) 与法线同手性（U^V == Normal）时，0,1,2,3 的顺序会算出 +Normal，即朝内、被背面剔除，必须交换 1、3 号顶点。
bool NeedsSwappedWinding(const FFaceFrame& Frame)
{
	return CrossAxis(Frame.U, Frame.V) == Frame.Normal;
}

constexpr int32 VertexOrderForward[4] = { 0, 1, 2, 3 };
constexpr int32 VertexOrderSwapped[4] = { 0, 3, 2, 1 };

// 面法线（带符号单位轴）-> GetFaceFrame 的方向编号，顺序同 FaceFrames 表：+X -X +Y -Y +Z -Z
int32 DirectionFromNormal(const FIntVector& Normal)
{
	if (Normal.X != 0) return Normal.X > 0 ? 0 : 1;
	if (Normal.Y != 0) return Normal.Y > 0 ? 2 : 3;
	return Normal.Z > 0 ? 4 : 5;
}

int32 DotAxis(const FIntVector& A, const FIntVector& B)
{
	return A.X * B.X + A.Y * B.Y + A.Z * B.Z;
}

// 24 向旋转就是一个 3x3 带符号置换矩阵，Rows[i] = 方块局部第 i 轴旋转后指向的世界轴向
struct FAxisPermutation
{
	FIntVector Rows[3] = { FIntVector(1, 0, 0), FIntVector(0, 1, 0), FIntVector(0, 0, 1) };

	FIntVector LocalToWorld(const FIntVector& Local) const
	{
		return Rows[0] * Local.X + Rows[1] * Local.Y + Rows[2] * Local.Z;
	}

	// 正交矩阵的逆变换就是转置：世界 -> 局部
	FIntVector WorldToLocal(const FIntVector& World) const
	{
		return FIntVector(DotAxis(World, Rows[0]), DotAxis(World, Rows[1]), DotAxis(World, Rows[2]));
	}
};

// 取整到最近的带符号单位轴（90° 倍数的旋转矩阵行必然落在坐标轴上，只剩浮点误差）
FIntVector SnapToAxis(const FVector& V)
{
	const double AbsX = FMath::Abs(V.X), AbsY = FMath::Abs(V.Y), AbsZ = FMath::Abs(V.Z);
	if (AbsX >= AbsY && AbsX >= AbsZ) return FIntVector(V.X < 0.0 ? -1 : 1, 0, 0);
	if (AbsY >= AbsZ) return FIntVector(0, V.Y < 0.0 ? -1 : 1, 0);
	return FIntVector(0, 0, V.Z < 0.0 ? -1 : 1);
}

// 由 FVoxelState::Rotation 的 6 位编码构造置换矩阵。
// 三个 90° 转的组合顺序直接交给引擎的 FRotationMatrix（Roll->X、Pitch->Y、Yaw->Z），
// 这样“方块的贴图朝向”和“把同一个方块当模型旋转”结果一致，90° 的方向符号不必手推。
FAxisPermutation MakeAxisPermutation(uint8 RotationBits)
{
	const FRotator Rotator{
		float((RotationBits >> 2) & 0x3) * 90.0f,	// Pitch
		float((RotationBits >> 4) & 0x3) * 90.0f,	// Yaw
		float(RotationBits & 0x3) * 90.0f };		// Roll

	const FMatrix Matrix = FRotationMatrix(Rotator);
	return {
		SnapToAxis(Matrix.TransformVector(FVector::UnitX())),
		SnapToAxis(Matrix.TransformVector(FVector::UnitY())),
		SnapToAxis(Matrix.TransformVector(FVector::UnitZ())) };
}

class FVoxelMeshHelper
{
public:

	FVoxelMeshHelper(TArray<FVoxelMeshData>& InData) : Data(InData) {}

	/**
	 * 提交一个贪心合并得到的矩形面（w×h 个体素共享同一 Mask）。
	 * @param Direction	面方向 0~5，见 GetFaceFrame
	 * @param Plane		该面所在体素层在法线轴上的编号（0~LENGTH-1）
	 * @param u, v		矩形在面内局部网格中的起点，u 沿 Face.U，v 沿 Face.V
	 * @param w, h		矩形在 u、v 方向上的体素数量
	 * 顶点是矩形 4 个角点相对于该 Section 原点（体素 (0,0,0) 的起始角点）的偏移 LocalOffset，乘以 VoxelSize 即为 Section 局部坐标。
	 * Mask 里解出的 Rotation 只改变贴图轴 TexU/TexV（UV 与切线朝向），不改变顶点位置——方块本来就是轴对齐的。
	 */
	void AddMeshData(uint64 Mask, const FVector& VoxelSize, int32 Direction, int32 Plane, int32 u, int32 v, int32 w, int32 h)
	{
		FVoxelMeshData* MeshData = FindOrAddMeshData(Mask);

		const FFaceFrame& Face = GetFaceFrame(Direction);
		const int32 FaceSign = Face.Normal.X + Face.Normal.Y + Face.Normal.Z;	// +1 正向面 / -1 负向面

		// 矩形贴在 Plane 层体素的外侧边界上：正向面取 Plane+1 这层角点，负向面取 Plane 这层角点。
		// Normal/U/V 两两垂直且均为正轴向（Normal 的符号只用于判断贴哪一侧），
		// 所以“轴向量 * 该轴上的坐标”即为 4 个角点相对 Section 原点的偏移 LocalOffset。
		const FIntVector Origin = Face.Normal * FaceSign * (FaceSign > 0 ? Plane + 1 : Plane);
		const FIntVector Corners[4] = {
			Origin + Face.U * u + Face.V * v,				// (u, v)
			Origin + Face.U * (u + w) + Face.V * v,			// (u+w, v)
			Origin + Face.U * (u + w) + Face.V * (v + h),	// (u+w, v+h)
			Origin + Face.U * u + Face.V * (v + h) };		// (u, v+h)

		// 旋转后的贴图轴：先用逆旋转把世界面法线换回局部，确定这面贴的是方块哪个局部面，
		// 再把该局部面的 (U,V) 旋回世界
		const FAxisPermutation Rotation = GetAxisRotation(FCacheState{ Mask }.Rotation);
		const FFaceFrame& LocalFace = GetFaceFrame(DirectionFromNormal(Rotation.WorldToLocal(Face.Normal)));
		const FIntVector TexU = Rotation.LocalToWorld(LocalFace.U);
		const FIntVector TexV = Rotation.LocalToWorld(LocalFace.V);

		// 顶点顺序只服务几何朝向（从外面看 CCW，正面朝外）；
		// UV 由角点相对起始角点的偏移投影到贴图轴上，所以 24 向旋转下 U/V 自动跟着转，不需要额外的分支表
		const int32* Order = NeedsSwappedWinding(Face) ? VertexOrderSwapped : VertexOrderForward;
		const FVector FaceNormal{ FVector(Face.Normal) };	// 轴向单位向量，无需归一化
		// 引擎只存 TangentX 和一个手性符号，运行时按 TangentY = (Normal ^ TangentX) * 符号 重建，
		// 符号要让它重建出的副切线正好等于贴图 V 轴，否则法线贴图的 G 通道在这些面上是镜像的
		const FProcMeshTangent FaceTangent{ FVector(TexU), CrossAxis(Face.Normal, TexU) != TexV };

		const int32 FirstVertex = MeshData->Vertices.Num();
		for (int32 i = 0; i < 4; ++i)
		{
			const FIntVector& Corner = Corners[Order[i]];
			const FIntVector Delta = Corner - Corners[0];
			MeshData->Vertices.Add(FVector(Corner) * VoxelSize);
			MeshData->Normals.Add(FaceNormal);
			MeshData->Tangents.Add(FaceTangent);
			MeshData->UVs.Add(FVector2D(DotAxis(Delta, TexU), DotAxis(Delta, TexV)));	// 1 个体素面 = 1 个 UV 单位
		}

		MeshData->Triangles.Append({
			FirstVertex + 0, FirstVertex + 1, FirstVertex + 2,
			FirstVertex + 0, FirstVertex + 2, FirstVertex + 3 });
	}

private:

	// Rotation 编码 -> 局部到世界的置换矩阵，最多 64 项（实际 24 项），省掉每个矩形重建一次 FRotationMatrix
	FAxisPermutation GetAxisRotation(uint8 RotationBits)
	{
		if (const FAxisPermutation* Cached = AxisRotCache.Find(RotationBits))
		{
			return *Cached;
		}
		return AxisRotCache.Add(RotationBits, MakeAxisPermutation(RotationBits));
	}

	FVoxelMeshData* FindOrAddMeshData(uint64 Mask)
	{
		// 存下标而不是指针：Data 是 TArray，Emplace_GetRef 会扩容，之前缓存的 FVoxelMeshData* 会全部失效
		// （一个 Section 里有第二种材质/朝向时，第一个就会写进已释放的内存）
		if (const int32* Found = MaskToIndex.Find(Mask))
		{
			return &Data[*Found];
		}

		FCacheState State{ Mask };
		FVoxelMeshData& NewData = Data.Emplace_GetRef();
		MaskToIndex.Emplace(Mask, Data.Num() - 1);
		NewData.bCollides = State.Type ? State.Type->bCollides : false;
		NewData.Material = State.Type ? State.Type->Material.LoadSynchronous() : nullptr;
		return &NewData;		// 本次调用内不会再扩容，引用有效；跨调用一律走下标
	}

	TMap<uint64, int32> MaskToIndex;
	TMap<uint8, FAxisPermutation> AxisRotCache;
	TArray<FVoxelMeshData>& Data;

};

}

FVoxelState::FVoxelState(FName InType, const FRotator& Rotator)
	: Type(InType)
{
	SetRotation(Rotator);
}

void FVoxelState::SetRotation(const FRotator& Rotator)
{
    // 辅助函数：根据角度获得对应的两位编码
    auto GetIndex = [](float Angle) -> uint8
        {
            if (Angle < 90.0f)  return 0b00;
            if (Angle < 180.0f) return 0b01;
            if (Angle < 270.0f) return 0b10;
            return 0b11;
        };

    // 获取各轴的编码
    const uint8 RollIndex = GetIndex(FRotator::ClampAxis(Rotator.Roll));
    const uint8 PitchIndex = GetIndex(FRotator::ClampAxis(Rotator.Pitch));
    const uint8 YawIndex = GetIndex(FRotator::ClampAxis(Rotator.Yaw));

    // 组装字节：位 0-1 : Roll 位 2-3 : Pitch 位 4-5 : Yaw 位 6-7 : 固定为 0
    Rotation |= (RollIndex & 0b11);
    Rotation |= (PitchIndex & 0b11) << 2;
    Rotation |= (YawIndex & 0b11) << 4;
}

const UVoxelType* FVoxelState::GetTypeInstance() const
{
	if (Type.IsNone())
	{
		return nullptr;
	}

	// 同一类型只告警一次：BuildMeshesIfNeeded 发现解析失败会周期性自动重建，重试会让本函数反复进来
	auto WarnOnce = [](const FName& InType, const FString& Message)
	{
		static TSet<FName> Warned;
		if (!Warned.Contains(InType))
		{
			Warned.Add(InType);
			UE_LOG(LogTemp, Warning, TEXT("%s"), *Message);
		}
	};

	if (!UAssetManager::IsInitialized())
	{
		// 编辑器/专用服务里 AssetManager 可能还没起来，别在网格生成线程上硬取
		WarnOnce(Type, FString::Printf(TEXT("AssetManager 未初始化，无法解析 VoxelType“%s”，网格重建将自动重试"), *Type.ToString()));
		Voxel::VoxelTypeResolveFailures()++;
		return nullptr;
	}

	UAssetManager& AssetManager = UAssetManager::Get();
	const FPrimaryAssetId AssetId{ UVoxelType::AssetType, Type };
	if (UVoxelType* TypeInstance = Cast<UVoxelType>(AssetManager.GetPrimaryAssetObject(AssetId)))
	{
		return TypeInstance;
	}

	TSharedPtr<FStreamableHandle> Handle = AssetManager.LoadPrimaryAsset(AssetId);
	if (!Handle.IsValid())
	{
		// 配置里注册过但查不到，最常见原因是 AssetManager 的首轮 PrimaryAsset 异步扫描还没完成（编辑器冷启动竞态）；
		// 真没配置的话，补 DefaultGame.ini 的 PrimaryAssetTypesToScan 即可
		WarnOnce(Type, FString::Printf(TEXT("VoxelType“%s”暂不是可加载的 PrimaryAsset（多为 AssetManager 初次扫描未完成，会自动重试；若一直如此，检查 AssetManager 的 PrimaryAssetTypesToScan 配置与 /Game/Voxel 目录）"), *Type.ToString()));
		Voxel::VoxelTypeResolveFailures()++;
		return nullptr;
	}

	Handle->WaitUntilComplete();
	const UVoxelType* Loaded = Cast<UVoxelType>(AssetManager.GetPrimaryAssetObject(AssetId));
	if (!Loaded)
	{
		WarnOnce(Type, FString::Printf(TEXT("VoxelType“%s”加载后仍解析失败，网格重建将自动重试"), *Type.ToString()));
		Voxel::VoxelTypeResolveFailures()++;
	}
	return Loaded;
}

FVoxelState FVoxelSection::GetVoxel(FIntVector Coord) const
{
    if (BitsPerVoxel == 0)
    {
        return SingleValue;
    }

	const uint32 Data = ReadBits(PackedData, ToIndex(Coord) * BitsPerVoxel, BitsPerVoxel);

	return Palettes[static_cast<int32>(Data)];
}

bool FVoxelSection::SetVoxel(FIntVector Coord, FVoxelState State)
{
	if (GetVoxel(Coord) == State)
	{
		return false;		// 内容没变，不用重建网格
	}

	if (BitsPerVoxel == 0)
	{
		// 转换为调色板模式
		Palettes.Reset();
		Palettes.Add(SingleValue);
		Palettes.Add(State);
		BitsPerVoxel = 1;
		PackedData.SetNumZeroed(PackedDataNeedSize());
	}

	const uint32 Index = FindOrAddPalenteIndex(State);
	WriteBits(PackedData, ToIndex(Coord) * BitsPerVoxel, BitsPerVoxel, Index);

	return true;
}

void FVoxelSection::Compact()
{
	if (BitsPerVoxel == 0) return;

	// 重新收集实际使用的状态
	TArray<int32> OldToNew;
	OldToNew.Init(INDEX_NONE, Palettes.Num());
	TArray<FVoxelState> NewPalettes;

	for (int32 i = 0; i < VOLUME; ++i)
	{
		const uint32 Idx = ReadBits(PackedData, i * BitsPerVoxel, BitsPerVoxel);
		if (OldToNew[Idx] == INDEX_NONE)
		{
			OldToNew[Idx] = NewPalettes.Num();
			NewPalettes.Add(Palettes[Idx]);
		}
	}

	if (NewPalettes.Num() == 1)
	{
		// 只有一个状态，转换为单值模式
		SingleValue = NewPalettes[0];
		BitsPerVoxel = 0;
		PackedData.Empty();
		Palettes.Empty();
	}
	else if (NewPalettes.Num() < Palettes.Num())
	{
		// 重新打包数据
		const int32 OldBitsPerVoxel = BitsPerVoxel;
		BitsPerVoxel = BitsForCount(NewPalettes.Num());
		TArray<uint32> NewPackedData;
		NewPackedData.SetNumZeroed(PackedDataNeedSize());
		for (int32 i = 0; i < VOLUME; ++i)
		{
			const uint32 OldIdx = ReadBits(PackedData, i * OldBitsPerVoxel, OldBitsPerVoxel);
			WriteBits(NewPackedData, i * BitsPerVoxel, BitsPerVoxel, OldToNew[OldIdx]);
		}
		PackedData = MoveTemp(NewPackedData);
	}
}

TArray<FVoxelMeshData> FVoxelSection::BuildMeshData(const AVoxelTerrainActor* Terrain, FIntVector SectionCoord) const
{
	TArray<FVoxelMeshData> Data;
	const bool bSingleValue = BitsPerVoxel == 0;
	if (bSingleValue && SingleValue.IsNone())
	{
		return Data;		// 全部为空，不需要生成网格
	}

	TArray<FCacheState> CachedStates;
	if (bSingleValue)
	{
		// 全部为同一类型（只检查 6 个边缘面，下面的 Plane 循环会跳过必然被剔除的内部层）
		const FCacheState State{ SingleValue };
		if (!State.Type)
		{
			// VoxelType 解析不出来就整块静默没有网格；GetTypeInstance 已按类型去重告警并触发自动重试，这里只留 Verbose 线索
			UE_LOG(LogTemp, Verbose, TEXT("[Voxel] Section (%d,%d,%d) 的类型“%s”解析失败，本 Section 不生成网格"),
				SectionCoord.X, SectionCoord.Y, SectionCoord.Z, *SingleValue.Type.ToString());
			return Data;
		}
		CachedStates.Init(State, VOLUME);
	}
	else
	{
		CachedStates.SetNumUninitialized(VOLUME);
		int32 Unresolved = 0;
		for (int32 i = 0; i < VOLUME; ++i)
		{
			const int32 PaletteIndex = ReadBits(PackedData, i * BitsPerVoxel, BitsPerVoxel);
			const FVoxelState& VoxelState = Palettes[PaletteIndex];
			CachedStates[i] = FCacheState{ VoxelState };
			if (VoxelState.Type != NAME_None && !CachedStates[i].Type)
			{
				++Unresolved;
			}
		}
		if (Unresolved > 0)
		{
			UE_LOG(LogTemp, Verbose, TEXT("[Voxel] Section (%d,%d,%d) 有 %d 个体素的 VoxelType 解析失败，这些方块不会出现在网格中"),
				SectionCoord.X, SectionCoord.Y, SectionCoord.Z, Unresolved);
		}
	}

	FVoxelMeshHelper MeshHelper{ Data };
	TStaticArray<uint64, SQUARE> Mask = { 0 };
	for (int32 i = 0; i < 6; ++i)	// 面方向
	{
		const FFaceFrame& Frame = GetFaceFrame(i);
		const int32 FaceSign = Frame.Normal.X + Frame.Normal.Y + Frame.Normal.Z;	// +1 正向面 / -1 负向面
		const FIntVector PlaneAxis = Frame.Normal * FaceSign;						// 法线轴的正方向单位向量

		// 单值模式下内部层的两侧类型相同、必然剔除，只遍历贴着她的那一层
		const int32 FirstPlane = bSingleValue ? (FaceSign > 0 ? LENGTH - 1 : 0) : 0;
		const int32 LastPlane = bSingleValue ? FirstPlane + 1 : LENGTH;

		for (int32 j = FirstPlane; j < LastPlane; ++j)	// 沿法线轴的体素层
		{
			FMemory::Memzero(Mask.GetData(), Mask.Num() * sizeof(uint64));

			// 只有真正处于 Section 边界的层才需要跨 Section（±Z）或跨 Chunk（±X/±Y）查询相邻体素
			const FVoxelSection* TargetSection = this;
			if ((FaceSign > 0 && j == LENGTH - 1) || (FaceSign < 0 && j == 0))
			{
				if (Frame.Normal.Z != 0)
				{
					TargetSection = Terrain->GetChunkSection(SectionCoord + FIntVector{ 0, 0, Frame.Normal.Z });
				}
				else if (Frame.Normal.X != 0)
				{
					TargetSection = Terrain->GetChunkSection(SectionCoord + FIntVector{ Frame.Normal.X, 0, 0 });
				}
				else
				{
					TargetSection = Terrain->GetChunkSection(SectionCoord + FIntVector{ 0, Frame.Normal.Y, 0 });
				}
			}

			for (int32 k = 0; k < SQUARE; ++k)	// 面内位置：k & 15 沿 U 轴，(k >> 4) & 15 沿 V 轴
			{
				const FIntVector Coord = PlaneAxis * j + Frame.U * (k & 15) + Frame.V * ((k >> 4) & 15);
				const int32 Index = ToIndex(Coord);
				const FCacheState State = CachedStates[Index];
				if (!State.Type) continue;

				const FIntVector OffsetCoord = GetOffsetCoord(Index, i);
				const UVoxelType* TargetType = TargetSection ?
					TargetSection == this ? CachedStates[ToIndex(OffsetCoord)].Type :
					TargetSection->GetVoxel(OffsetCoord).GetTypeInstance() :
					nullptr;

				if (!IsFaceVisible(State.Type, TargetType)) continue;

				Mask[k] = State.Packed();
			}

			for (int32 v = 0; v < LENGTH; ++v)	// 贪心合并：u 沿 Frame.U（宽 w），v 沿 Frame.V（高 h）
			{
				for (int32 u = 0; u < LENGTH; )
				{
					const uint64 CurrentMask = Mask[u + v * LENGTH];
					if (CurrentMask == 0)
					{
						++u;
						continue;
					}

					int32 w = 1;
					while (u + w < LENGTH && Mask[u + w + v * LENGTH] == CurrentMask)
					{
						++w;
					}

					int32 h = 1;
					bool bCanExtend = true;
					while (bCanExtend && v + h < LENGTH)
					{
						for (int32 x = 0; x < w; ++x)
						{
							if (Mask[u + x + (v + h) * LENGTH] != CurrentMask)
							{
								bCanExtend = false;
								break;
							}
						}
						if (bCanExtend)
						{
							++h;
						}
					}

					for (int32 x = 0; x < w; ++x)
					{
						for (int32 y = 0; y < h; ++y)
						{
							Mask[u + x + (v + y) * LENGTH] = 0;
						}
					}

					MeshHelper.AddMeshData(CurrentMask, Terrain->GetVoxelSize(), i, j, u, v, w, h);
					u += w;	// 本行已消费的宽度，否则死循环
				}
			}
		}
	}
	if (Data.IsEmpty() && !bSingleValue)
	{
		// 有调色板数据却没生成任何面：若本轮存在类型解析失败，多半是冷启动 AssetManager 还没扫完（会自动重试），降级 Verbose
		if (Voxel::VoxelTypeResolveFailures() > 0)
		{
			UE_LOG(LogTemp, Verbose, TEXT("[Voxel] Section (%d,%d,%d) 无输出：%s"),
				SectionCoord.X, SectionCoord.Y, SectionCoord.Z, *ToDebugString());
		}
		else
		{
			UE_LOG(LogTemp, Warning, TEXT("[Voxel] Section (%d,%d,%d) 无输出：%s"),
				SectionCoord.X, SectionCoord.Y, SectionCoord.Z, *ToDebugString());
		}
	}
	return Data;
}

FString FVoxelSection::ToDebugString() const
{
	int32 SolidPalettes = 0;
	for (const FVoxelState& State : Palettes)
	{
		if (!State.IsNone()) ++SolidPalettes;
	}

	const uint32* Words = PackedData.Num() >= 2 ? PackedData.GetData() : nullptr;
	FString PaletteDesc;
	for (int32 i = 0; i < Palettes.Num(); ++i)
	{
		PaletteDesc += FString::Printf(TEXT("%d:%s/%d "), i, *Palettes[i].Type.ToString(), Palettes[i].Rotation);
	}
	return FString::Printf(TEXT("bits=%d 调色板=[%s](实心%d) 位流=%d字 首两字={%u,%u} single=%s"),
		BitsPerVoxel, *PaletteDesc, SolidPalettes, PackedData.Num(),
		Words ? Words[0] : 0u, Words ? Words[1] : 0u,
		SingleValue.Type.IsNone() ? TEXT("None") : *SingleValue.Type.ToString());
}

int32 FVoxelSection::ToIndex(FIntVector Coord)
{
	return Coord.X + Coord.Y * LENGTH + Coord.Z * SQUARE;
}

int32 FVoxelSection::BitsForCount(int32 Count)
{
	int32 Bits = 1;
	while ((1 << Bits) < Count)
	{
		++Bits;
	}
	return Bits;
}

int32 FVoxelSection::PackedDataNeedSize() const
{
	return (VOLUME * BitsPerVoxel + 31) / 32;
}

uint32 FVoxelSection::ReadBits(const TArray<uint32>& Data, int32 BitOffset, int32 BitCount)
{
	if (BitCount <= 0 || BitCount > 32)
	{
		return 0;
	}

	const int32 Word = BitOffset >> 5;
	const int32 Shift = BitOffset & 31;

	uint32 Result = Data[Word] >> Shift;
	if (BitCount + Shift > 32 && Word + 1 < Data.Num())	// 跨越了两个 uint32
	{
		Result |= Data[Word + 1] << (32 - Shift);
	}
	if (BitCount < 32)		// 只保留低 BitCount 位
	{
		Result &= (1u << BitCount) - 1u;
	}
	return Result;
}

void FVoxelSection::WriteBits(TArray<uint32>& Data, int32 BitOffset, int32 BitCount, uint32 Value)
{
	if (BitCount == 0)
	{
		return;
	}
	const uint32 Mask = (BitCount == 32) ? 0xFFFFFFFFu : ((1u << BitCount) - 1u);
	Value &= Mask;

	const int32 Word = BitOffset >> 5;
	const int32 Shift = BitOffset & 31;

	// 低位部分写入当前字
	uint32& W0 = Data[Word];
	W0 = (W0 & ~(Mask << Shift)) | (Value << Shift);

	// 溢出部分写入下一个字（此分支内 Shift 必然 > 0，无 UB）
	if (BitCount + Shift > 32)
	{
		const int32 RemainderBits = BitCount + Shift - 32; // 必然 < 32
		const uint32 RemainderMask = (1u << RemainderBits) - 1u;
		uint32& W1 = Data[Word + 1];
		W1 = (W1 & ~RemainderMask) | (Value >> (32 - Shift));
	}
}

uint32 FVoxelSection::FindOrAddPalenteIndex(FVoxelState State)
{
	const int32 Index = Palettes.IndexOfByKey(State);
	if (Index != INDEX_NONE)
	{
		return static_cast<uint32>(Index);
	}

	Palettes.Emplace(State);

	const int32 RequiredBits = BitsForCount(Palettes.Num());
	if (RequiredBits > BitsPerVoxel)
	{
		// 需要更多的位数来表示调色板索引
		const int32 OldBitsPerVoxel = BitsPerVoxel;
		BitsPerVoxel = RequiredBits;
		TArray<uint32> NewPackedData;
		NewPackedData.SetNumZeroed(PackedDataNeedSize());

		for (int32 i = 0; i < VOLUME; ++i)
		{
			const uint32 OldValue = ReadBits(PackedData, i * OldBitsPerVoxel, OldBitsPerVoxel);
			WriteBits(NewPackedData, i * BitsPerVoxel, BitsPerVoxel, OldValue);
		}

		PackedData = MoveTemp(NewPackedData);
	}
	return static_cast<int32>(Palettes.Num() - 1);
}

bool FVoxelSection::IsFaceVisible(const UVoxelType* Source, const UVoxelType* Target)
{
	check(Source);

	if (!Target) return true;			// 没有相邻区块，显示面

	if (Source == Target)
	{
		return false;					// 相同类型，隐藏面
	}
	else
	{
		return Target->bTranslucent;	// 不同类型，如果相邻体素是透明的，显示面，不然隐藏面
	}
}

FIntVector FVoxelSection::GetOffsetCoord(int32 Source, int32 Direction)
{
	FIntVector Coord{ Source & 15, (Source >> 4) & 15, (Source >> 8) & 15 };
	Coord += GetFaceFrame(Direction).Normal;

	Coord.X = (Coord.X + 16) & 15;
	Coord.Y = (Coord.Y + 16) & 15;
	Coord.Z = (Coord.Z + 16) & 15;

	return Coord;
}

FVoxelChunk::FVoxelChunk(int32 InMinHeight, int32 InMaxHeight)
	: BaseSectionZ(InMinHeight / LENGTH)	// 调用方保证 MinHeight 是 LENGTH 的整数倍
{
	const int32 Height = InMaxHeight - InMinHeight;
	const int32 NumSections = Height / LENGTH;
	checkf(Height % LENGTH == 0, TEXT("Height must be a multiple of Voxel::LENGTH"));
	checkf(InMinHeight % LENGTH == 0, TEXT("MinHeight must be a multiple of Voxel::LENGTH to locate a section"));
	Sections.SetNum(NumSections);
	MeshComponents.SetNum(NumSections);
}

const FVoxelSection* FVoxelChunk::GetSection(int32 SectionZ) const
{
	return const_cast<FVoxelChunk*>(this)->GetSection(SectionZ);
}

FVoxelSection* FVoxelChunk::GetSection(int32 SectionZ)
{
	const int32 Index = GetSectionIndex(SectionZ);
	return Sections.IsValidIndex(Index) ? &Sections[Index] : nullptr;
}

void FVoxelChunk::BuildMesh(AVoxelTerrainActor* Terrain, FIntVector SectionCoord)
{
	const int32 Index = GetSectionIndex(SectionCoord.Z);
	if (!Sections.IsValidIndex(Index))
	{
		return;		// 该高度不在本 Chunk 的范围内
	}

	const TArray<FVoxelMeshData> MeshData = Sections[Index].BuildMeshData(Terrain, SectionCoord);
	if (MeshData.IsEmpty())
	{
		ClearMesh(SectionCoord.Z);
		return;
	}

	UProceduralMeshComponent* MeshComponent = FindOrAddChunkMeshComponent(Terrain, SectionCoord, Index);
	MeshComponent->ClearAllMeshSections();
	for (int32 i = 0; i < MeshData.Num(); ++i)
	{
		const FVoxelMeshData& Data = MeshData[i];

		MeshComponent->CreateMeshSection_LinearColor(i, Data.Vertices, Data.Triangles, Data.Normals,
			Data.UVs, TArray<FLinearColor>{}, Data.Tangents, Data.bCollides);
		MeshComponent->SetMaterial(i, Data.Material);
	}
}

void FVoxelChunk::BuildAllMeshes(AVoxelTerrainActor* Terrain, FIntVector2 ChunkCoord)
{
	for (int32 SectionZ = BaseSectionZ; SectionZ < BaseSectionZ + Sections.Num(); ++SectionZ)
	{
		BuildMesh(Terrain, FIntVector{ ChunkCoord.X, ChunkCoord.Y, SectionZ });
	}
}

void FVoxelChunk::ClearMesh(int32 CoordZ)
{
	const int32 Index = GetSectionIndex(CoordZ);
	if (MeshComponents.IsValidIndex(Index) && MeshComponents[Index])
	{
		MeshComponents[Index]->DestroyComponent();
		MeshComponents[Index] = nullptr;
	}
}

void FVoxelChunk::ClearAllMeshes()
{
	for (UProceduralMeshComponent*& MeshComponent : MeshComponents)
	{
		if (MeshComponent)
		{
			MeshComponent->DestroyComponent();
			MeshComponent = nullptr;
		}
	}
}

int32 FVoxelChunk::GetSectionIndex(int32 SectionZ) const
{
	return SectionZ - BaseSectionZ;
}

bool FVoxelChunk::SyncHeightRange(int32 InMinHeight, int32 InMaxHeight)
{
	const int32 ExpectedBase = InMinHeight / LENGTH;
	const int32 ExpectedNum = (InMaxHeight - InMinHeight) / LENGTH;
	if (BaseSectionZ != ExpectedBase || Sections.Num() != ExpectedNum)
	{
		// 高度范围改过了：Section 的数量和 Z 排布都和存档对不上，不能凑合用
		return false;
	}

	// 网格组件是运行时创建且 RF_Transient，不入档；这里只保证数组长度和 Sections 一致
	MeshComponents.SetNum(Sections.Num());
	return true;
}

UProceduralMeshComponent* FVoxelChunk::FindOrAddChunkMeshComponent(AVoxelTerrainActor* Terrain, FIntVector SectionCoord, int32 Index)
{
	// PIE 的对象复制不走 Transient 属性：副本里 MeshComponents 可能整个为空而 Sections 照旧，
	// 用之前先补齐到和 Sections 等长（只增不减，新槽位零初始化为 nullptr）
	if (MeshComponents.Num() < Sections.Num())
	{
		MeshComponents.SetNum(Sections.Num());
	}

	if (UProceduralMeshComponent* Existing = MeshComponents[Index])
	{
		// 这里存的是裸指针：组件被 Actor 销毁/重建（重跑 Construction Script、切关卡等）时会失效，
		// 校验不过就丢掉缓存重新建一个，别往已销毁的组件上写 mesh section。
		// 还要求属主与本世界一致：防止复制路径哪天带上编辑器世界的旧指针，把 PIE 的网格画进编辑器世界
		if (IsValid(Existing) && Existing->IsRegistered() && Existing->GetOwner() == Terrain && Existing->GetWorld() == Terrain->GetWorld())
		{
			return Existing;
		}
		MeshComponents[Index] = nullptr;
	}

	const FName ComponentName = MakeUniqueObjectName(Terrain, UProceduralMeshComponent::StaticClass(),
		*FString::Printf(TEXT("VoxelSection_%d_%d_%d"), SectionCoord.X, SectionCoord.Y, SectionCoord.Z));
	// RF_Transient：网格是派生数据，不写进关卡也不随对象复制走；GC 由 Chunks 里的 UPROPERTY 数组保住
	UProceduralMeshComponent* NewComponent = NewObject<UProceduralMeshComponent>(Terrain, ComponentName, RF_Transient);

	NewComponent->SetupAttachment(Terrain->GetRootComponent());
	NewComponent->SetRelativeLocation(FVector{ SectionCoord } * Terrain->GetVoxelSize() * LENGTH);
	NewComponent->RegisterComponent();

	MeshComponents[Index] = NewComponent;
	return NewComponent;
}
