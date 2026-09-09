#include "VoxelChunk.h"
#include "VoxelTerrainActor.h"
#include "VoxelType.h"
#include "Engine/AssetManager.h"

using namespace Voxel;

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

	if (!UAssetManager::IsInitialized())
	{
		// 编辑器/专用服务里 AssetManager 可能还没起来，别在网格生成线程上硬取
		UE_LOG(LogTemp, Warning, TEXT("AssetManager 未初始化，无法解析 VoxelType“%s”，网格重建将自动重试"), *Type.ToString());
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
		UE_LOG(LogTemp, Warning, TEXT("VoxelType“%s”暂不是可加载的 PrimaryAsset（多为 AssetManager 初次扫描未完成，会自动重试；若一直如此，检查 AssetManager 的 PrimaryAssetTypesToScan 配置"), *Type.ToString());
		return nullptr;
	}

	Handle->WaitUntilComplete();
	return Handle->GetLoadedAsset<UVoxelType>();
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

FVoxelChunk::FVoxelChunk(FIntVector2 InChunkCoord, int32 InMinHeight, int32 InMaxHeight)
	: ChunkCoord(MoveTemp(InChunkCoord))
	, BaseSectionZ(InMinHeight / LENGTH)	// 调用方保证 MinHeight 是 LENGTH 的整数倍
{
	const int32 Height = InMaxHeight - InMinHeight;
	const int32 NumSections = Height / LENGTH;
	checkf(Height % LENGTH == 0, TEXT("Height must be a multiple of Voxel::LENGTH"));
	checkf(InMinHeight % LENGTH == 0, TEXT("MinHeight must be a multiple of Voxel::LENGTH to locate a section"));
	Sections.Reserve(NumSections);
	for (int32 Z = BaseSectionZ; Z < NumSections; ++Z)
	{
		Sections.Emplace(FIntVector{ ChunkCoord.X, ChunkCoord.Y, Z });
	}
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

int32 FVoxelChunk::GetSectionIndex(int32 SectionZ) const
{
	return SectionZ - BaseSectionZ;
}
