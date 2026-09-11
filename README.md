# VoxelTerrain

一个面向 UE5 的运行时体素地形插件，把「体素编辑、网格渲染、导航寻路」做成一套自洽的系统：

- 体素数据用**调色板 + 位打包**压缩存储，网格用**贪心合并**生成，内存与面数都省；
- 导航**完全不依赖 NavMesh**：把可站立格子和连接烘焙进体素数据，A* 直接在体素图上跑；
- 附带路径跟随组件与导航连接代理，面向「格子制 / 回合制」玩法（AI 沿格子逐格走，格与格之间可挂跳跃、爬梯等自定义位移）。

| 项目 | 说明 |
| --- | --- |
| 模块 | `VoxelTerrain`（Runtime，单模块） |
| 依赖插件 | ProceduralMeshComponent |
| 版本 | 0.1（实验性） |
| 作者 | xuanlin |

---

## 功能特性

### 体素编辑
- XY 方向无限延伸，Z 方向限制在 `[MinHeight, MaxHeight)`（需为 16 的倍数）。
- `SetVoxel` / `SetVoxels` / `GetVoxel` 逐点读写；`FillBox` / `FillSphere` 批量填充（球支持只留球壳，TypeName 传 `NAME_None` 即挖空）。
- 每个体素带 **24 向旋转**（Roll/Pitch/Yaw 各量化到 90°），旋转参与材质槽解析与 UV 朝向。
- 体素射线检测 `LineSingleTraceVoxel`：DDA 逐格步进，不依赖物理碰撞，返回命中格坐标、法线、命中点与体素状态。

### 网格渲染
- 每个 Section 贪心合并出最少面数，按「体素类型 + 朝向」分材质槽，挂在 ProceduralMeshComponent 上。
- 运行时改体素走脏区增量重建，按预算分帧消化（默认每帧最多 5 个 Section），不卡帧。
- 体素类型 `UVoxelType`（PrimaryDataAsset）决定材质、是否碰撞、是否透明、是否朝向敏感。

### 生成器
- `UVoxelGenerator` 蓝图可扩展基类，覆写 `GenerateVoxel(Terrain)` 即可用任意规则铺体素。
- 地形 Actor 细节面板提供三个编辑器按钮：**生成默认地形 / 清除地形 / 重建地形网格**。
- 示例资产在 `/Game/Voxel`：`BP_TestGenerator`（示例生成器）、`DA_TestVoxelType`（示例体素类型）。

### 导航与寻路（自研，不用 NavMesh）
- 全量烘焙 + 脏区局部重建的导航数据：落脚格、竖直净空、平面连接、台阶连接。
- 世界里的静态网格体会被光栅化成导航阻碍（ISM/HISM 按单实例处理），桥、屋檐、悬空平台都能挡路。
- 区域通行权重：按格刷「难走程度」，`0` 是软墙（能站但 A* 不走），无需重烘。
- A* 寻路 `FindPath`，二叉堆 + 惰性删除，带展开上限与占地避让。
- 导航连接代理 `UVoxelNavLinkProxy`：台阶自动连接、手动连接任意两点（跳台/爬梯/传送/跨沟）；内置 `UVoxelNavLinkJumpProxy` 抛物线跳跃（角色可走引擎真实弹道，动画蓝图零改动）。
- 路径跟随组件 `UVoxelPathFollowingComponent`：逐格行走、暂停/继续、占地预约与避让、卡住保护。
- AI 占地表：`TryOccupyCoord` / `TryOccupyLink` / `FindFreeNearbyCoord`，多 AI 互不踩脚。

---

## 安装

1. 把 `VoxelTerrain` 文件夹复制到项目的 `Plugins/` 目录下。
2. 重新生成项目并启动编辑器，在 **Edit → Plugins** 里启用 VoxelTerrain（ProceduralMeshComponent 会随之自动启用）。
3. **注册体素类型资产扫描**（关键步骤，不做网格重建会一直告警重试）：
   **Project Settings → Asset Manager → Primary Asset Types to Scan** 添加一条：
   - Primary Asset Type：`Voxel`
   - Asset Base Class：`VoxelType`（即 `UVoxelType`）
   - Directories：`/Game/Voxel`（或你存放体素类型资产的目录）

   本仓库项目已在 `Config/DefaultGame.ini` 里配好，可直接参考。

---

## 快速上手

### 1. 摆一个地形
在关卡里放一个 **VoxelTerrainActor**。三个基础参数在 Details 的 Voxel 分类下：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| Voxel Size | 100 | 单个体素的边长（cm）。**有地形数据后锁定**，改要先「清除地形」。 |
| Min Height / Max Height | -16 / 64 | 有效高度范围（体素坐标），**必须都是 16 的倍数**，同样有数据后锁定。 |

### 2. 建体素类型
在 `/Game/Voxel` 下创建 **VoxelType** 资产（继承 `UVoxelType`），填：
- **Type Name**：体素的逻辑名（蓝图调 `SetVoxel` 时传的就是它），需全项目唯一；
- **Material**：该类体素使用的材质；
- **bCollides**：生成的网格是否带碰撞；
- **bTranslucent**：透明体素（相邻体素不会因为它而隐藏面，玻璃/水面用）；
- **bOrientationSensitive**：外观是否随旋转变化。纯色/六面同纹的方块关掉它，贪心合并可以忽略朝向，合并更充分。

### 3. 写一个生成器
创建蓝图继承 **VoxelGenerator**，覆写 `GenerateVoxel`，用编辑 API 铺地形，例如在蓝图中调用 `FillBox` 铺一层地面、`FillSphere` 挖个坑。然后把生成器蓝图指到 VoxelTerrainActor 的 **Voxel → Generator → VoxelGenerator** 槽上。

### 4. 生成与保存
点地形 Actor 细节面板的 **生成默认地形** 按钮，网格立即重建并可撤销（Ctrl+Z）。保存关卡即可——体素数据随关卡序列化入档，网格组件不入档。

勾选 **bRunGeneratorOnBeginPlay** 可以让运行时（PIE/打包）在 BeginPlay 时自动跑一遍生成器。

### 5. 运行时编辑（C++ / 蓝图）

```cpp
AVoxelTerrainActor* Terrain = ...;

// 单点放置 / 挖空（NAME_None 即挖空）
Terrain->SetVoxel({10, 10, 0}, "Stone");
Terrain->SetVoxel({10, 10, 1}, NAME_None);

// 批量：长方体与球（球 bSolid=false 只留 1 体素厚球壳）
Terrain->FillBox({0, 0, -1}, {30, 30, -1}, "Dirt");
Terrain->FillSphere(Center, 3.0, NAME_None);

// 体素射线检测（不需要物理碰撞）
FVoxelTraceHit Hit;
if (Terrain->LineSingleTraceVoxel(Start, End, Hit))
{
    // Hit.Coord 命中格、Hit.Normal 面法线、Hit.Voxel 命中的体素状态
}

// 查询
const FVoxelState State = Terrain->GetVoxel({10, 10, 0});
```

> ⚠ 编辑器世界里不跑 Tick：编辑器脚本里批量填充后想立刻看到网格，传 `RebuildCount > 0` 或再调一次 `BuildAllMeshes()`；运行时不用管，Tick 会分帧重建。

### 6. 坐标换算
体素坐标是**整数格坐标**，1 格 = VoxelSize，相对地形 Actor 的变换：

```cpp
FIntVector Coord = Terrain->WorldLocationToCoord(WorldLoc); // 世界坐标 → 格坐标
FVector   Loc   = Terrain->CoordToWorldLocation(Coord);     // 格坐标 → 格心世界坐标
FIntVector Section = AVoxelTerrainActor::GetSectionCoordFromWorldCoord(Coord); // 格 → Section 坐标
```

---

## 导航与寻路

### 烘焙模型
导航数据是**烘焙产物**，与体素数据一一对应：

- **落脚格**：自身为空、下方一格有支撑的格子。地表、洞穴地板、悬空平台的每一层都各自是节点。
- **净空 AllowHeight**：落脚格向上连续空格数（封顶 `Max Allow Height`）。A* 查询时按 AI 的 `AgentHeight` 过滤「站不下的格子」。
- **连接**：
  - 平面连接：同一层水平 4 邻，路径跟随组件自己插值走过去；
  - 自动连接：相邻格高差 ≤ `Nav Link Max Height Diff` 的台阶，自动挂上 `Default Link Proxy` 指定的代理类；
  - 手动连接：`AddLinkProxy(FVoxelNavLinkProxyData)` 把**任意两点**连起来（跳台、爬梯、传送点、跨沟），配合代理的 `bOneWay` 可做单向跳台。

**BeginPlay 自动全量烘焙一次**；运行期改体素只局部重建受影响的 Section。

### 区域通行权重
```cpp
Terrain->SetCoordNavWeight(Coord, 3.f);   // 这格更难走（代价 ×3）
Terrain->SetCoordNavWeightBox(Min, Max, 0.f);  // 一片软墙：能站，但 A* 自动绕开
Terrain->ClearCoordNavWeight(Coord);
```
权重是**查询期数据**，刷它不需要重烘导航。`1` 正常、`>1` 更难走、`0` 软墙；负数与 NaN 会被拒收。

### A* 寻路
```cpp
TArray<FVoxelPathPoint> Path = Terrain->FindPath(StartCoord, EndCoord, AgentHeight);
```
- 两端必须是落脚格且净空 ≥ `AgentHeight`，否则返回空数组（不是「绕不过去」，是「站不住」，调用方应先吸附）；
- 终点或途经格被别的 AI 占着时，A* 会绕开（占地是查询期判定的，见下文 AI 占地）。

### 路径跟随
给会动的 Actor 挂一个 **VoxelPathFollowingComponent**（组件不寻路，只负责把算好的路径走完）：

```cpp
// 标准用法：先算路径，再交给组件
FIntVector Start = Terrain->WorldLocationToCoord(Actor->GetActorLocation() - Comp->LocationOffset);
TArray<FVoxelPathPoint> Path = Terrain->FindPath(Start, Goal, AgentHeight);
Comp->RequestMove(Path);   // 失败会立刻广播 OnMoveFinished，不会动 Actor
```

- **驱动方式**（Drive）：`Auto` 默认——有移动组件（CharacterMovement 等）就下发速度由它走，没有就退回直接插值 `SetActorLocation`（落点精确吸附格心）。浮动 Pawn（`UFloatingPawnMovement`）必须打开移动组件的 **bUseAccelerationForPaths**，否则速度请求会被慢慢抹掉。
- **暂停/继续**：`PauseMove` / `ResumeMove`，保留路径与进度；正在过连接（跳跃中）时不允许暂停。
- **结果**：`OnMoveFinished` 委托广播一次 `FVoxelPathFollowingResultInfo`（到达 / 路径不可用 / 没找到地形 / 被占或卡住 / 被中止）。
- **卡住保护**：`Max Stall Time` 内没能更靠近目标点就按 Blocked 收尾。

### 自定义连接代理（跳跃、爬梯、传送…）
连接要「跨层」或「不相邻」时，位移无法逐格插值表达，就交给代理类把 Agent 挪过去。继承 `UVoxelNavLinkProxy` 重写 `ReceiveLinkReached`，动作做完调 `ResumePathFollowing` 放行即可。

插件内置 **VoxelNavLinkJumpProxy**（抛物线跳跃）：
- 角色默认走**引擎真实弹道**（切 MOVE_Falling、解析起跳速度、落地自动切回），`IsFalling` / `GetVelocity` / `Landed` 全是真的，动画蓝图的跳跃状态机不用改任何节点；
- 非角色或不吃物理时退回**运动学弧线**（代理逐帧挪位置，速度照常写回）；
- 蓝图子类重写 `ReceiveJumpStarted` 挂音效/蒙太奇/粒子。

### AI 占地
```cpp
Terrain->TryOccupyCoord(Coord);     // 占住一格（别人的 A* 会绕开）
Terrain->ReleaseCoord(Coord);
Terrain->FindFreeNearbyCoord(Target, AgentHeight, Radius); // 目标格被占时找附近空位
```
路径跟随组件开着 `bClaim Cells` 时会自动维护这些登记（出发预约终点、行进中占当前格、过连接占连接）。

> ⚠ **连着重发请求的顺序**：先 `StopMovement()` 再算路径。上一个请求预约的终点还占在占地表里，先寻路会把自己预约的格当成被别人占了。

---

## 使用注意事项

**地形参数与数据**
- `VoxelSize` / `MinHeight` / `MaxHeight` 决定坐标尺度与 Section 划分，**有地形数据后在细节面板里锁死**，要先点「清除地形」才能改。
- `MinHeight` / `MaxHeight` 必须是 16 的倍数。
- 单次 `FillBox` / `FillSphere` 有包围盒体积上限（约 100 万体素），防止蓝图笔误把编辑器卡死；超限整次拒绝并告警。

**资产**
- `UVoxelType` 必须在 Asset Manager 里以 `Voxel` 类型注册（见「安装」），否则网格重建会一直告警重试。
- 体素类型通过 `Type Name` 引用，重命名资产里的 Type Name 会让已有地形指向空类型（表现为该体素不再渲染）。

**网格与序列化**
- 网格组件是**派生数据，不入档**：载入关卡 / 进 PIE 后自动重建一次。别手动改网格组件；体素与网格不同步时（如改了材质、存档来自旧版本）点「重建地形网格」。
- 编辑器里「生成默认地形」走事务，可 Ctrl+Z；「清除地形」保存关卡后地图里就真的没有地形了。
- 体素旋转量化到 90°（24 向），任意角度会被 snapped。

**导航**
- 整张导航图按**「占地 1 格」的 AI** 烘焙：只判落脚格自身那一列的净空，不判身体横向扫到的格子。1 格宽的隘口、贴墙的落脚格对宽体型 AI 是假通路。胶囊半径 40 上下的人形 AI（VoxelSize=100 即 1 米）占 1 格是准确的。
- `Max Allow Height` 与 `Nav Link Max Height Diff` 都被夹在 `[1, 15]`：导航牵连范围刻意不超过一层，保证改一体素只需重烘上下各一个 Section。
- 骨骼网格体、Pawn 身体、触发器不参与烘焙；静态网格体按世界 AABB 近似光栅化（旋转/凹体会偏保守多占几格）。
- 在蓝图里改自动连接开关/参数（`ConfigureAutoNavLinks`）会**立刻全量重烘一次**导航，不是廉价操作。
- AIController 管朝向时，关掉组件的 `bFace Move Direction` 和跳跃代理的 `bFace Destination`，免得两边抢方向。
- 跳跃弧线期间别用 Root Motion 驱动 Actor，会和弧线打架。

---

## 实现原理

### 数据存储：Chunk → Section → 调色板
```
AVoxelTerrainActor
 └─ Chunks: TMap<FIntVector2, FVoxelChunk>          XY 方向按 Chunk 划分（16×16 列，纵贯全高度范围）
     └─ FVoxelChunk.Sections: FVoxelSection[]        Z 方向按 Section 划分（16³ 体素 = 4096 格）
         └─ 调色板 + 位打包                            Section 内体素的实际存储
```
- 每个 Section 维护一个 `FVoxelState`（Type + Rotation）调色板，体素本身只存调色板索引的**位打包数据**：4 种状态只需 2 bit/体素，一个 Section 满打满算 1 KB。
- 全空或全同的 Section 走**单值模式**（`BitsPerVoxel == 0`），连索引数组都不存。
- 写入时调色板不够用会自动扩位重打包；单 Section 变更超过 100 次（`CompactThreshold`）触发 `Compact()`，回收不再使用的调色板项，缩回最紧位宽。
- 24 向旋转编码为 6 bit（Roll/Pitch/Yaw 各 2 bit 量化），一个 `FVoxelState` 共 4 字节。

### 网格生成：贪心合并
`FVoxelSection::BuildMeshData` 按六个面方向分别扫描，把**同类型、同朝向、共面**的暴露面合并成最大矩形（贪心合并），每个材质槽（Type + Rotation）输出一份 `FVoxelMeshData`，再整段喂给 ProceduralMeshComponent。

- **面剔除**：相邻体素非透明则藏面；相邻是透明体素（`bTranslucent`）或跨出 Section 边界时向邻 Section 查询后再定。
- **24 向旋转**在网格期解为一个 3×3 带符号置换矩阵，统一驱动「面朝向选择、UV 展开、切线基（含手性）」，保证「方块的贴图朝向」与「把它当模型转 90°」一致。
- `bOrientationSensitive = false` 的类型在合并判定时忽略 Rotation，能合并出更大的矩形。

### 增量重建：脏区
改一体素时（`MarkSectionDirty`）：
- 本 Section 必然重建；
- 改动落在边界上，相邻 Section 的**遮挡判断**会变，一并标脏；
- Z 方向按「净空向上看 + 台阶连接」的可达高度（`GetMaxImpactHeight`）向上/向下多标一个 Section。

脏区在游戏 Tick 里按预算消化（默认每帧 5 个 Section），每个脏 Section 同时重建网格与导航数据。编辑器世界不跑 Tick，所以编辑器按钮内部强制全量重建。

### 序列化
体素数据全是 UPROPERTY，序列化直接交给反射系统，随关卡存档走；网格组件、导航数据都是派生数据，不入档也不参与对象复制。载入包体后 `Serialize` 只打一个「需要补建网格」的标记：编辑器等 AssetManager 初扫完成后重建（保证材质能解析出来），游戏侧由 BeginPlay 统一重建。

### 导航烘焙
`FVoxelSection::BuildNavData` 对本 Section 外扩一圈的窄带做三件事：
1. **光栅化外部阻碍**：对窄带做对象类型 Overlap，把命中的静态网格体（ISM/HISM 按单实例包围盒）换算成被占体素；地形自己的 ProceduralMesh、骨骼网格体、Pawn 一律忽略。
2. **合成实心栅格**：体素 + 外部阻碍 → 布尔格。落脚格 = 自身空、下方有支撑；净空 = 向上连续空格数（封顶 `MaxAllowHeight`，取到上限只表示「至少这么高」——所以净空变化不会向上传播超过一层）。
3. **写连接**：同层水平 4 邻写平面连接；高差 ≤ `NavLinkMaxHeightDiff` 的相邻格挂自动代理连接；手动连接按端点归属分堆写入（可跨 Section 任意远，支持单向）。烘焙出的图是无向图（平面连接双向写），A* 顺着 Links 就能走到任意可达格。

### A* 寻路
- 启发式为一致性函数，终点在**弹出**时确认，此时 g 值即最短代价，回溯即得路径。
- 引擎 Core 没有现成优先队列，手搓了一个最小的二叉小顶堆；不支持改键，同一格找到更短路线就重复入堆，弹出时靠 Closed 集合**惰性删除**。
- 代价 = `max(1, 区域权重) × max(1, 代理权重)`，按「进入这一格」计价（起点不计）。只罚不奖（低于 1 按 1 算）是为了保住「每步 ≥ 1」的启发式下界。
- **占地只在查询期判**：被占的格直接不展开，绝不写进烘焙数据。
- 展开超过 5 万节点放弃并告警（大概率不连通或太远）。

### 路径跟随
- **双驱动**：`NavMovement` 模式向实现了 `INavMovementInterface` 的组件下发速度/输入，到点按容差判定，还处理了「带加减速一步越过格心导致来回蹭」的**垂足判定**；`DirectLocation` 模式直接插值并在到点时精确吸附格心。`Auto` 优先前者。
- **连接握手**：走到带代理的跳点时进入 `WaitingLink`——先占住这条连接，把 Agent 交给代理（如跳跃），代理做完动作调 `ResumePathFollowing` 放行，组件才继续走。期间拒绝暂停（要停得让代理停它自己的动作），并有等待超时兜底。
- **占地表**：出发时预约终点格、行进中维护当前格、过连接时占住连接，别人的 A* 因此能自动绕开。这套登记有顺序要求（先 Stop 再重新寻路），见上文注意事项。
- **站位口径**：所有落点都是「格心 + 竖直偏移」（自动按胶囊半高 − 半个体素推算，让脚底踩在格底面上），保证「Actor 位置 ↔ 格坐标」始终一一对应。

---

## 自动化测试

插件带一组路径跟随的回归/诊断测试（纯代码搭地形，不依赖关卡与资产）：

```sh
UnrealEditor-Cmd.exe <项目>.uproject -ExecCmds="Automation RunTests VoxelTerrain.PathFollowing;Quit" -Unattended -NoSound -NullRHI -Stdout -AllowStdOutLogVerbosity -NoLoadingScreen
```

---

## 目录结构

```
VoxelTerrain/
├─ VoxelTerrain.uplugin            插件描述（Runtime 模块，依赖 ProceduralMeshComponent）
├─ Config/FilterPlugin.ini         打包过滤配置
├─ Source/VoxelTerrain/
│  ├─ VoxelTerrain.Build.cs        模块依赖
│  ├─ Public/
│  │  ├─ VoxelTerrainActor.h       地形 Actor：编辑 API、脏区、坐标换算、占地表
│  │  ├─ VoxelChunk.h              Chunk / Section / 调色板存储、网格与导航数据结构
│  │  ├─ VoxelType.h               体素类型资产（材质、碰撞、透明、朝向敏感）
│  │  ├─ VoxelGenerator.h          生成器基类（蓝图可扩展）
│  │  ├─ VoxelNavLinkProxy.h       非平面连接代理基类
│  │  ├─ VoxelNavLinkJumpProxy.h   抛物线跳跃代理
│  │  └─ VoxelPathFollowingComponent.h  路径跟随组件
│  └─ Private/
│     ├─ VoxelChunk.cpp            调色板位打包 / Compact
│     ├─ VoxelChunk_Mesh.cpp       贪心合并网格生成
│     ├─ VoxelChunk_Nav.cpp        导航烘焙（落脚格 / 净空 / 连接 / 阻碍光栅化）
│     ├─ VoxelTerrainActor.cpp     编辑 API / 脏区 / 序列化 / 体素射线
│     ├─ VoxelTerrainActor_Nav.cpp 全量烘焙 / 权重 / A* / 占地表
│     ├─ VoxelPathFollowingComponent.cpp  路径跟随实现
│     ├─ VoxelNavLinkJumpProxy.cpp 跳跃代理实现
│     └─ VoxelPathFollowingTest.cpp 自动化测试
└─ (项目侧示例) /Game/Voxel/BP_TestGenerator、DA_TestVoxelType
```

---

## 已知限制

- 导航按「占地 1 格、竖直净空」的 AI 烘焙，宽体型（横向多格）需要烘焙按半径分档记录净空，当前未实现（查询期补不出来）。
- 外部静态网格体的导航阻碍用世界 AABB 近似，旋转或凹形网格会偏保守。
- 导航图不支持运行期「半格」精度；地形与导航的更新粒度都是整格。
- 未做路径中途重算与动态避障——按「一次只有一个角色动」的回合制假设设计，动态多 AI 拥挤场景请配合占地表与 `FindFreeNearbyCoord` 自行调度。
