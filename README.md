# gogrid

网格地图的小型工具库：解析、A* 寻路、逐格视线（Bresenham）以及扇形视野裁剪。

## 扇形视野

```go
g, _ := grid.Parse([]string{
    "..........",
    ".....#####",
    "..........",
    "..........",
})

cfg := grid.FOVConfig{
    Origin:   grid.Point{X: 4, Y: 2},
    Facing:   grid.North,  // N / E / S / W
    AngleDeg: 90,          // (0, 360]，360 即全开
    Radius:   6,           // [1, MaxRadius()]
}
vis, err := g.FOV(cfg)
```

| 方法 | 含义 |
| --- | --- |
| `VisibleSet(origin, radius)` | 全开：半径内、逐格视线可见的可走格（基准） |
| `FOV(cfg)` | 扇形可见格 = 扇形几何掩码 ∩ 逐格视线可见集 |
| `SectorCells(cfg)` | 仅按几何裁剪的扇形掩码（无遮挡上界） |
| `DiffSets(a, b)` | 双向差集，用于对照与统计 |
| `ValidateFOV(cfg)` | 只校验参数，不计算 |

### 语义口径

- **可见格**：可走格（`#` 为墙，不返回）。观察点自身总在结果内。
- **半径**：格心之间的欧氏距离，`dx²+dy² ≤ r²`（含等号）。
- **张角**：以朝向中线为轴的对称扇形，夹角 `≤ AngleDeg/2`（含 `1e-9` 容差）。
- **全开不变**：`AngleDeg == 360` 时 `FOV` 直接委托 `VisibleSet`，内容与顺序完全一致。
- **子集关系**：任意张角下 `FOV ⊆ VisibleSet`；任意 360° 下 `FOV == VisibleSet`。
- **差集上界**：`FOV \ VisibleSet` 恒为空（测试逐项核对）。实际有意义的量是
  `SectorCells \ FOV`——落在扇形内却被墙挡掉的格，即“遮挡损失”。

### 参数错误

错误均以参数名开头，可用 `errors.Is` 判定：

| 情形 | 错误 | 信息前缀 |
| --- | --- | --- |
| 朝向非 N/E/S/W | `ErrFacing` | `facing:` |
| 张角为 0/负数/NaN/Inf/>360 | `ErrAngle` | `angle:` |
| 半径 <1 或 >`MaxRadius()` | `ErrRadius` | `radius:` |
| 观察点越界或为墙 | `ErrOrigin` | `origin:` |

`MaxRadius()` = `ceil(hypot(W, H))`。

## 测试

```sh
go test ./...
go test ./grid -bench 'BenchmarkFOV' -benchmem
```

覆盖：参数校验（含边界合法值）、360° 与全开逐元素一致、
可见格数随张角单调不减、窄角/宽角几何边界、贴墙、地图角与凹角、
全遮挡、固定种子随机地图批量对照、`SectorCells` 上界关系。

## 统计命令

```sh
go run ./cmd/fovstat                          # 地图集 + 汇总表
go run ./cmd/fovstat -scenario wall-hug       # 单场景
go run ./cmd/fovstat -map '.....,.##..,..#..,.....' -x 0 -y 0 -facing E -radius 4 -angles 90,360
```

`-map` 用逗号分隔行，`#` 为墙。输出包含：地图、逐档可见格数、扇形掩码格数、
被遮挡格及其坐标、FOV 越出全开集的格数（应为 0）；再用两档半径
（基准 R 与 `max(1, R/2)`）各报可见格数与中位数耗时（51 次采样）。
