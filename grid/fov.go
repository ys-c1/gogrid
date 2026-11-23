package grid

import (
	"errors"
	"fmt"
	"math"
)

// Facing 是视野朝向。屏幕坐标系：x 向右，y 向下。
type Facing int

const (
	North Facing = iota // 上（-y）
	East                // 右（+x）
	South               // 下（+y）
	West                // 左（-x）
)

// 参数错误：错误信息以参数名开头，便于调用方定位。
var (
	ErrFacing = errors.New("facing")
	ErrAngle  = errors.New("angle")
	ErrRadius = errors.New("radius")
	ErrOrigin = errors.New("origin")
)

func (f Facing) String() string {
	switch f {
	case North:
		return "N"
	case East:
		return "E"
	case South:
		return "S"
	case West:
		return "W"
	}
	return fmt.Sprintf("Facing(%d)", int(f))
}

// Valid 报告朝向是否为四个合法方向之一。
func (f Facing) Valid() bool { return f >= North && f <= West }

// centerDeg 返回朝向的中线角度（度）。atan2 的 0° 指向 +x。
func (f Facing) centerDeg() float64 {
	switch f {
	case North:
		return -90
	case East:
		return 0
	case South:
		return 90
	case West:
		return 180
	}
	return 0
}

// FOVConfig 描述一次扇形视野计算。
//
//   - Origin 为观察点，必须是可行走格；
//   - Facing 为中线朝向；
//   - AngleDeg 为张角（度），取值 (0, 360]，360 表示全开；
//   - Radius 为欧氏半径（格），取值 [1, MaxRadius()]。
type FOVConfig struct {
	Origin   Point
	Facing   Facing
	AngleDeg float64
	Radius   int
}

// MaxRadius 返回合法半径的上界：网格外接矩形的对角线向上取整。
func (g *Grid) MaxRadius() int {
	return int(math.Ceil(math.Hypot(float64(g.W), float64(g.H))))
}

// Validate 只校验参数，不计算视野。返回的错误均带参数名。
func (g *Grid) ValidateFOV(cfg FOVConfig) error {
	if !cfg.Facing.Valid() {
		return fmt.Errorf("%w: 朝向必须是 N/E/S/W 之一（收到 %v）", ErrFacing, cfg.Facing)
	}
	if math.IsNaN(cfg.AngleDeg) || math.IsInf(cfg.AngleDeg, 0) ||
		cfg.AngleDeg <= 0 || cfg.AngleDeg > 360 {
		return fmt.Errorf("%w: 张角必须在 (0, 360] 度内（收到 %v）", ErrAngle, cfg.AngleDeg)
	}
	if cfg.Radius < 1 || cfg.Radius > g.MaxRadius() {
		return fmt.Errorf("%w: 半径必须在 [1, %d] 内（收到 %d）", ErrRadius, g.MaxRadius(), cfg.Radius)
	}
	return g.validateOrigin(cfg.Origin)
}

func (g *Grid) validateOrigin(p Point) error {
	ok, err := g.Walkable(p)
	if err != nil {
		return fmt.Errorf("%w: 坐标 (%d,%d) 越界", ErrOrigin, p.X, p.Y)
	}
	if !ok {
		return fmt.Errorf("%w: 坐标 (%d,%d) 是墙，不能作为观察点", ErrOrigin, p.X, p.Y)
	}
	return nil
}

// VisibleSet 是“全开”视野：半径内、逐格视线可见的所有可行走格。
// 观察点自身总是包含在内；墙体不作为可见格返回（与 Visible 语义一致）。
// 结果按行优先（y 升序、x 升序）排列。
func (g *Grid) VisibleSet(origin Point, radius int) ([]Point, error) {
	if radius < 1 || radius > g.MaxRadius() {
		return nil, fmt.Errorf("%w: 半径必须在 [1, %d] 内（收到 %d）", ErrRadius, g.MaxRadius(), radius)
	}
	if err := g.validateOrigin(origin); err != nil {
		return nil, err
	}
	r2 := radius * radius
	out := make([]Point, 0)
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if !g.cells[y][x] {
				continue
			}
			dx, dy := x-origin.X, y-origin.Y
			if dx*dx+dy*dy > r2 {
				continue
			}
			ok, err := g.Visible(origin, Point{X: x, Y: y})
			if err != nil || !ok {
				continue
			}
			out = append(out, Point{X: x, Y: y})
		}
	}
	return out, nil
}

// FOV 计算扇形视野内的可见格。
//
// 结果 = “扇形几何掩码” ∩ “逐格视线可见集”：只有落在张角与半径内的格
// 才做 Bresenham 视线判定，因此窄角时的工作量显著小于全开扫描。
// 由此返回值恒为 VisibleSet 的子集；AngleDeg == 360 时直接返回 VisibleSet，
// 与全开模式完全一致（内容与顺序均不变）。
// 半径使用格心之间的欧氏距离，边界含等号；角度边界含 1e-9 容差。
// 结果按行优先排列。
func (g *Grid) FOV(cfg FOVConfig) ([]Point, error) {
	if err := g.ValidateFOV(cfg); err != nil {
		return nil, err
	}
	if cfg.AngleDeg >= 360 {
		return g.VisibleSet(cfg.Origin, cfg.Radius)
	}
	r2 := cfg.Radius * cfg.Radius
	out := make([]Point, 0)
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if !g.cells[y][x] {
				continue
			}
			dx, dy := x-cfg.Origin.X, y-cfg.Origin.Y
			if dx*dx+dy*dy > r2 {
				continue
			}
			p := Point{X: x, Y: y}
			if !inSector(cfg.Origin, p, cfg.Facing, cfg.AngleDeg) {
				continue
			}
			ok, err := g.Visible(cfg.Origin, p)
			if err != nil || !ok {
				continue
			}
			out = append(out, p)
		}
	}
	return out, nil
}

// SectorCells 只按几何返回扇形内的可走格：半径内、张角内、不检查遮挡。
// 它是 FOV 的“无遮挡上界”——FOV 结果 = SectorCells ∩ 逐格视线可见集。
// 参数校验与 FOV 相同；AngleDeg == 360 时返回半径内所有可走格。
// 结果按行优先排列。
func (g *Grid) SectorCells(cfg FOVConfig) ([]Point, error) {
	if err := g.ValidateFOV(cfg); err != nil {
		return nil, err
	}
	r2 := cfg.Radius * cfg.Radius
	out := make([]Point, 0)
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if !g.cells[y][x] {
				continue
			}
			p := Point{X: x, Y: y}
			dx, dy := p.X-cfg.Origin.X, p.Y-cfg.Origin.Y
			if dx*dx+dy*dy > r2 {
				continue
			}
			if cfg.AngleDeg < 360 && !inSector(cfg.Origin, p, cfg.Facing, cfg.AngleDeg) {
				continue
			}
			out = append(out, p)
		}
	}
	return out, nil
}

// inSector 判断格 p 是否落在以 origin 为中心、朝向 facing、张角 angleDeg 的扇形内。
func inSector(origin, p Point, facing Facing, angleDeg float64) bool {
	dx, dy := p.X-origin.X, p.Y-origin.Y
	if dx == 0 && dy == 0 {
		return true
	}
	deg := math.Atan2(float64(dy), float64(dx)) * 180 / math.Pi
	diff := normalize180(deg - facing.centerDeg())
	return math.Abs(diff) <= angleDeg/2+1e-9
}

// normalize180 把角度差归一化到 [-180, 180)。
func normalize180(d float64) float64 {
	d = math.Mod(d+180, 360)
	if d < 0 {
		d += 360
	}
	return d - 180
}

// SetDiff 表示两个点集的双向差集。
type SetDiff struct {
	OnlyA []Point // 在 A 中、不在 B 中
	OnlyB []Point // 在 B 中、不在 A 中
}

// DiffSets 计算 a\b 与 b\a，保持各自输入中的行优先顺序。
func DiffSets(a, b []Point) SetDiff {
	inA := make(map[Point]struct{}, len(a))
	inB := make(map[Point]struct{}, len(b))
	for _, p := range a {
		inA[p] = struct{}{}
	}
	for _, p := range b {
		inB[p] = struct{}{}
	}
	var d SetDiff
	for _, p := range a {
		if _, ok := inB[p]; !ok {
			d.OnlyA = append(d.OnlyA, p)
		}
	}
	for _, p := range b {
		if _, ok := inA[p]; !ok {
			d.OnlyB = append(d.OnlyB, p)
		}
	}
	return d
}
