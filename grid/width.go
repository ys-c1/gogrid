package grid

import (
	"errors"
	"fmt"
	"math"
)

// 宽度约束寻路相关错误。原有窄身接口 Path 的错误语义保持不变，
// 这里的错误仅由 PathWide / Inflate 等新接口返回。
var (
	ErrRadius    = errors.New("半径非法")
	ErrBlocked   = errors.New("起终点被堵")
	ErrClearance = errors.New("宽度约束未满足：路径与障碍距离不足")
	ErrDetour    = errors.New("路径长度超出允许比例")
	ErrRounds    = errors.New("膨胀轮次超出折算结果")
)

// RadiusError 记录非法的半径值（负数、NaN 或无穷大）。
type RadiusError struct{ Radius float64 }

func (e *RadiusError) Error() string { return fmt.Sprintf("%v: %g", ErrRadius, e.Radius) }
func (e *RadiusError) Unwrap() error { return ErrRadius }

// BoundsError 记录越界坐标，便于直接定位。
type BoundsError struct{ Point Point }

func (e *BoundsError) Error() string {
	return fmt.Sprintf("%v: (%d,%d)", ErrOOB, e.Point.X, e.Point.Y)
}
func (e *BoundsError) Unwrap() error { return ErrOOB }

// BlockedError 记录被堵端点及其坐标。Role 标明是起点还是终点，
// 以及堵塞发生在原网格还是膨胀后的网格。
type BlockedError struct {
	Point Point
	Role  string
}

func (e *BlockedError) Error() string {
	return fmt.Sprintf("%v: %s (%d,%d)", ErrBlocked, e.Role, e.Point.X, e.Point.Y)
}
func (e *BlockedError) Unwrap() error { return ErrBlocked }

const maxInt = int(^uint(0) >> 1)

// RoundsForRadius 把半径折算成膨胀轮数。一格为一个距离单位，
// 半径 r 需要 ceil(r) 轮 8 邻域膨胀；r=0 时不膨胀。
// 极大的半径饱和到 maxInt，交由 Inflate 按网格尺寸截断。
func RoundsForRadius(radius float64) (int, error) {
	if math.IsNaN(radius) || math.IsInf(radius, 0) || radius < 0 {
		return 0, &RadiusError{Radius: radius}
	}
	c := math.Ceil(radius)
	if c >= float64(maxInt) {
		return maxInt, nil
	}
	return int(c), nil
}

// CheckRounds 核验膨胀轮次：实测轮数不得多于按半径折算的结果。
// 返回折算值，便于调用方登记。
func CheckRounds(measured int, radius float64) (int, error) {
	derived, err := RoundsForRadius(radius)
	if err != nil {
		return 0, err
	}
	if measured > derived {
		return derived, fmt.Errorf("%w: 实测 %d > 折算 %d", ErrRounds, measured, derived)
	}
	return derived, nil
}

// PathSteps 返回相邻格步数，即路径边数。单点路径为 0，空路径为 0。
func PathSteps(path []Point) int {
	if len(path) < 2 {
		return 0
	}
	return len(path) - 1
}

// Inflate 返回按半径膨胀后的新网格，以及实际执行的膨胀轮数。
// 每一轮把与障碍格 8 邻接的可走格一并堵住；网格外不算障碍。
// 轮数由 RoundsForRadius 折算，最多按网格尺寸截断，绝不会多于折算值。
func (g *Grid) Inflate(radius float64) (*Grid, int, error) {
	if g == nil || g.W <= 0 || g.H <= 0 {
		return nil, 0, ErrEmpty
	}
	rounds, err := RoundsForRadius(radius)
	if err != nil {
		return nil, 0, err
	}
	// 轮数超过 W+H 后网格必然全堵，截断只是饱和，不改变结果。
	if maxRounds := g.W + g.H; rounds > maxRounds {
		rounds = maxRounds
	}
	out := g.clone()
	for i := 0; i < rounds; i++ {
		out = out.dilateOnce()
	}
	return out, rounds, nil
}

func (g *Grid) clone() *Grid {
	out := &Grid{W: g.W, H: g.H, cells: make([][]bool, g.H)}
	for y := range g.cells {
		out.cells[y] = append([]bool(nil), g.cells[y]...)
	}
	return out
}

// dilateOnce 做一轮 8 邻域膨胀。障碍格保持不可走，邻接障碍的可走格变不可走。
func (g *Grid) dilateOnce() *Grid {
	out := &Grid{W: g.W, H: g.H, cells: make([][]bool, g.H)}
	for y := 0; y < g.H; y++ {
		out.cells[y] = make([]bool, g.W)
		for x := 0; x < g.W; x++ {
			if !g.cells[y][x] {
				continue
			}
			open := true
			for dy := -1; dy <= 1 && open; dy++ {
				for dx := -1; dx <= 1; dx++ {
					nx, ny := x+dx, y+dy
					if nx < 0 || ny < 0 || nx >= g.W || ny >= g.H {
						continue
					}
					if !g.cells[ny][nx] {
						open = false
						break
					}
				}
			}
			out.cells[y][x] = open
		}
	}
	return out
}

// MinClearance 返回路径上各格中心到最近障碍格（#）中心的欧氏距离最小值。
// 路径为空或网格中没有障碍时返回 +Inf。
func MinClearance(g *Grid, path []Point) float64 {
	if g == nil || len(path) == 0 {
		return math.Inf(1)
	}
	var obstacles []Point
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if !g.cells[y][x] {
				obstacles = append(obstacles, Point{x, y})
			}
		}
	}
	if len(obstacles) == 0 {
		return math.Inf(1)
	}
	bestSq := maxInt
	for _, p := range path {
		for _, o := range obstacles {
			dx, dy := p.X-o.X, p.Y-o.Y
			if d := dx*dx + dy*dy; d < bestSq {
				bestSq = d
			}
		}
	}
	return math.Sqrt(float64(bestSq))
}

// CheckClearance 单独核验不变量一：路径与障碍的最小距离不得小于半径。
func CheckClearance(g *Grid, path []Point, radius float64) (float64, error) {
	if math.IsNaN(radius) || math.IsInf(radius, 0) || radius < 0 {
		return 0, &RadiusError{Radius: radius}
	}
	d := MinClearance(g, path)
	if d+1e-9 < radius {
		return d, fmt.Errorf("%w: 最小距离 %.6g < 半径 %g", ErrClearance, d, radius)
	}
	return d, nil
}

// CheckDetour 单独核验不变量二：宽路径相对窄路径的步数增幅不得超过 maxDetour。
func CheckDetour(narrowSteps, wideSteps int, maxDetour float64) (float64, error) {
	if math.IsNaN(maxDetour) || math.IsInf(maxDetour, 0) || maxDetour < 0 {
		return 0, fmt.Errorf("%w: 允许比例非法 %g", ErrDetour, maxDetour)
	}
	if narrowSteps < 0 {
		narrowSteps = 0
	}
	if wideSteps < 0 {
		wideSteps = 0
	}
	ratio := 0.0
	if narrowSteps > 0 {
		ratio = float64(wideSteps-narrowSteps) / float64(narrowSteps)
	} else if wideSteps > 0 {
		ratio = math.Inf(1)
	}
	if ratio > maxDetour+1e-9 {
		return ratio, fmt.Errorf("%w: 实际增加 %.6g > 允许 %g", ErrDetour, ratio, maxDetour)
	}
	return ratio, nil
}

// WideResult 是带宽度约束寻路的结果与核验数据。
type WideResult struct {
	Path        []Point
	Visited     int
	Radius      float64
	Rounds      int
	MinDistance float64 // 到原网格障碍的最小欧氏距离；无路径时为 +Inf
	NarrowSteps int     // 原有窄身最短路径步数
	WideSteps   int     // 膨胀后路径步数；无路径时为 0（此时 Path 为 nil）
	Detour      float64 // (WideSteps-NarrowSteps)/NarrowSteps
}

// PathWide 在按半径膨胀后的网格上寻找从 from 到 to 的最短路径。
// 路径只经过膨胀后仍可走的格；两条不变量逐条核验：
//  1. 路径格到原障碍的最小距离 >= radius（CheckClearance）；
//  2. 相对原窄身最短路径的步数增幅 <= maxDetour（CheckDetour）。
//
// 另外核验实测膨胀轮数不多于 RoundsForRadius 的折算值（CheckRounds）。
// 半径为负、网格为空、端点越界或端点被堵都会返回带具体数值/坐标的错误；
// 端点被堵会同时报出起点和终点中所有被堵的坐标。
// 不变量失败或膨胀后不连通时，返回的 WideResult 仍非 nil，便于查看实测值。
func (g *Grid) PathWide(from, to Point, radius, maxDetour float64) (*WideResult, error) {
	if g == nil || g.W <= 0 || g.H <= 0 {
		return nil, ErrEmpty
	}
	if _, err := RoundsForRadius(radius); err != nil {
		return nil, err
	}
	if math.IsNaN(maxDetour) || math.IsInf(maxDetour, 0) || maxDetour < 0 {
		return nil, fmt.Errorf("%w: 允许比例非法 %g", ErrDetour, maxDetour)
	}
	if err := g.checkBounds(from); err != nil {
		return nil, err
	}
	if err := g.checkBounds(to); err != nil {
		return nil, err
	}
	if errs := g.blockedErrors(from, to, "原网格"); len(errs) > 0 {
		return nil, errors.Join(errs...)
	}

	// 窄身最短路径作为长度基准；窄身都不通时宽度约束无从谈起。
	narrow, _, err := g.Path(from, to)
	if err != nil {
		return nil, err
	}
	narrowSteps := PathSteps(narrow)

	inflated, rounds, err := g.Inflate(radius)
	if err != nil {
		return nil, err
	}
	// 轮次核验：实测不得多于折算结果。
	if _, err := CheckRounds(rounds, radius); err != nil {
		return nil, err
	}
	if errs := inflated.blockedErrors(from, to, "膨胀后"); len(errs) > 0 {
		return nil, errors.Join(errs...)
	}

	wide, visited, err := inflated.Path(from, to)
	if err != nil {
		return &WideResult{
			Radius: radius, Rounds: rounds, NarrowSteps: narrowSteps,
			MinDistance: math.Inf(1),
		}, err
	}
	wideSteps := PathSteps(wide)

	minDist, clrErr := CheckClearance(g, wide, radius)
	res := &WideResult{
		Path: wide, Visited: visited, Radius: radius, Rounds: rounds,
		MinDistance: minDist, NarrowSteps: narrowSteps, WideSteps: wideSteps,
	}
	if clrErr != nil {
		return res, clrErr
	}
	detour, detErr := CheckDetour(narrowSteps, wideSteps, maxDetour)
	res.Detour = detour
	if detErr != nil {
		return res, detErr
	}
	return res, nil
}

func (g *Grid) checkBounds(p Point) error {
	if p.X < 0 || p.Y < 0 || p.X >= g.W || p.Y >= g.H {
		return &BoundsError{Point: p}
	}
	return nil
}

func (g *Grid) blockedErrors(from, to Point, stage string) []error {
	var errs []error
	if !g.cells[from.Y][from.X] {
		errs = append(errs, &BlockedError{Point: from, Role: stage + "起点"})
	}
	if !g.cells[to.Y][to.X] {
		errs = append(errs, &BlockedError{Point: to, Role: stage + "终点"})
	}
	return errs
}
