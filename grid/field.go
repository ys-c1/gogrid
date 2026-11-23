package grid

import (
	"errors"
	"fmt"
	"sync/atomic"
)

var (
	ErrNoTargets     = errors.New("未提供目标")
	ErrTargetBlocked = errors.New("目标不可走")
	ErrZeroGradient  = errors.New("梯度为零")
)

// CoordError 携带出错坐标，便于上层直接定位。
type CoordError struct {
	Kind error
	At   Point
}

func (e *CoordError) Error() string {
	return fmt.Sprintf("%v: 坐标 (%d,%d)", e.Kind, e.At.X, e.At.Y)
}

func (e *CoordError) Unwrap() error { return e.Kind }

var neighbors4 = [...]Point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}

// Field 是从一组目标格反向 BFS 得到的共享距离场。
// 单位只读距离场做梯度下降，不再各自寻路。
type Field struct {
	g        *Grid
	dist     []int // 下标 y*W+x；-1 表示墙体或不可达
	targets  []Point
	isTarget map[Point]struct{}
	size     int // 距离场覆盖的可走格数
	fallback atomic.Int64
}

// FieldStats 用于收尾汇报。
type FieldStats struct {
	W, H      int
	Walkable  int // 全图可走格
	Covered   int // 距离场规模（可达可走格）
	Targets   int
	Fallbacks int
}

// BuildField 从 targets 多源反向扩散距离场。目标可多个；原有 Path 接口不受影响。
func BuildField(g *Grid, targets ...Point) (*Field, error) {
	var at Point
	if len(targets) > 0 {
		at = targets[0]
	}
	if g == nil || g.W <= 0 || g.H <= 0 || len(g.cells) != g.H {
		return nil, &CoordError{Kind: ErrEmpty, At: at}
	}
	if len(targets) == 0 {
		return nil, &CoordError{Kind: ErrNoTargets, At: at}
	}

	f := &Field{
		g:        g,
		dist:     make([]int, g.W*g.H),
		isTarget: make(map[Point]struct{}, len(targets)),
	}
	for i := range f.dist {
		f.dist[i] = -1
	}

	queue := make([]Point, 0, len(targets))
	for _, t := range targets {
		ok, err := g.Walkable(t)
		if err != nil {
			return nil, &CoordError{Kind: ErrOOB, At: t}
		}
		if !ok {
			return nil, &CoordError{Kind: ErrTargetBlocked, At: t}
		}
		if _, dup := f.isTarget[t]; dup {
			continue
		}
		f.isTarget[t] = struct{}{}
		f.targets = append(f.targets, t)
		f.dist[f.idx(t)] = 0
		f.size++
		queue = append(queue, t)
	}

	for head := 0; head < len(queue); head++ {
		cur := queue[head]
		d := f.dist[f.idx(cur)]
		for _, dir := range neighbors4 {
			n := Point{cur.X + dir.X, cur.Y + dir.Y}
			if !f.inGrid(n) {
				continue
			}
			ni := f.idx(n)
			if !g.cells[n.Y][n.X] || f.dist[ni] >= 0 {
				continue
			}
			f.dist[ni] = d + 1
			f.size++
			queue = append(queue, n)
		}
	}
	return f, nil
}

func (f *Field) inGrid(p Point) bool {
	return p.X >= 0 && p.Y >= 0 && p.X < f.g.W && p.Y < f.g.H
}

func (f *Field) idx(p Point) int { return p.Y*f.g.W + p.X }

// Dist 返回格点距离与是否在距离场内。
func (f *Field) Dist(p Point) (int, bool) {
	if f == nil || !f.inGrid(p) {
		return 0, false
	}
	d := f.dist[f.idx(p)]
	return d, d >= 0
}

func (f *Field) Size() int      { return f.size }
func (f *Field) Fallbacks() int { return int(f.fallback.Load()) }

func (f *Field) Targets() []Point {
	out := make([]Point, len(f.targets))
	copy(out, f.targets)
	return out
}

func (f *Field) Stats() FieldStats {
	return FieldStats{
		W:         f.g.W,
		H:         f.g.H,
		Walkable:  f.g.WalkableCount(),
		Covered:   f.size,
		Targets:   len(f.targets),
		Fallbacks: f.Fallbacks(),
	}
}

func (f *Field) isTargetCell(p Point) bool {
	_, ok := f.isTarget[p]
	return ok
}

// lowerNeighbor 返回严格下降的邻居中距离最小者（方向序作确定性 tie-break）。
func (f *Field) lowerNeighbor(p Point) (Point, bool) {
	d, ok := f.Dist(p)
	if !ok {
		return Point{}, false
	}
	var best Point
	bestD := d
	found := false
	for _, dir := range neighbors4 {
		n := Point{p.X + dir.X, p.Y + dir.Y}
		nd, ok := f.Dist(n)
		if !ok || nd >= bestD {
			continue
		}
		best, bestD, found = n, nd, true
	}
	return best, found
}

// Follow 让一个单位从 from 沿梯度下降到目标。
// 返回路线、本单位回退次数；场级回退总数见 Fallbacks。
func (f *Field) Follow(from Point) ([]Point, int, error) {
	if f == nil {
		return nil, 0, &CoordError{Kind: ErrEmpty, At: from}
	}
	if _, ok := f.Dist(from); !ok {
		// 越界与墙/不可达分开报错；墙与不可达格都没有梯度。
		if _, err := f.g.Walkable(from); err != nil {
			return nil, 0, &CoordError{Kind: ErrOOB, At: from}
		}
		return nil, 0, &CoordError{Kind: ErrZeroGradient, At: from}
	}

	path := []Point{from}
	cur := from
	fallbacks := 0
	for !f.isTargetCell(cur) {
		next, ok := f.lowerNeighbor(cur)
		if !ok {
			// 局部极小：计入回退，用原有单点 A* 接口补完剩余路线。
			fallbacks++
			f.fallback.Add(1)
			rest, err := f.fallbackPath(cur)
			if err != nil {
				return path, fallbacks, &CoordError{Kind: ErrZeroGradient, At: cur}
			}
			path = append(path, rest[1:]...)
			cur = rest[len(rest)-1]
			break
		}
		cur = next
		path = append(path, cur)
	}
	return path, fallbacks, nil
}

// fallbackPath 选择到任一目标的最短 A* 路径。
func (f *Field) fallbackPath(from Point) ([]Point, error) {
	var best []Point
	for _, t := range f.targets {
		p, _, err := f.g.Path(from, t)
		if err != nil {
			continue
		}
		if best == nil || len(p) < len(best) {
			best = p
		}
	}
	if best == nil {
		return nil, ErrNoPath
	}
	return best, nil
}

// CheckTriangle 核验距离场的三角不等式：相邻可走格的场值之差不超过 1。
// 相邻检查可组合出任意两点间的全局三角不等式。
func (f *Field) CheckTriangle() error {
	for y := 0; y < f.g.H; y++ {
		for x := 0; x < f.g.W; x++ {
			p := Point{x, y}
			d, ok := f.Dist(p)
			if !ok {
				continue
			}
			if f.isTargetCell(p) {
				if d != 0 {
					return fmt.Errorf("三角不等式核验: 目标格 (%d,%d) 距离为 %d，应为 0", x, y, d)
				}
			} else if d == 0 {
				return fmt.Errorf("三角不等式核验: 非目标格 (%d,%d) 距离为 0", x, y)
			}
			for _, dir := range neighbors4 {
				n := Point{x + dir.X, y + dir.Y}
				nd, ok := f.Dist(n)
				if !ok {
					if f.inGrid(n) && f.g.cells[n.Y][n.X] {
						return fmt.Errorf("三角不等式核验: 可走格 (%d,%d) 与 (%d,%d) 相邻却不在同一距离场",
							x, y, n.X, n.Y)
					}
					continue
				}
				if d-nd > 1 || nd-d > 1 {
					return fmt.Errorf("三角不等式核验: (%d,%d)=%d 与 (%d,%d)=%d 相邻，落差超过 1",
						x, y, d, n.X, n.Y, nd)
				}
			}
		}
	}
	return nil
}

// CheckDescent 核验：从距离场中每个格子沿严格下降的梯度走，终点必为目标格。
func (f *Field) CheckDescent() error {
	const (
		unknown int8 = iota
		good
	)
	state := make([]int8, len(f.dist))
	for y := 0; y < f.g.H; y++ {
		for x := 0; x < f.g.W; x++ {
			start := Point{x, y}
			if _, ok := f.Dist(start); !ok {
				continue
			}
			cur := start
			var trail []int
			steps := 0
			for {
				ci := f.idx(cur)
				if state[ci] == good {
					break
				}
				if f.isTargetCell(cur) {
					state[ci] = good
					break
				}
				next, ok := f.lowerNeighbor(cur)
				if !ok {
					d, _ := f.Dist(cur)
					return fmt.Errorf("梯度下降核验: 在 (%d,%d) 陷入局部极小，距离 %d",
						cur.X, cur.Y, d)
				}
				trail = append(trail, ci)
				cur = next
				steps++
				if steps > f.size {
					return fmt.Errorf("梯度下降核验: 从 (%d,%d) 出发未在 %d 步内抵达目标",
						start.X, start.Y, f.size)
				}
			}
			for _, i := range trail {
				state[i] = good
			}
		}
	}
	return nil
}
