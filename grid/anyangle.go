package grid

import (
	"container/heap"
	"errors"
	"fmt"
	"math"
)

// 任意角寻路的输入/判定错误。
var (
	ErrStartBlocked = errors.New("起点在墙内")
	ErrGoalBlocked  = errors.New("目标在墙内")
	ErrSight        = errors.New("视线判定失败")
)

// SightError 表示一次视线判定失败，并携带出问题的两个端点坐标。
// Err 为 nil 表示视线被墙阻断；非 nil 表示 Visible 自身返回了错误（例如越界）。
type SightError struct {
	A, B Point
	Err  error
}

func (e *SightError) Error() string {
	if e.Err == nil {
		return fmt.Sprintf("%v: (%d,%d) 与 (%d,%d) 之间被墙阻断",
			ErrSight, e.A.X, e.A.Y, e.B.X, e.B.Y)
	}
	return fmt.Sprintf("%v: (%d,%d) 与 (%d,%d): %v",
		ErrSight, e.A.X, e.A.Y, e.B.X, e.B.Y, e.Err)
}

func (e *SightError) Unwrap() error { return e.Err }

func (e *SightError) Is(target error) bool {
	return target == ErrSight || (e.Err != nil && errors.Is(e.Err, target))
}

var dirs8 = [...]Point{
	{1, 0}, {-1, 0}, {0, 1}, {0, -1},
	{1, 1}, {1, -1}, {-1, 1}, {-1, -1},
}

type fnode struct {
	p   Point
	g   float64
	f   float64
	seq int
}

type fpq []*fnode

func (q fpq) Len() int { return len(q) }
func (q fpq) Less(i, j int) bool {
	if q[i].f != q[j].f {
		return q[i].f < q[j].f
	}
	return q[i].seq < q[j].seq
}
func (q fpq) Swap(i, j int) { q[i], q[j] = q[j], q[i] }
func (q *fpq) Push(x any)   { *q = append(*q, x.(*fnode)) }
func (q *fpq) Pop() any {
	old := *q
	n := old[len(old)-1]
	*q = old[:len(old)-1]
	return n
}

func dist(a, b Point) float64 {
	return math.Hypot(float64(a.X-b.X), float64(a.Y-b.Y))
}

// anyAngleH 是任意角寻路的启发式：n 到目标的欧氏直线距离。
// 可采纳性见 PathAnyAngle 的说明。
func anyAngleH(n, to Point) float64 { return dist(n, to) }

func octile(a, b Point) float64 {
	dx := abs(a.X - b.X)
	dy := abs(a.Y - b.Y)
	lo := dx
	if dy < lo {
		lo = dy
	}
	hi := dx
	if dy > hi {
		hi = dy
	}
	return float64(hi-lo) + math.Sqrt2*float64(lo)
}

func (g *Grid) requireWalkable(p Point, role string, blocked error) error {
	ok, err := g.Walkable(p)
	if err != nil {
		return fmt.Errorf("%w: %s (%d,%d)", err, role, p.X, p.Y)
	}
	if !ok {
		return fmt.Errorf("%w: %s (%d,%d)", blocked, role, p.X, p.Y)
	}
	return nil
}

// checkSight 把 Visible 的错误补上两端坐标；墙阻挡仍返回 (false, nil)。
func (g *Grid) checkSight(a, b Point) (bool, error) {
	ok, err := g.Visible(a, b)
	if err != nil {
		return false, &SightError{A: a, B: b, Err: err}
	}
	return ok, nil
}

// validatePath 逐段复核折线是否都能直达，失败时指出具体坐标。
func (g *Grid) validatePath(path []Point) error {
	for i := 1; i < len(path); i++ {
		ok, err := g.checkSight(path[i-1], path[i])
		if err != nil {
			return err
		}
		if !ok {
			return &SightError{A: path[i-1], B: path[i]}
		}
	}
	return nil
}

func rebuildPath(parent map[Point]Point, from, to Point) []Point {
	rev := []Point{to}
	for at := to; at != from; {
		at = parent[at]
		rev = append(rev, at)
	}
	for i, j := 0, len(rev)-1; i < j; i, j = i+1, j-1 {
		rev[i], rev[j] = rev[j], rev[i]
	}
	return rev
}

func polylineLength(path []Point) float64 {
	var total float64
	for i := 1; i < len(path); i++ {
		total += dist(path[i-1], path[i])
	}
	return total
}

// PathAnyAngle 做任意角寻路（Theta*）：相邻格只负责推进搜索，真正落点可沿
// 视线直达任一祖先，因此输出是一串可以跨格直行的折线，而不是逐格锯齿。
//
// 每段代价取两端点的欧氏距离，g 值用 float64 连续更新。返回的每一段都经过
// Visible 复核，确保“每一步都直接可达”；任何一段不满足都会返回 *SightError，
// 并带上该段两个端点的坐标。
//
// 启发式 h 取 n 到目标的欧氏直线距离。每步代价正是两点的欧氏距离，所以任意
// 一条 n→目标的折线长度都 ≥ 直线距离（三角不等式）；障碍只会删边、不会让
// 路径变短。因此 h 不会高估真实剩余代价，是可采纳的。又因
// |h(a)-h(b)| ≤ dist(a,b) = 边代价，h 同时是一致的。
//
// Theta* 对每个邻居同时尝试 path1（从父节点直视邻居）与 path2（逐格落到
// 邻居）。path2 完整包含八方向 A* 的转移，path1 由三角不等式只会更短，
// 故返回长度必不长于 Path8 的最优长度。
func (g *Grid) PathAnyAngle(from, to Point) ([]Point, float64, int, error) {
	if err := g.requireWalkable(from, "起点", ErrStartBlocked); err != nil {
		return nil, 0, 0, err
	}
	if err := g.requireWalkable(to, "目标", ErrGoalBlocked); err != nil {
		return nil, 0, 0, err
	}
	if from == to {
		return []Point{from}, 0, 1, nil
	}

	gScore := map[Point]float64{from: 0}
	parent := map[Point]Point{}
	open := &fpq{}
	seq := 0
	heap.Push(open, &fnode{p: from, f: anyAngleH(from, to), seq: seq})
	seq++
	expanded := 0

	for open.Len() > 0 {
		cur := heap.Pop(open).(*fnode)
		if best, seen := gScore[cur.p]; !seen || cur.g > best+1e-12 {
			continue // 堆里的过期条目
		}
		expanded++
		if cur.p == to {
			path := rebuildPath(parent, from, to)
			if err := g.validatePath(path); err != nil {
				return nil, 0, expanded, err
			}
			return path, polylineLength(path), expanded, nil
		}

		p, ok := parent[cur.p]
		if !ok {
			p = cur.p // 起点没有父节点，path1 与 path2 重合
		}
		for _, d := range dirs8 {
			nb := Point{cur.p.X + d.X, cur.p.Y + d.Y}
			if walk, err := g.Walkable(nb); err != nil || !walk {
				continue
			}
			// path1：父节点能否直视邻居。
			losParent, err := g.checkSight(p, nb)
			if err != nil {
				return nil, 0, expanded, err
			}
			if losParent {
				if ng := gScore[p] + dist(p, nb); ng < scoreOf(gScore, nb) {
					gScore[nb] = ng
					parent[nb] = p
					heap.Push(open, &fnode{p: nb, g: ng, f: ng + anyAngleH(nb, to), seq: seq})
					seq++
				}
			}
			// path2：保底地逐格走到邻居，保证不劣于八方向 A*。
			losCur, err := g.checkSight(cur.p, nb)
			if err != nil {
				return nil, 0, expanded, err
			}
			if !losCur {
				continue
			}
			if ng := cur.g + dist(cur.p, nb); ng < scoreOf(gScore, nb) {
				gScore[nb] = ng
				parent[nb] = cur.p
				heap.Push(open, &fnode{p: nb, g: ng, f: ng + anyAngleH(nb, to), seq: seq})
				seq++
			}
		}
	}

	return nil, 0, expanded, fmt.Errorf(
		"%w: 目标 (%d,%d) 从起点 (%d,%d) 不可达",
		ErrNoPath, to.X, to.Y, from.X, from.Y)
}

func scoreOf(m map[Point]float64, p Point) float64 {
	if v, ok := m[p]; ok {
		return v
	}
	return math.Inf(1)
}

// Path8 是逐格八方向 A*，作为“路径不长于逐格解”的对照基线。
// 正交步代价 1，对角步代价 √2，启发式用八方向精确距离（octile）：
// 无遮挡时 h 恰等于真实剩余长度，有遮挡时只会更小，故可采纳且一致。
// 返回路径、折线欧氏长度和节点展开数。
func (g *Grid) Path8(from, to Point) ([]Point, float64, int, error) {
	if err := g.requireWalkable(from, "起点", ErrStartBlocked); err != nil {
		return nil, 0, 0, err
	}
	if err := g.requireWalkable(to, "目标", ErrGoalBlocked); err != nil {
		return nil, 0, 0, err
	}
	if from == to {
		return []Point{from}, 0, 1, nil
	}

	gScore := map[Point]float64{from: 0}
	parent := map[Point]Point{}
	open := &fpq{}
	seq := 0
	heap.Push(open, &fnode{p: from, f: octile(from, to), seq: seq})
	seq++
	expanded := 0

	for open.Len() > 0 {
		cur := heap.Pop(open).(*fnode)
		if best, seen := gScore[cur.p]; !seen || cur.g > best+1e-12 {
			continue
		}
		expanded++
		if cur.p == to {
			path := rebuildPath(parent, from, to)
			return path, polylineLength(path), expanded, nil
		}
		for _, d := range dirs8 {
			nb := Point{cur.p.X + d.X, cur.p.Y + d.Y}
			if walk, err := g.Walkable(nb); err != nil || !walk {
				continue
			}
			// 与任意角寻路使用同一套“可直达”语义，保证基线是可行解的子集。
			if los, err := g.checkSight(cur.p, nb); err != nil || !los {
				continue
			}
			step := 1.0
			if d.X != 0 && d.Y != 0 {
				step = math.Sqrt2
			}
			if ng := cur.g + step; ng < scoreOf(gScore, nb) {
				gScore[nb] = ng
				parent[nb] = cur.p
				heap.Push(open, &fnode{p: nb, g: ng, f: ng + octile(nb, to), seq: seq})
				seq++
			}
		}
	}

	return nil, 0, expanded, fmt.Errorf(
		"%w: 目标 (%d,%d) 从起点 (%d,%d) 不可达",
		ErrNoPath, to.X, to.Y, from.X, from.Y)
}
