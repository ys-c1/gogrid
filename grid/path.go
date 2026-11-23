package grid

import (
	"container/heap"
	"fmt"
)

type node struct {
	p    Point
	g, f int
	idx  int
}

type pq []*node

func (q pq) Len() int            { return len(q) }
func (q pq) Less(i, j int) bool  { return q[i].f < q[j].f }
func (q pq) Swap(i, j int)       { q[i], q[j] = q[j], q[i] }
func (q *pq) Push(x interface{}) { *q = append(*q, x.(*node)) }
func (q *pq) Pop() interface{} {
	old := *q
	n := old[len(old)-1]
	*q = old[:len(old)-1]
	return n
}

func abs(a int) int {
	if a < 0 {
		return -a
	}
	return a
}

// astar 四邻域 A* 内核，调用方负责端点可走性与同点校验。
// 签名与行为保持与原始实现一致（曼哈顿启发、visited 计数）。
func (g *Grid) astar(from, to Point) ([]Point, int, error) {
	open := &pq{}
	heap.Init(open)
	best := map[Point]int{from: 0}
	prev := map[Point]Point{}
	heap.Push(open, &node{p: from, g: 0, f: abs(from.X-to.X) + abs(from.Y-to.Y)})
	visited := 0
	for open.Len() > 0 {
		cur := heap.Pop(open).(*node)
		visited++
		if cur.p == to {
			var path []Point
			for at := to; ; at = prev[at] {
				path = append([]Point{at}, path...)
				if at == from {
					break
				}
			}
			return path, visited, nil
		}
		dirs := []Point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}
		for _, d := range dirs {
			nx := Point{cur.p.X + d.X, cur.p.Y + d.Y}
			ok, err := g.Walkable(nx)
			if err != nil || !ok {
				continue
			}
			ng := cur.g + 1
			if old, seen := best[nx]; seen && old <= ng {
				continue
			}
			best[nx] = ng
			prev[nx] = cur.p
			h := abs(nx.X-to.X) + abs(nx.Y-to.Y)
			heap.Push(open, &node{p: nx, g: ng, f: ng + h})
		}
	}
	return nil, visited, ErrNoPath
}

// Path 原始接口保持不变：返回含起终点的格点路径、扩展节点数与错误。
func (g *Grid) Path(from, to Point) ([]Point, int, error) {
	if ok, err := g.Walkable(from); err != nil || !ok {
		if err != nil {
			return nil, 0, err
		}
		return nil, 0, ErrNoPath
	}
	if ok, err := g.Walkable(to); err != nil || !ok {
		if err != nil {
			return nil, 0, err
		}
		return nil, 0, ErrNoPath
	}
	return g.astar(from, to)
}

// PathVia 带必经点的寻路：按 vias 顺序逐段 A*，拼接后返回。
// 端点与每个必经点都必须可走，否则返回带坐标的 *BlockedPointError；
// 相邻点重合返回 *SamePointError；某两段之间无路返回 *NoPathError。
func (g *Grid) PathVia(vias ...Point) ([]Point, int, error) {
	if len(vias) < 2 {
		return nil, 0, fmt.Errorf("至少需要起点和终点两个点")
	}
	// 先做全部点位校验，无路/同点之前先暴露不可走与越界。
	for _, v := range vias {
		ok, err := g.Walkable(v)
		if err != nil {
			return nil, 0, &BlockedPointError{Point: v, OutOfBounds: true}
		}
		if !ok {
			return nil, 0, &BlockedPointError{Point: v}
		}
	}
	for i := 1; i < len(vias); i++ {
		if vias[i] == vias[i-1] {
			return nil, 0, &SamePointError{Point: vias[i]}
		}
	}
	var full []Point
	visited := 0
	for i := 1; i < len(vias); i++ {
		seg, n, err := g.astar(vias[i-1], vias[i])
		visited += n
		if err != nil {
			return nil, visited, &NoPathError{From: vias[i-1], To: vias[i]}
		}
		if full == nil {
			full = seg
		} else {
			full = append(full, seg[1:]...) // 段间共用一个必经点，去重
		}
	}
	return full, visited, nil
}
