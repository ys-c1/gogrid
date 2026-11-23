package grid

import "container/heap"

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
