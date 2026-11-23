package grid

import (
	"errors"
	"fmt"
)

// Rule 选择连通判定所用的邻域。
type Rule int

const (
	Rule4 Rule = 4 // 四邻：上、下、左、右
	Rule8 Rule = 8 // 八邻：四邻 + 四个对角
)

// ErrBadRule 表示连通规则不是 Rule4 或 Rule8。
var ErrBadRule = errors.New("非法连通规则")

func (r Rule) String() string {
	switch r {
	case Rule4:
		return "四邻"
	case Rule8:
		return "八邻"
	default:
		return fmt.Sprintf("Rule(%d)", int(r))
	}
}

func (r Rule) valid() bool { return r == Rule4 || r == Rule8 }

// CoordError 附带坐标的错误。空网格、越界等与位置有关的查询都以它返回。
type CoordError struct {
	Err error
	P   Point
}

func (e *CoordError) Error() string {
	return fmt.Sprintf("%v: 坐标 (%d,%d)", e.Err, e.P.X, e.P.Y)
}

func (e *CoordError) Unwrap() error { return e.Err }

// 只取“前向”边（右、下、右下、左下），每条相邻边恰好合并一次。
// 这样并查集的合并结果与扫描顺序无关。
var edges4 = [...]Point{{1, 0}, {0, 1}}
var edges8 = [...]Point{{1, 0}, {0, 1}, {1, 1}, {-1, 1}}

// RegionMap 是连通的区域标记结果。
// 可走格为 1..Count，墙体为 0。
// 编号按区域在行优先扫描中首次出现的顺序分配，因此与内部处理顺序无关。
// 字段在构造后不会被实现内部再修改，可被并发读取。
type RegionMap struct {
	Rule    Rule
	W, H    int
	Count   int   // 区域数
	MaxArea int   // 最大区域面积（格子数）
	Areas   []int // 各区域面积，Areas[id-1] 对应编号 id
	labels  []int
}

// Components 用“一遍扫描 + 并查集”给全部可走格编号。
// 返回的区域编号是稳定的：同一区域只会有一个编号，且编号只由区域在网格中
// 首次出现的行优先位置决定，不受合并/遍历顺序影响。
func (g *Grid) Components(rule Rule) (*RegionMap, error) {
	return g.componentsInOrder(rule, nil)
}

// componentsInOrder 允许测试传入一个格子索引的排列来打乱扫描顺序。
// order 为 nil 时按行优先顺序扫描；否则必须是 0..W*H-1 的一个排列。
func (g *Grid) componentsInOrder(rule Rule, order []int) (*RegionMap, error) {
	// 网格级错误没有自然的格子位置，统一以原点 (0,0) 报出。
	if g == nil || g.W <= 0 || g.H <= 0 {
		return nil, &CoordError{Err: ErrEmpty, P: Point{0, 0}}
	}
	if !rule.valid() {
		return nil, &CoordError{Err: ErrBadRule, P: Point{0, 0}}
	}

	n := g.W * g.H
	if len(g.cells) < g.H {
		return nil, &CoordError{Err: ErrEmpty, P: Point{0, 0}}
	}
	for y := 0; y < g.H; y++ {
		if len(g.cells[y]) < g.W {
			return nil, &CoordError{Err: ErrShape, P: Point{0, y}}
		}
	}

	d := newDSU(n)
	edges := edges4[:]
	if rule == Rule8 {
		edges = edges8[:]
	}

	if order == nil {
		order = make([]int, n)
		for i := range order {
			order[i] = i
		}
	}
	walkable := func(i int) bool {
		return g.cells[i/g.W][i%g.W]
	}
	for _, i := range order {
		if i < 0 || i >= n || !walkable(i) {
			continue
		}
		x, y := i%g.W, i/g.W
		for _, e := range edges {
			nx, ny := x+e.X, y+e.Y
			if nx < 0 || ny < 0 || nx >= g.W || ny >= g.H {
				continue
			}
			ni := ny*g.W + nx
			if walkable(ni) {
				d.union(i, ni)
			}
		}
	}

	m := &RegionMap{Rule: rule, W: g.W, H: g.H, labels: make([]int, n)}
	areas := []int{0} // 1-based，areas[0] 占位
	rootID := make([]int, n)
	for i := 0; i < n; i++ {
		if !walkable(i) {
			continue
		}
		r := d.find(i)
		id := rootID[r]
		if id == 0 {
			id = len(areas)
			rootID[r] = id
			areas = append(areas, 0)
		}
		m.labels[i] = id
		areas[id]++
	}

	m.Areas = areas[1:]
	m.Count = len(m.Areas)
	for _, a := range m.Areas {
		if a > m.MaxArea {
			m.MaxArea = a
		}
	}
	return m, nil
}

// Labels 返回标记数组的副本，下标为 y*W+x；墙体为 0，可走格为 1..Count。
func (m *RegionMap) Labels() []int {
	if m == nil {
		return nil
	}
	out := make([]int, len(m.labels))
	copy(out, m.labels)
	return out
}

// LabelAt 返回某格的区域编号：可走格为 1..Count，墙体为 0。
// 空网格与越界都会返回带坐标的 CoordError。
func (m *RegionMap) LabelAt(p Point) (int, error) {
	if m == nil || m.W <= 0 || m.H <= 0 || len(m.labels) != m.W*m.H {
		return 0, &CoordError{Err: ErrEmpty, P: p}
	}
	if p.X < 0 || p.Y < 0 || p.X >= m.W || p.Y >= m.H {
		return 0, &CoordError{Err: ErrOOB, P: p}
	}
	return m.labels[p.Y*m.W+p.X], nil
}

type dsu struct {
	parent []int
	size   []int
}

func newDSU(n int) *dsu {
	d := &dsu{parent: make([]int, n), size: make([]int, n)}
	for i := 0; i < n; i++ {
		d.parent[i] = i
		d.size[i] = 1
	}
	return d
}

func (d *dsu) find(x int) int {
	for d.parent[x] != x {
		d.parent[x] = d.parent[d.parent[x]] // 路径折半
		x = d.parent[x]
	}
	return x
}

func (d *dsu) union(a, b int) {
	ra, rb := d.find(a), d.find(b)
	if ra == rb {
		return
	}
	if d.size[ra] < d.size[rb] {
		ra, rb = rb, ra
	}
	d.parent[rb] = ra
	d.size[ra] += d.size[rb]
}
