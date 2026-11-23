package grid

import (
	"errors"
	"fmt"
	"math/rand"
	"testing"
)

// refComponents 是逐格广度优先的参照实现，独立于并查集实现。
// 返回每个可走格所属的参照区域编号（从 1 开始），墙体为 0。
func refComponents(t *testing.T, g *Grid, rule Rule) ([]int, int, int) {
	t.Helper()
	ref := make([]int, g.W*g.H)
	dirs := []Point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}
	if rule == Rule8 {
		dirs = append(dirs, Point{1, 1}, Point{1, -1}, Point{-1, 1}, Point{-1, -1})
	}
	next := 0
	areas := map[int]int{}
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			start := Point{x, y}
			if !g.cells[y][x] || ref[y*g.W+x] != 0 {
				continue
			}
			next++
			queue := []Point{start}
			ref[y*g.W+x] = next
			for len(queue) > 0 {
				cur := queue[0]
				queue = queue[1:]
				for _, d := range dirs {
					n := Point{cur.X + d.X, cur.Y + d.Y}
					if n.X < 0 || n.Y < 0 || n.X >= g.W || n.Y >= g.H {
						continue
					}
					ni := n.Y*g.W + n.X
					if g.cells[n.Y][n.X] && ref[ni] == 0 {
						ref[ni] = next
						queue = append(queue, n)
					}
				}
			}
		}
	}
	max := 0
	for _, id := range ref {
		if id != 0 {
			areas[id]++
		}
	}
	for _, a := range areas {
		if a > max {
			max = a
		}
	}
	return ref, next, max
}

// assertParity 校验两条不变量：
// 1) 标记覆盖全部可走格且编号落在 1..Count；
// 2) 同一区域只有一个标记 —— 标记划分与逐格 BFS 的参照划分完全一致。
func assertParity(t *testing.T, g *Grid, m *RegionMap, ref []int, refCount, refMax int) {
	t.Helper()
	if m.Count != refCount {
		t.Fatalf("%v 区域数 %d，参照 %d", m.Rule, m.Count, refCount)
	}
	if m.MaxArea != refMax {
		t.Fatalf("%v 最大面积 %d，参照 %d", m.Rule, m.MaxArea, refMax)
	}
	refToLabel := map[int]int{}
	labelToRef := map[int]int{}
	labels := m.Labels()
	if len(labels) != g.W*g.H {
		t.Fatalf("标记长度 %d，应为 %d", len(labels), g.W*g.H)
	}
	for i, lb := range labels {
		x, y := i%g.W, i/g.W
		walkable := g.cells[y][x]
		if !walkable {
			if lb != 0 {
				t.Fatalf("墙格 (%d,%d) 标记应为 0，实际 %d", x, y, lb)
			}
			if ref[i] != 0 {
				t.Fatalf("参照在墙格 (%d,%d) 有区域", x, y)
			}
			continue
		}
		// 不变量 1：覆盖 + 编号范围。
		if lb < 1 || lb > m.Count {
			t.Fatalf("可走格 (%d,%d) 标记 %d 越出 1..%d", x, y, lb, m.Count)
		}
		// 不变量 2：与 BFS 划分一一对应。
		if prev, ok := refToLabel[ref[i]]; ok && prev != lb {
			t.Fatalf("参照区域 %d 在 (%d,%d) 标记 %d，之前为 %d", ref[i], x, y, lb, prev)
		}
		if prev, ok := labelToRef[lb]; ok && prev != ref[i] {
			t.Fatalf("标记 %d 在 (%d,%d) 属于参照区域 %d，之前为 %d", lb, x, y, ref[i], prev)
		}
		refToLabel[ref[i]] = lb
		labelToRef[lb] = ref[i]
	}
	for id, a := range m.Areas {
		if a < 1 {
			t.Fatalf("区域 %d 面积为 %d", id+1, a)
		}
	}
	sum := 0
	for _, a := range m.Areas {
		sum += a
	}
	if sum != g.WalkableCount() {
		t.Fatalf("面积和 %d，可走格 %d", sum, g.WalkableCount())
	}
}

func mustParse(t *testing.T, rows []string) *Grid {
	t.Helper()
	g, err := Parse(rows)
	if err != nil {
		t.Fatal(err)
	}
	return g
}

func TestComponentsSingleRegion(t *testing.T) {
	g := mustParse(t, []string{"...", ".#.", "..."})
	for _, rule := range []Rule{Rule4, Rule8} {
		m, err := g.Components(rule)
		if err != nil {
			t.Fatal(err)
		}
		if m.Count != 1 || m.MaxArea != 8 {
			t.Fatalf("%v 应为单区 8 格，实际 %d 区最大 %d", rule, m.Count, m.MaxArea)
		}
		ref, rc, rm := refComponents(t, g, rule)
		assertParity(t, g, m, ref, rc, rm)
	}
}

func TestComponentsMultipleRegions(t *testing.T) {
	g := mustParse(t, []string{
		"..#...#",
		"..#...#",
		"#######",
		".......",
		"...#...",
	})
	for _, rule := range []Rule{Rule4, Rule8} {
		m, err := g.Components(rule)
		if err != nil {
			t.Fatal(err)
		}
		ref, rc, rm := refComponents(t, g, rule)
		assertParity(t, g, m, ref, rc, rm)
		// 贯通墙把网格切成上左、上右、下方三块，两种规则都跨不过墙。
		if m.Count != 3 || m.MaxArea != 13 {
			t.Fatalf("%v 应为 3 区最大 13 格，实际 %d 区最大 %d", rule, m.Count, m.MaxArea)
		}
	}
}

func TestComponentsDiagonal(t *testing.T) {
	g := mustParse(t, []string{".#.", "#..", ".#."})
	m4, err := g.Components(Rule4)
	if err != nil {
		t.Fatal(err)
	}
	if m4.Count != 3 || m4.MaxArea != 4 { // 中心、左、右各一区，最大 4 格
		t.Fatalf("四邻应为 3 区最大 4 格，实际 %d 区最大 %d", m4.Count, m4.MaxArea)
	}
	m8, err := g.Components(Rule8)
	if err != nil {
		t.Fatal(err)
	}
	if m8.Count != 1 || m8.MaxArea != 6 {
		t.Fatalf("八邻应为 1 区 6 格，实际 %d 区最大 %d", m8.Count, m8.MaxArea)
	}
	for _, m := range []*RegionMap{m4, m8} {
		ref, rc, rm := refComponents(t, g, m.Rule)
		assertParity(t, g, m, ref, rc, rm)
	}
}

func TestComponentsAllWalls(t *testing.T) {
	g := mustParse(t, []string{"###", "###"})
	for _, rule := range []Rule{Rule4, Rule8} {
		m, err := g.Components(rule)
		if err != nil {
			t.Fatal(err)
		}
		if m.Count != 0 || m.MaxArea != 0 {
			t.Fatalf("%v 全墙应为 0 区，实际 %d 区最大 %d", rule, m.Count, m.MaxArea)
		}
		for i, lb := range m.Labels() {
			if lb != 0 {
				t.Fatalf("全墙格 %d 标记 %d", i, lb)
			}
		}
	}
}

func TestComponentsAllOpen(t *testing.T) {
	g := mustParse(t, []string{"....", "....", "...."})
	for _, rule := range []Rule{Rule4, Rule8} {
		m, err := g.Components(rule)
		if err != nil {
			t.Fatal(err)
		}
		if m.Count != 1 || m.MaxArea != 12 {
			t.Fatalf("%v 全空应为 1 区 12 格，实际 %d 区最大 %d", rule, m.Count, m.MaxArea)
		}
		ref, rc, rm := refComponents(t, g, rule)
		assertParity(t, g, m, ref, rc, rm)
	}
}

func TestComponentsErrorsCarryCoords(t *testing.T) {
	// 空网格。
	var zero Grid
	_, err := zero.Components(Rule4)
	if !errors.Is(err, ErrEmpty) {
		t.Fatalf("空网格应报 ErrEmpty，实际 %v", err)
	}
	var ce *CoordError
	if !errors.As(err, &ce) {
		t.Fatalf("空网格错误应带坐标，实际 %T", err)
	}
	if ce.P != (Point{0, 0}) {
		t.Fatalf("空网格坐标应为 (0,0)，实际 %v", ce.P)
	}

	g := mustParse(t, []string{"...", "..."})

	// 越界：查询与标记两侧都要报出坐标。
	if _, err := g.Walkable(Point{-1, 2}); !errors.Is(err, ErrOOB) {
		t.Fatalf("越界应报 ErrOOB，实际 %v", err)
	} else if !errors.As(err, &ce) || ce.P != (Point{-1, 2}) {
		t.Fatalf("Walkable 越界坐标错误: %v", err)
	}
	m, err := g.Components(Rule4)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.LabelAt(Point{3, 0}); !errors.Is(err, ErrOOB) {
		t.Fatalf("LabelAt 越界应报 ErrOOB，实际 %v", err)
	} else if !errors.As(err, &ce) || ce.P != (Point{3, 0}) {
		t.Fatalf("LabelAt 越界坐标错误: %v", err)
	}
	if _, err := m.LabelAt(Point{0, 9}); !errors.As(err, &ce) || ce.P != (Point{0, 9}) {
		t.Fatalf("LabelAt 越界坐标错误: %v", err)
	}

	// 规则非法。
	_, err = g.Components(Rule(6))
	if !errors.Is(err, ErrBadRule) {
		t.Fatalf("非法规则应报 ErrBadRule，实际 %v", err)
	}
	if !errors.As(err, &ce) {
		t.Fatalf("非法规则错误应带坐标，实际 %v", err)
	}

	// 行宽不一致也带坐标。
	if _, err := Parse([]string{"..", "..."}); !errors.As(err, &ce) || ce.P.Y != 1 {
		t.Fatalf("行宽错误坐标不对: %v", err)
	}
}

// TestComponentsScale 在规模化网格上验证两条不变量，并与逐格 BFS 对照。
func TestComponentsScale(t *testing.T) {
	if testing.Short() {
		t.Skip("规模化用例")
	}
	rng := rand.New(rand.NewSource(20261124))
	for _, size := range []int{3, 41, 160} {
		rows := make([]string, size)
		for y := 0; y < size; y++ {
			b := make([]byte, size)
			for x := 0; x < size; x++ {
				if rng.Intn(100) < 35 {
					b[x] = '#'
				} else {
					b[x] = '.'
				}
			}
			rows[y] = string(b)
		}
		g := mustParse(t, rows)
		for _, rule := range []Rule{Rule4, Rule8} {
			m, err := g.Components(rule)
			if err != nil {
				t.Fatal(err)
			}
			ref, rc, rm := refComponents(t, g, rule)
			assertParity(t, g, m, ref, rc, rm)
			if got := len(m.Areas); got != m.Count {
				t.Fatalf("%dx%d %v 面积表 %d 项，区域数 %d", size, size, rule, got, m.Count)
			}
			t.Logf("%dx%d %v: 区域数=%d 最大面积=%d", size, size, rule, m.Count, m.MaxArea)
		}
	}
}

// TestComponentsShuffleStable 要求打乱扫描顺序后区域编号不变。
func TestComponentsShuffleStable(t *testing.T) {
	g := mustParse(t, []string{
		"..#...#",
		"#..#..#",
		"...#...",
		"##.#.##",
		"...#...",
		"#.#...#",
	})
	for _, rule := range []Rule{Rule4, Rule8} {
		base, err := g.Components(rule)
		if err != nil {
			t.Fatal(err)
		}
		want := base.Labels()
		n := g.W * g.H
		orders := [][]int{make([]int, n)}
		for i := range orders[0] {
			orders[0][i] = n - 1 - i // 逆序
		}
		rng := rand.New(rand.NewSource(7))
		for k := 0; k < 25; k++ {
			o := make([]int, n)
			for i := range o {
				o[i] = i
			}
			rng.Shuffle(n, func(i, j int) { o[i], o[j] = o[j], o[i] })
			orders = append(orders, o)
		}
		for _, order := range orders {
			got, err := g.componentsInOrder(rule, order)
			if err != nil {
				t.Fatal(err)
			}
			if got.Count != base.Count || got.MaxArea != base.MaxArea {
				t.Fatalf("%v 打乱后区域统计变化: %d/%d vs %d/%d", rule,
					got.Count, got.MaxArea, base.Count, base.MaxArea)
			}
			if fmt.Sprint(got.Labels()) != fmt.Sprint(want) {
				t.Fatalf("%v 打乱后编号不稳定:\n got %v\nwant %v", rule, got.Labels(), want)
			}
			if fmt.Sprint(got.Areas) != fmt.Sprint(base.Areas) {
				t.Fatalf("%v 打乱后面积表不稳定: %v vs %v", rule, got.Areas, base.Areas)
			}
		}
	}
}

// TestPathUnaffected 确认调用连通标记前后寻路结果完全一致。
func TestPathUnaffected(t *testing.T) {
	g := mustParse(t, []string{
		"....#..",
		".##.#..",
		"....#..",
		"#.##...",
		"......#",
	})
	cases := []struct{ from, to Point }{
		{Point{0, 0}, Point{4, 4}},
		{Point{0, 0}, Point{2, 4}},
		{Point{6, 4}, Point{0, 0}},
		{Point{5, 0}, Point{1, 3}},
	}
	type result struct {
		path    []Point
		visited int
		err     error
	}
	run := func() []result {
		out := make([]result, len(cases))
		for i, c := range cases {
			p, v, err := g.Path(c.from, c.to)
			out[i] = result{p, v, err}
		}
		return out
	}
	before := run()
	for _, rule := range []Rule{Rule4, Rule8} {
		if _, err := g.Components(rule); err != nil {
			t.Fatal(err)
		}
	}
	after := run()
	for i := range cases {
		if fmt.Sprint(before[i].path) != fmt.Sprint(after[i].path) ||
			before[i].visited != after[i].visited ||
			fmt.Sprint(before[i].err) != fmt.Sprint(after[i].err) {
			t.Fatalf("用例 %d 寻路结果被改变: %+v vs %+v", i, before[i], after[i])
		}
	}
	if before[1].err != nil && before[1].path != nil {
		t.Fatal("不可达时应无路径")
	}
}
