package grid

import (
	"errors"
	"fmt"
	"math/rand"
	"strings"
	"testing"
)

// 地形约定：'.' 普通地权重 1，'g' 草地权重 2，'m' 泥地权重 5，'#' 墙体不可走。
func newWeightedGrid(t *testing.T, rows []string, weights map[byte]int) *Grid {
	t.Helper()
	g, err := Parse(rows)
	if err != nil {
		t.Fatalf("解析失败: %v", err)
	}
	for y, row := range rows {
		for x, ch := range []byte(row) {
			if w, ok := weights[ch]; ok {
				if err := g.SetWeight(Point{x, y}, w); err != nil {
					t.Fatalf("设置权重失败 %s: %v", Point{x, y}, err)
				}
			}
		}
	}
	return g
}

func terrainWeights() map[byte]int {
	return map[byte]int{'.': 1, 'g': 2, 'm': 5}
}

// 1. 泥地：直穿泥地代价高，代价场路径应绕行。
func TestCostMudDetour(t *testing.T) {
	// 5x5，中间一竖列泥地；从 (0,2) 到 (4,2)。
	rows := []string{
		".....",
		".....",
		".mmm.",
		".....",
		".....",
	}
	g := newWeightedGrid(t, rows, terrainWeights())
	f, err := NewCostField(g, Point{0, 2})
	if err != nil {
		t.Fatal(err)
	}
	// 直穿：进入 3 个泥格 5*3 + 1 个普通格 = 16
	direct := 5*3 + 1
	// 绕行：上/下绕共 6 步（2 步让开泥列 + 4 步横向），每步权重 1 = 6
	detour := 6
	cost, err := f.CostAt(Point{4, 2})
	if err != nil {
		t.Fatal(err)
	}
	if cost != detour {
		t.Fatalf("泥地应绕行，期望代价 %d，实际 %d（直穿代价 %d）", detour, cost, direct)
	}
	path, pathCost, err := f.PathTo(Point{4, 2})
	if err != nil {
		t.Fatal(err)
	}
	if pathCost != cost {
		t.Fatalf("路径代价 %d 与代价场 %d 不一致", pathCost, cost)
	}
	for _, p := range path {
		if w, _ := g.Weight(p); w == 5 {
			t.Fatalf("路径穿过泥地 %s", p)
		}
	}
}

// 2. 草地：权重 2，短距离穿草与长距离绕行的权衡。
func TestCostGrass(t *testing.T) {
	// 3x3，中间横排草地；从 (0,1) 到 (2,1)。
	rows := []string{
		"...",
		".g.",
		"...",
	}
	g := newWeightedGrid(t, rows, terrainWeights())
	f, err := NewCostField(g, Point{0, 1})
	if err != nil {
		t.Fatal(err)
	}
	// 直穿：草(2)+普通(1)=3；绕行上/下：4 个普通格 = 4，故应直穿。
	cost, _ := f.CostAt(Point{2, 1})
	if cost != 3 {
		t.Fatalf("草地场景期望代价 3，实际 %d", cost)
	}
	// 把草地加重到 5，则应绕行（代价 4）。
	if _, err := f.UpdateWeight(Point{1, 1}, 5); err != nil {
		t.Fatal(err)
	}
	cost, _ = f.CostAt(Point{2, 1})
	if cost != 4 {
		t.Fatalf("草地加重后应绕行，期望代价 4，实际 %d", cost)
	}
}

// 3. 混合地形：泥地 + 草地 + 普通地，校验全图若干格代价。
func TestCostMixed(t *testing.T) {
	rows := []string{
		".g.",
		"m#m",
		"...",
	}
	g := newWeightedGrid(t, rows, terrainWeights())
	f, err := NewCostField(g, Point{0, 0})
	if err != nil {
		t.Fatal(err)
	}
	want := map[Point]int{
		{0, 0}: 0, // 源点
		{1, 0}: 2, // 草地
		{2, 0}: 3, // 经草地
		{0, 1}: 5, // 泥地
		{0, 2}: 6, // 经泥地后普通格
		{1, 2}: 7,
		{2, 2}: 8,
		{2, 1}: 9, // (2,2)->(2,1) 泥地：8+5=13? 下方断言以实际最短为准
	}
	// 单独精确核算 (2,1)：路径 (0,0)->(1,0)2 ->(2,0)3 -> 下泥 (2,1)=3+5=8；
	// 或经下方 (2,2)=8 再上泥=13。最短为 8。
	want[Point{2, 1}] = 8
	for p, w := range want {
		got, err := f.CostAt(p)
		if err != nil {
			t.Fatal(err)
		}
		if got != w {
			t.Fatalf("混合地形 %s 期望代价 %d，实际 %d", p, w, got)
		}
	}
}

// 4. 无路：目标被墙封死，返回带坐标的错误，代价为不可达。
func TestCostNoPath(t *testing.T) {
	rows := []string{
		".#.",
		".#.",
		".#.",
	}
	g, _ := Parse(rows)
	f, err := NewCostField(g, Point{0, 0})
	if err != nil {
		t.Fatal(err)
	}
	cost, err := f.CostAt(Point{2, 0})
	if err != nil {
		t.Fatal(err)
	}
	if cost != inf {
		t.Fatalf("不可达格代价应为 %d，实际 %d", inf, cost)
	}
	if _, _, err := f.PathTo(Point{2, 0}); err == nil {
		t.Fatal("无路应报错")
	} else if !errors.Is(err, ErrNoPath) {
		t.Fatalf("错误类型应为 ErrNoPath，实际 %v", err)
	}
}

// 5. 起终点相同：路径只含源点，代价 0。
func TestCostSameSourceTarget(t *testing.T) {
	g, _ := Parse([]string{"...", "..."})
	f, err := NewCostField(g, Point{1, 1})
	if err != nil {
		t.Fatal(err)
	}
	path, cost, err := f.PathTo(Point{1, 1})
	if err != nil {
		t.Fatal(err)
	}
	if len(path) != 1 || path[0] != (Point{1, 1}) || cost != 0 {
		t.Fatalf("起终点相同异常: path=%v cost=%d", path, cost)
	}
}

// 6. 等权路径行为不变：全权重 1 时代价场路径与旧 Path 逐格一致（长度、代价、路径点）。
func TestEqualWeightUnchanged(t *testing.T) {
	rows := []string{
		"...",
		".#.",
		"...",
	}
	g, _ := Parse(rows)
	from, to := Point{0, 0}, Point{2, 2}
	oldPath, _, err := g.Path(from, to)
	if err != nil {
		t.Fatal(err)
	}
	f, err := NewCostField(g, from)
	if err != nil {
		t.Fatal(err)
	}
	newPath, newCost, err := f.PathTo(to)
	if err != nil {
		t.Fatal(err)
	}
	if len(newPath) != len(oldPath) {
		t.Fatalf("等权路径长度变化: 旧 %d 新 %d", len(oldPath), len(newPath))
	}
	// 等权（全 1）时，代价 = 路径边数 = 旧路径点数 - 1，行为与单位步 A* 一致。
	if newCost != len(newPath)-1 {
		t.Fatalf("等权代价应等于路径边数: 代价 %d 路径点数 %d", newCost, len(newPath))
	}
	if newCost != len(oldPath)-1 {
		t.Fatalf("等权代价应等于旧路径边数: 旧路径点数 %d 新代价 %d", len(oldPath), newCost)
	}
	// 旧路径上每个格点权重都应为 1（等权前提）。
	for _, p := range oldPath {
		if w, _ := g.Weight(p); w != 1 {
			t.Fatalf("等权前提被破坏: %s 权重 %d", p, w)
		}
	}
	for i := range oldPath {
		if oldPath[i] != newPath[i] {
			t.Fatalf("等权路径第 %d 格变化: 旧 %s 新 %s", i, oldPath[i], newPath[i])
		}
	}
}

// 7. 错误输入：负权重、空网格、终点不可走，均返回带坐标的错误。
func TestCostErrors(t *testing.T) {
	g, _ := Parse([]string{".#.", "..."})

	// 负权重
	err := g.SetWeight(Point{0, 0}, -3)
	if !errors.Is(err, ErrNegativeWeight) {
		t.Fatalf("负权重应报 ErrNegativeWeight，实际 %v", err)
	}
	if err == nil || !contains(err.Error(), "(0,0)") {
		t.Fatalf("负权重错误应带坐标，实际 %v", err)
	}

	// 空网格
	var empty *Grid
	if _, err := NewCostField(empty, Point{0, 0}); !errors.Is(err, ErrEmpty) {
		t.Fatalf("空网格应报 ErrEmpty，实际 %v", err)
	}
	empty2 := &Grid{}
	if err := empty2.SetWeight(Point{0, 0}, 1); !errors.Is(err, ErrEmpty) {
		t.Fatalf("空网格 SetWeight 应报 ErrEmpty，实际 %v", err)
	}

	// 对墙体设权重（在建场之前直接对网格操作）
	if err := g.SetWeight(Point{1, 0}, 2); !errors.Is(err, ErrBlocked) {
		t.Fatalf("墙体设权重应报 ErrBlocked，实际 %v", err)
	}

	// 终点不可走（墙体）
	f, err := NewCostField(g, Point{0, 0})
	if err != nil {
		t.Fatal(err)
	}
	if _, _, errPath := f.PathTo(Point{1, 0}); !errors.Is(errPath, ErrBlocked) {
		t.Fatalf("终点不可走应报 ErrBlocked，实际 %v", errPath)
	} else if !contains(errPath.Error(), "(1,0)") {
		t.Fatalf("终点不可走错误应带坐标，实际 %v", errPath)
	}
	// 源点不可走
	if _, err := NewCostField(g, Point{1, 0}); !errors.Is(err, ErrBlocked) {
		t.Fatalf("源点不可走应报 ErrBlocked，实际 %v", err)
	}
	// 越界带坐标
	if _, err := f.CostAt(Point{9, 9}); !errors.Is(err, ErrOOB) {
		t.Fatalf("越界应报 ErrOOB，实际 %v", err)
	}
	if _, err := f.UpdateWeight(Point{9, 9}, 2); !errors.Is(err, ErrOOB) {
		t.Fatalf("增量更新越界应报 ErrOOB，实际 %v", err)
	}
}

func contains(s, sub string) bool {
	return strings.Contains(s, sub)
}

// 8. 增量与全量逐格相同：上升、下降两种变更，全图逐格比对 dist 与 parent。
func TestIncrementalMatchesFull(t *testing.T) {
	rows := []string{
		".....",
		".###.",
		"...m.",
		".###.",
		".....",
	}
	g := newWeightedGrid(t, rows, terrainWeights())
	src := Point{0, 0}

	cases := []struct {
		name string
		p    Point
		newW int
	}{
		{"权重上升", Point{3, 2}, 9},
		{"权重下降", Point{3, 2}, 1},
		{"普通格上升", Point{4, 0}, 7},
		{"普通格下降", Point{4, 0}, 1},
		{"泥地降为草地", Point{3, 2}, 2},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			// 每次从同一初始状态出发。
			inc, _ := NewCostField(g.Clone(), src)
			full, _ := NewCostField(g.Clone(), src)

			stats, err := inc.UpdateWeight(tc.p, tc.newW)
			if err != nil {
				t.Fatal(err)
			}
			fullGrid := full.g
			_ = fullGrid.SetWeight(tc.p, tc.newW)
			fullN := full.Recompute()

			assertFieldsEqual(t, inc, full, tc.name)
			if stats.Recomputed > fullN {
				t.Fatalf("%s: 增量重算 %d 格多于全量 %d 格", tc.name, stats.Recomputed, fullN)
			}
			t.Logf("%s: 变更点%s 权重->%d，增量重算 %d 格 / 全量 %d 格，边界格 %v",
				tc.name, tc.p, tc.newW, stats.Recomputed, fullN, stats.Boundary)

			// 边界格必须在受影响区域外缘，且距离与全量结果一致（未被重算）。
			affected := map[Point]bool{}
			for _, ap := range stats.Affected {
				affected[ap] = true
			}
			for _, b := range stats.Boundary {
				if affected[b] {
					t.Fatalf("%s: 边界格 %s 不应在受影响区域内", tc.name, b)
				}
				ib, _ := inc.CostAt(b)
				fb, _ := full.CostAt(b)
				if ib != fb {
					t.Fatalf("%s: 边界格 %s 代价不一致 %d vs %d", tc.name, b, ib, fb)
				}
				// 边界格必须与至少一个受影响格相邻。
				if !adjacentToAffected(g, b, affected) {
					t.Fatalf("%s: 边界格 %s 与受影响区域不相邻", tc.name, b)
				}
			}
		})
	}
}

func adjacentToAffected(g *Grid, b Point, affected map[Point]bool) bool {
	for _, d := range [4]Point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}} {
		if affected[Point{b.X + d.X, b.Y + d.Y}] {
			return true
		}
	}
	return false
}

// 9. 边界格单独核验 + 回退场景：无法封死区域时 FellBack=true 且结果仍与全量一致。
func TestBoundaryAndFallback(t *testing.T) {
	rows := []string{
		"...",
		"...",
		"...",
	}
	g, _ := Parse(rows)
	f, err := NewCostField(g, Point{0, 0})
	if err != nil {
		t.Fatal(err)
	}
	// 提升 (1,1)：受影响区域是其前驱子树，边界格应被单独列出且可独立核验。
	stats, err := f.UpdateWeight(Point{1, 1}, 4)
	if err != nil {
		t.Fatal(err)
	}
	if len(stats.Boundary) == 0 {
		t.Fatal("边界格不应为空")
	}
	t.Logf("边界格单独列出: %v", stats.Boundary)
	// 全量对照
	full, _ := NewCostField(g.Clone(), Point{0, 0})
	_ = full.g.SetWeight(Point{1, 1}, 4)
	full.Recompute()
	assertFieldsEqual(t, f, full, "边界核验")

	// 回退：越界变更通过 UpdateOrRecompute 触发全量重算。
	stats2, err := f.UpdateOrRecompute(Point{99, 99}, 3)
	if err == nil {
		t.Fatal("越界应返回错误")
	}
	if !stats2.FellBack {
		t.Fatal("越界时应回退全量重算")
	}
	if stats2.Recomputed != g.WalkableCount() {
		t.Fatalf("回退全量应重算全部可走格 %d，实际 %d", g.WalkableCount(), stats2.Recomputed)
	}
	t.Logf("越界回退全量重算 %d 格，错误: %v", stats2.Recomputed, err)
}

// 10. 随机网格随机变更：上升/下降各跑多轮，增量结果与全量逐格相同。
func TestRandomIncrementalConsistency(t *testing.T) {
	rng := rand.New(rand.NewSource(20261124))
	for round := 0; round < 200; round++ {
		w, h := 4+rng.Intn(7), 4+rng.Intn(7)
		rows := make([]string, h)
		for y := 0; y < h; y++ {
			b := make([]byte, w)
			for x := 0; x < w; x++ {
				if rng.Intn(100) < 22 {
					b[x] = '#'
				} else {
					switch rng.Intn(3) {
					case 0:
						b[x] = 'm'
					case 1:
						b[x] = 'g'
					default:
						b[x] = '.'
					}
				}
			}
			rows[y] = string(b)
		}
		g := newWeightedGrid(t, rows, terrainWeights())
		// 找一个可走源点。
		var src Point
		found := false
		for y := 0; y < h && !found; y++ {
			for x := 0; x < w; x++ {
				if g.cells[y][x] {
					src, found = Point{x, y}, true
					break
				}
			}
		}
		if !found {
			continue
		}
		inc, err := NewCostField(g.Clone(), src)
		if err != nil {
			t.Fatal(err)
		}
		full, err := NewCostField(g.Clone(), src)
		if err != nil {
			t.Fatal(err)
		}
		// 连续做 5 次随机权重变更，每次都与全量对照。
		for step := 0; step < 5; step++ {
			var p Point
			for {
				p = Point{rng.Intn(w), rng.Intn(h)}
				if g.cells[p.Y][p.X] {
					break
				}
			}
			nw := rng.Intn(9) + 1
			stats, err := inc.UpdateWeight(p, nw)
			if err != nil {
				t.Fatalf("round %d step %d: %v", round, step, err)
			}
			_ = full.g.SetWeight(p, nw)
			full.Recompute()
			assertFieldsEqual(t, inc, full, fmt.Sprintf("random round %d step %d", round, step))
			if stats.Recomputed > g.WalkableCount() {
				t.Fatalf("round %d: 增量重算格数 %d 超过全图可走格 %d", round, stats.Recomputed, g.WalkableCount())
			}
		}
	}
}

func assertFieldsEqual(t *testing.T, inc, full *CostField, label string) {
	t.Helper()
	for y := 0; y < inc.g.H; y++ {
		for x := 0; x < inc.g.W; x++ {
			if inc.dist[y][x] != full.dist[y][x] {
				t.Fatalf("%s: %s 代价不一致 增量=%d 全量=%d",
					label, Point{x, y}, inc.dist[y][x], full.dist[y][x])
			}
			if inc.parent[y][x] != full.parent[y][x] {
				t.Fatalf("%s: %s 前驱不一致 增量=%d 全量=%d",
					label, Point{x, y}, inc.parent[y][x], full.parent[y][x])
			}
		}
	}
}

// 基准：两档网格上报全量与增量的耗时与重算格数（go test -bench）。
func benchGrid(size int) *Grid {
	rows := make([]string, size)
	for y := 0; y < size; y++ {
		b := make([]byte, size)
		for x := 0; x < size; x++ {
			// 中间横一堵墙（右端留口），保证非平凡。
			if y == size/2 && x != size-1 {
				b[x] = '#'
			} else {
				b[x] = '.'
			}
		}
		rows[y] = string(b)
	}
	g, _ := Parse(rows)
	for y := size / 3; y < 2*size/3; y++ {
		for x := size / 3; x < 2*size/3; x++ {
			if g.cells[y][x] {
				_ = g.SetWeight(Point{x, y}, 5)
			}
		}
	}
	return g
}

var benchSink int

func BenchmarkFullSmall(b *testing.B) {
	benchFull(b, 20)
}

func BenchmarkIncrementalSmall(b *testing.B) {
	benchIncremental(b, 20)
}

func BenchmarkFullLarge(b *testing.B) {
	benchFull(b, 100)
}

func BenchmarkIncrementalLarge(b *testing.B) {
	benchIncremental(b, 100)
}

func benchFull(b *testing.B, size int) {
	g := benchGrid(size)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		f, _ := NewCostField(g.Clone(), Point{0, 0})
		benchSink = f.dist[0][0]
	}
}

func benchIncremental(b *testing.B, size int) {
	g := benchGrid(size)
	f, _ := NewCostField(g, Point{0, 0})
	p := Point{size / 3, size / 4} // 泥地边缘格
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		w := 1 + (i % 8)
		stats, err := f.UpdateWeight(p, w)
		if err != nil {
			b.Fatal(err)
		}
		benchSink = stats.Recomputed
	}
}
