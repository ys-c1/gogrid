package grid

import (
	"errors"
	"math"
	"math/rand"
	"strings"
	"testing"
)

type scenario struct {
	name string
	rows []string
	from Point
	to   Point
}

func scenarios() []scenario {
	return []scenario{
		{
			name: "开阔",
			rows: []string{
				"............",
				"............",
				"............",
				"............",
				"............",
				"............",
				"............",
				"............",
				"............",
				"............",
				"............",
				"............",
			},
			from: Point{0, 0}, to: Point{11, 11},
		},
		{
			name: "走廊",
			rows: []string{
				"#########",
				"#.......#",
				"#######.#",
				"#.......#",
				"#.#######",
				"#.......#",
				"#########",
			},
			from: Point{1, 1}, to: Point{7, 5},
		},
		{
			name: "拐角",
			rows: []string{
				"..........",
				"..........",
				"..........",
				"..........",
				"..........",
				"#########.",
				"..........",
				"..........",
				"..........",
				"..........",
			},
			from: Point{0, 0}, to: Point{9, 9},
		},
		{
			name: "障碍群",
			rows: []string{
				"............",
				".###..###...",
				".###..###...",
				"............",
				"...####.....",
				"...####.....",
				".###........",
				".###..###...",
				"......###...",
				"............",
				"###.........",
				"............",
			},
			from: Point{0, 0}, to: Point{11, 11},
		},
		{
			name: "对角",
			rows: []string{
				"........",
				"..##....",
				"..##....",
				"........",
				"....##..",
				"....##..",
				"........",
				"........",
			},
			from: Point{0, 0}, to: Point{7, 7},
		},
	}
}

func mustParse(t *testing.T, rows []string) *Grid {
	t.Helper()
	g, err := Parse(rows)
	if err != nil {
		t.Fatalf("解析失败: %v", err)
	}
	return g
}

func assertSegmentsVisible(t *testing.T, g *Grid, path []Point) {
	t.Helper()
	if len(path) < 2 {
		t.Fatalf("路径至少应有起点和终点，实际 %d 点", len(path))
	}
	for i := 1; i < len(path); i++ {
		ok, err := g.Visible(path[i-1], path[i])
		if err != nil {
			t.Fatalf("第 %d 段 (%d,%d)->(%d,%d) 视线报错: %v",
				i, path[i-1].X, path[i-1].Y, path[i].X, path[i].Y, err)
		}
		if !ok {
			t.Fatalf("第 %d 段 (%d,%d)->(%d,%d) 不可直达",
				i, path[i-1].X, path[i-1].Y, path[i].X, path[i].Y)
		}
	}
}

func assertPathEndpoints(t *testing.T, path []Point, from, to Point) {
	t.Helper()
	if len(path) == 0 || path[0] != from || path[len(path)-1] != to {
		t.Fatalf("端点错误: 期望 %v -> %v，实际 %d 点", from, to, len(path))
	}
}

// 验证一（可直达）：任意角解里的每一段都必须是视线直达的。
func TestAnyAngleEveryStepDirect(t *testing.T) {
	for _, sc := range scenarios() {
		t.Run(sc.name, func(t *testing.T) {
			g := mustParse(t, sc.rows)
			path, _, _, err := g.PathAnyAngle(sc.from, sc.to)
			if err != nil {
				t.Fatalf("寻路失败: %v", err)
			}
			assertPathEndpoints(t, path, sc.from, sc.to)
			assertSegmentsVisible(t, g, path)
		})
	}
}

// 验证二（长度上界）：任意角解不长于逐格八方向解，也不长于原逐格四方向解。
func TestAnyAngleNotLongerThanCellPaths(t *testing.T) {
	for _, sc := range scenarios() {
		t.Run(sc.name, func(t *testing.T) {
			g := mustParse(t, sc.rows)
			anyPath, anyLen, _, err := g.PathAnyAngle(sc.from, sc.to)
			if err != nil {
				t.Fatalf("任意角寻路失败: %v", err)
			}
			cellPath, cellLen, _, err := g.Path8(sc.from, sc.to)
			if err != nil {
				t.Fatalf("八方向寻路失败: %v", err)
			}
			assertSegmentsVisible(t, g, cellPath)
			if math.Abs(polylineLength(anyPath)-anyLen) > 1e-9 {
				t.Fatalf("返回长度 %v 与折线长度 %v 不一致", anyLen, polylineLength(anyPath))
			}
			if anyLen > cellLen+1e-9 {
				t.Fatalf("任意角长度 %v 超过逐格八方向长度 %v", anyLen, cellLen)
			}

			fourPath, _, err := g.Path(sc.from, sc.to)
			if err != nil {
				t.Fatalf("四方向寻路失败: %v", err)
			}
			if fourLen := polylineLength(fourPath); anyLen > fourLen+1e-9 {
				t.Fatalf("任意角长度 %v 超过逐格四方向长度 %v", anyLen, fourLen)
			}
		})
	}
}

// 任意角解应严格短于逐格解，并压成更少的折点（体现“跨越格子直达”）。
func TestAnyAngleBeatsCellPathInOpenSpace(t *testing.T) {
	g := mustParse(t, []string{
		"..........",
		"..........",
		"..........",
		"..........",
		"..........",
		"..........",
		"..........",
		"..........",
		"..........",
		"..........",
	})
	path, length, _, err := g.PathAnyAngle(Point{0, 0}, Point{9, 9})
	if err != nil {
		t.Fatal(err)
	}
	if len(path) != 2 {
		t.Fatalf("开阔对角应只需两个点，实际 %d: %v", len(path), path)
	}
	if want := 9 * math.Sqrt2; math.Abs(length-want) > 1e-9 {
		t.Fatalf("长度应为 %v，实际 %v", want, length)
	}

	cellPath, _, _, err := g.Path8(Point{0, 0}, Point{9, 9})
	if err != nil {
		t.Fatal(err)
	}
	fourPath, _, err := g.Path(Point{0, 0}, Point{9, 9})
	if err != nil {
		t.Fatal(err)
	}
	fourLen := polylineLength(fourPath)
	if !(length < fourLen) {
		t.Fatalf("开阔地任意角应短于逐格四方向解，%v vs %v", length, fourLen)
	}
	if len(path) >= len(fourPath) {
		t.Fatalf("任意角折点应少于四方向解，%d vs %d", len(path), len(fourPath))
	}
	if len(path) > len(cellPath) {
		t.Fatalf("任意角折点不应多于八方向解，%d vs %d", len(path), len(cellPath))
	}
}

func TestAnyAngleErrors(t *testing.T) {
	g := mustParse(t, []string{
		".#.",
		".#.",
		"...",
	})

	t.Run("起点在墙内", func(t *testing.T) {
		_, _, _, err := g.PathAnyAngle(Point{1, 0}, Point{0, 0})
		if !errors.Is(err, ErrStartBlocked) {
			t.Fatalf("应返回 ErrStartBlocked，实际 %v", err)
		}
		if !strings.Contains(err.Error(), "(1,0)") {
			t.Fatalf("错误信息应指出起点坐标，实际 %q", err)
		}
	})

	t.Run("目标在墙内", func(t *testing.T) {
		_, _, _, err := g.PathAnyAngle(Point{0, 0}, Point{1, 1})
		if !errors.Is(err, ErrGoalBlocked) {
			t.Fatalf("应返回 ErrGoalBlocked，实际 %v", err)
		}
		if !strings.Contains(err.Error(), "(1,1)") {
			t.Fatalf("错误信息应指出目标坐标，实际 %q", err)
		}
	})

	t.Run("目标不可达", func(t *testing.T) {
		g2 := mustParse(t, []string{
			".#.",
			".#.",
		})
		_, _, _, err := g2.PathAnyAngle(Point{0, 0}, Point{2, 0})
		if !errors.Is(err, ErrNoPath) {
			t.Fatalf("应返回 ErrNoPath，实际 %v", err)
		}
		if !strings.Contains(err.Error(), "(0,0)") || !strings.Contains(err.Error(), "(2,0)") {
			t.Fatalf("错误信息应指出两端坐标，实际 %q", err)
		}
	})

	t.Run("越界", func(t *testing.T) {
		_, _, _, err := g.PathAnyAngle(Point{-1, 0}, Point{0, 0})
		if !errors.Is(err, ErrOOB) {
			t.Fatalf("应返回 ErrOOB，实际 %v", err)
		}
		if !strings.Contains(err.Error(), "(-1,0)") {
			t.Fatalf("错误信息应指出坐标，实际 %q", err)
		}
	})

	t.Run("视线判定失败", func(t *testing.T) {
		g3 := mustParse(t, []string{
			"...",
			".#.",
			"...",
		})
		err := g3.validatePath([]Point{{0, 0}, {2, 2}})
		var se *SightError
		if !errors.As(err, &se) {
			t.Fatalf("应返回 *SightError，实际 %T %v", err, err)
		}
		if !errors.Is(err, ErrSight) {
			t.Fatalf("应匹配 ErrSight，实际 %v", err)
		}
		if se.A != (Point{0, 0}) || se.B != (Point{2, 2}) {
			t.Fatalf("应指出失败段的坐标，实际 (%d,%d)->(%d,%d)",
				se.A.X, se.A.Y, se.B.X, se.B.Y)
		}
		if !strings.Contains(err.Error(), "(0,0)") || !strings.Contains(err.Error(), "(2,2)") {
			t.Fatalf("错误信息应包含两个端点，实际 %q", err)
		}
	})

	t.Run("视线越界", func(t *testing.T) {
		_, err := g.checkSight(Point{0, 0}, Point{9, 0})
		if !errors.Is(err, ErrOOB) || !errors.Is(err, ErrSight) {
			t.Fatalf("应同时匹配 ErrSight 与 ErrOOB，实际 %v", err)
		}
		if !strings.Contains(err.Error(), "(9,0)") {
			t.Fatalf("错误信息应指出越界端点，实际 %q", err)
		}
	})
}

// 启发式可采纳性论证的数值对照。
// (1) 一致性：|h(a)-h(b)| ≤ dist(a,b)，等价于 h 的三角不等式；
// (2) 可采纳：h ≤ 真实最优剩余代价，用视线完全图上的 Dijkstra 暴力求最优。
func TestHeuristicAdmissible(t *testing.T) {
	maps := [][]string{
		{
			"........",
			".##.##..",
			".##.##..",
			"........",
			"..####..",
			"........",
			".##.....",
			"........",
		},
		{
			"...#....",
			".#.#.##.",
			".#...#..",
			".####.#.",
			"....#.#.",
			".##...#.",
			"........",
		},
		{
			"........",
			"........",
			"........",
			"........",
			"........",
			"........",
		},
	}
	for mi, rows := range maps {
		g := mustParse(t, rows)
		cells := walkableCells(g)
		goal := cells[len(cells)-1]

		// (1) 一致性：|h(a,goal)-h(b,goal)| ≤ dist(a,b)。
		for _, a := range cells {
			for _, b := range cells {
				lhs := math.Abs(h(a, goal) - h(b, goal))
				if lhs > dist(a, b)+1e-12 {
					t.Fatalf("地图 %d: |h(%v)-h(%v)|=%v > dist=%v，一致性失败",
						mi, a, b, lhs, dist(a, b))
				}
			}
		}

		// (2) 可采纳性：与暴力最优比较。
		for _, s := range cells {
			opt := optimalAnyAngleCost(g, s)
			for _, tt := range cells {
				v, reachable := opt[tt]
				if !reachable {
					continue
				}
				if h(s, tt) > v+1e-9 {
					t.Fatalf("地图 %d: 启发式高估，h(%v,%v)=%v > 最优 %v",
						mi, s, tt, h(s, tt), v)
				}
			}
		}
	}
}

// h 直接指向算法实际使用的启发式，避免测试复制一份而失去校验意义。
func h(from, to Point) float64 { return anyAngleH(from, to) }

func walkableCells(g *Grid) []Point {
	var cells []Point
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if g.cells[y][x] {
				cells = append(cells, Point{x, y})
			}
		}
	}
	return cells
}

// optimalAnyAngleCost 在“任意两个互相可见的可行走格”构成的完全图上跑
// Dijkstra，得到从 s 出发到各格的最短折线长度，作为真实最优代价。
func optimalAnyAngleCost(g *Grid, s Point) map[Point]float64 {
	cells := walkableCells(g)
	index := make(map[Point]int, len(cells))
	for i, c := range cells {
		index[c] = i
	}
	const inf = math.MaxFloat64
	best := make([]float64, len(cells))
	done := make([]bool, len(cells))
	for i := range best {
		best[i] = inf
	}
	best[index[s]] = 0
	for {
		u := -1
		for i := range cells {
			if !done[i] && best[i] < inf && (u == -1 || best[i] < best[u]) {
				u = i
			}
		}
		if u == -1 {
			break
		}
		done[u] = true
		for v := range cells {
			if done[v] || u == v {
				continue
			}
			if ok, err := g.Visible(cells[u], cells[v]); err != nil || !ok {
				continue
			}
			if nd := best[u] + dist(cells[u], cells[v]); nd < best[v] {
				best[v] = nd
			}
		}
	}
	out := make(map[Point]float64)
	for i, c := range cells {
		if best[i] < inf {
			out[c] = best[i]
		}
	}
	return out
}

// 返回长度与“视线完全图”上的最优做对照，并明确记录差距。
// Theta* 允许在可见祖先之间跳跃，但不保证全局最优；本用例只要求：
// 结果不劣于逐格八方向解（硬性），并把与全局最优的差距如实打印出来。
func TestAnyAngleComparedToOptimal(t *testing.T) {
	for _, rows := range [][]string{
		{
			"........",
			".##.##..",
			".##.##..",
			"........",
			"..####..",
			"........",
			".##.....",
			"........",
		},
		{
			"............",
			".###..###...",
			".###..###...",
			"............",
			"...####.....",
			"...####.....",
			".###........",
			".###..###...",
			"......###...",
			"............",
			"###.........",
			"............",
		},
	} {
		g := mustParse(t, rows)
		cells := walkableCells(g)
		worstGap := 0.0
		for _, s := range cells {
			opt := optimalAnyAngleCost(g, s)
			for _, tt := range cells {
				v, reachable := opt[tt]
				if !reachable || s == tt {
					continue
				}
				_, lenAA, _, err := g.PathAnyAngle(s, tt)
				if err != nil {
					t.Fatalf("(%v)->(%v) 寻路失败: %v", s, tt, err)
				}
				_, len8, _, err := g.Path8(s, tt)
				if err != nil {
					t.Fatalf("(%v)->(%v) 八方向失败: %v", s, tt, err)
				}
				if lenAA > len8+1e-9 {
					t.Fatalf("(%v)->(%v): 任意角 %v 超过八方向 %v", s, tt, lenAA, len8)
				}
				if gap := lenAA - v; gap > worstGap {
					worstGap = gap
				}
			}
		}
		t.Logf("地图最坏与全局最优差距: %.6f", worstGap)
	}
}

// 随机地图上同时验证两条性质：每段直达、长度不长于逐格八方向。
func TestAnyAngleRandomized(t *testing.T) {
	rng := rand.New(rand.NewSource(20261124))
	for iter := 0; iter < 200; iter++ {
		w, hgt := 6+rng.Intn(7), 6+rng.Intn(7)
		rows := make([]string, hgt)
		for y := range rows {
			b := make([]byte, w)
			for x := range b {
				if rng.Intn(100) < 30 {
					b[x] = '#'
				} else {
					b[x] = '.'
				}
			}
			rows[y] = string(b)
		}
		g, err := Parse(rows)
		if err != nil {
			t.Fatal(err)
		}
		cells := walkableCells(g)
		if len(cells) < 2 {
			continue
		}
		from := cells[rng.Intn(len(cells))]
		to := cells[rng.Intn(len(cells))]
		if from == to {
			continue
		}
		anyPath, anyLen, _, aaErr := g.PathAnyAngle(from, to)
		cellPath, cellLen, _, c8Err := g.Path8(from, to)
		if (aaErr == nil) != (c8Err == nil) {
			t.Fatalf("迭代 %d 可达性不一致: AA=%v Path8=%v\n%s",
				iter, aaErr, c8Err, strings.Join(rows, "\n"))
		}
		if aaErr != nil {
			continue
		}
		assertPathEndpoints(t, anyPath, from, to)
		assertSegmentsVisible(t, g, anyPath)
		assertSegmentsVisible(t, g, cellPath)
		if math.Abs(polylineLength(anyPath)-anyLen) > 1e-9 {
			t.Fatalf("迭代 %d 长度不一致", iter)
		}
		if anyLen > cellLen+1e-9 {
			t.Fatalf("迭代 %d 任意角 %v 长于逐格八方向 %v\n%s",
				iter, anyLen, cellLen, strings.Join(rows, "\n"))
		}
	}
}

// TestReport 输出各场景的长度与节点展开数，作为收尾统计。
func TestReport(t *testing.T) {
	t.Logf("%-8s %10s %10s %10s %8s %8s %8s",
		"场景", "任意角长度", "八方向长度", "四方向长度", "AA展开", "8展开", "4展开")
	for _, sc := range scenarios() {
		g := mustParse(t, sc.rows)
		_, aaLen, aaExp, err := g.PathAnyAngle(sc.from, sc.to)
		if err != nil {
			t.Fatalf("%s: %v", sc.name, err)
		}
		_, c8Len, c8Exp, err := g.Path8(sc.from, sc.to)
		if err != nil {
			t.Fatalf("%s: %v", sc.name, err)
		}
		c4Path, c4Exp, err := g.Path(sc.from, sc.to)
		if err != nil {
			t.Fatalf("%s: %v", sc.name, err)
		}
		t.Logf("%-8s %10.4f %10.4f %10.4f %8d %8d %8d",
			sc.name, aaLen, c8Len, polylineLength(c4Path),
			aaExp, c8Exp, c4Exp)
	}
}
