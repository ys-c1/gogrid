package grid

import (
	"errors"
	"math"
	"strings"
	"testing"
)

// 用例地图。观察点用坐标系表示，地图本身只含 '.' 与 '#'。
var castMaps = map[string][]string{
	// 单墙：一块 3 格长的独立墙段，光在墙后应留下一道楔形阴影。
	"单墙": {
		"..........",
		"..........",
		"....###...",
		"..........",
		"..........",
	},
	// 走廊：1 格宽的长廊，观察点在中间，光只能沿走廊前后跑。
	"走廊": {
		"#######",
		"#.....#",
		"#.....#",
		"#.....#",
		"#######",
	},
	// 柱状：开阔地上的一根方柱，光绕过四周。
	"柱状": {
		".........",
		".........",
		"....#....",
		".........",
		".........",
	},
	// 角落：观察点贴着 L 形内角，光被两面墙裁掉一半。
	"角落": {
		"......",
		"......",
		"..#...",
		".##...",
		"......",
	},
	// 全遮挡：观察点正前方一整排墙，除自身外无可见格。
	"全遮挡": {
		".....",
		"#####",
		".....",
	},
}

// 测试内独立重写的 Bresenham 逐格视线（不调用 Visible），
// 用作投射结果的权威对照，避免与被测代码共用同一实现。
func testBresenhamVisible(g *Grid, a, b Point) bool {
	x0, y0 := a.X, a.Y
	x1, y1 := b.X, b.Y
	dx, dy := x1-x0, y1-y0
	if dx < 0 {
		dx = -dx
	}
	if dy < 0 {
		dy = -dy
	}
	sx, sy := 1, 1
	if x0 > x1 {
		sx = -1
	}
	if y0 > y1 {
		sy = -1
	}
	err := dx - dy
	for {
		if !g.cells[y0][x0] {
			return false
		}
		if x0 == x1 && y0 == y1 {
			return true
		}
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			x0 += sx
		}
		if e2 < dx {
			err += dx
			y0 += sy
		}
	}
}

func testPointSet(pts []Point) map[Point]bool {
	m := make(map[Point]bool, len(pts))
	for _, p := range pts {
		m[p] = true
	}
	return m
}

// testPerCellSet 独立枚举扇形几何内的可走格并重算逐格视线。
func testPerCellSet(t *testing.T, g *Grid, cfg CastConfig) map[Point]bool {
	t.Helper()
	out := make(map[Point]bool)
	r2 := cfg.Radius * cfg.Radius
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if !g.cells[y][x] {
				continue
			}
			dx, dy := x-cfg.Origin.X, y-cfg.Origin.Y
			if dx*dx+dy*dy > r2 {
				continue
			}
			if cfg.AngleDeg < 360 && (dx != 0 || dy != 0) {
				deg := math.Atan2(float64(dy), float64(dx)) * 180 / math.Pi
				d := math.Mod(deg-cfg.CenterDeg+180, 360)
				if d < 0 {
					d += 360
				}
				d -= 180
				if math.Abs(d) > cfg.AngleDeg/2+1e-9 {
					continue
				}
			}
			if testBresenhamVisible(g, cfg.Origin, Point{X: x, Y: y}) {
				out[Point{X: x, Y: y}] = true
			}
		}
	}
	return out
}

func sameSet(a, b map[Point]bool) bool {
	if len(a) != len(b) {
		return false
	}
	for p := range a {
		if !b[p] {
			return false
		}
	}
	return true
}

// TestCastValidation 覆盖张角为零、半径越界、网格为空三类输入，
// 并断言错误信息点出具体是哪个量。
func TestCastValidation(t *testing.T) {
	g, err := Parse(castMaps["柱状"])
	if err != nil {
		t.Fatal(err)
	}
	origin := Point{X: 0, Y: 0}
	base := CastConfig{Origin: origin, CenterDeg: 0, AngleDeg: 90, Radius: 3}

	cases := []struct {
		name  string
		grid  *Grid
		mut   func(*CastConfig)
		want  error
		param string
	}{
		{"张角为零", g, func(c *CastConfig) { c.AngleDeg = 0 }, ErrAngle, "angle"},
		{"张角为负", g, func(c *CastConfig) { c.AngleDeg = -30 }, ErrAngle, "angle"},
		{"张角超 360", g, func(c *CastConfig) { c.AngleDeg = 360.5 }, ErrAngle, "angle"},
		{"张角 NaN", g, func(c *CastConfig) { c.AngleDeg = math.NaN() }, ErrAngle, "angle"},
		{"张角 Inf", g, func(c *CastConfig) { c.AngleDeg = math.Inf(1) }, ErrAngle, "angle"},
		{"中线非有限", g, func(c *CastConfig) { c.CenterDeg = math.NaN() }, ErrAngle, "angle"},
		{"半径为零", g, func(c *CastConfig) { c.Radius = 0 }, ErrRadius, "radius"},
		{"半径为负", g, func(c *CastConfig) { c.Radius = -2 }, ErrRadius, "radius"},
		{"半径越界", g, func(c *CastConfig) { c.Radius = g.MaxRadius() + 1 }, ErrRadius, "radius"},
		{"网格为空", nil, func(*CastConfig) {}, ErrGrid, "grid"},
		{"零尺寸网格", &Grid{}, func(*CastConfig) {}, ErrGrid, "grid"},
		{"观察点越界", g, func(c *CastConfig) { c.Origin = Point{X: -1, Y: 0} }, ErrOrigin, "origin"},
		{"观察点是墙", g, func(c *CastConfig) { c.Origin = Point{X: 4, Y: 2} }, ErrOrigin, "origin"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			cfg := base
			tc.mut(&cfg)
			_, err := tc.grid.Project(cfg)
			if err == nil {
				t.Fatalf("期望报错，实际通过")
			}
			if !errors.Is(err, tc.want) {
				t.Fatalf("错误类型 %v，期望 %v", err, tc.want)
			}
			if !strings.Contains(err.Error(), tc.param) {
				t.Fatalf("错误信息未点出参数名 %q：%s", tc.param, err)
			}
			if tc.grid != nil {
				if _, perErr := tc.grid.PerCellVisible(cfg); !errors.Is(perErr, tc.want) {
					t.Fatalf("PerCellVisible 错误口径不一致：%v", perErr)
				}
			}
		})
	}

	// 边界合法值不得报错。
	for _, cfg := range []CastConfig{
		{Origin: origin, CenterDeg: 0, AngleDeg: 360, Radius: g.MaxRadius()},
		{Origin: origin, CenterDeg: 271.5, AngleDeg: 0.001, Radius: 1},
	} {
		if _, err := g.Project(cfg); err != nil {
			t.Fatalf("合法参数 %+v 报错：%v", cfg, err)
		}
	}
}

// TestVisibleAPINUnchanged 确认原有逐格可见性接口行为不变。
func TestVisibleAPIUnchanged(t *testing.T) {
	g, _ := Parse([]string{"...", ".#.", "..."})
	if ok, err := g.Visible(Point{0, 0}, Point{1, 0}); err != nil || !ok {
		t.Fatalf("相邻可见：ok=%v err=%v", ok, err)
	}
	if ok, err := g.Visible(Point{1, 0}, Point{1, 2}); err != nil || ok {
		t.Fatalf("穿过墙体应不可见：ok=%v err=%v", ok, err)
	}
	if _, err := g.Visible(Point{-1, 0}, Point{0, 0}); !errors.Is(err, ErrOOB) {
		t.Fatalf("越界应返回 ErrOOB，实际 %v", err)
	}
}

// TestCastScenarios 逐场景核验两件事，且分开验：
//   - 可见集与逐格射线全等（CheckConsistency + 测试内独立重算）；
//   - 扫掠射线不跨墙（CheckNoWallCrossing）。
//
// 同时断言射线数不超过反解值。
func TestCastScenarios(t *testing.T) {
	type origin struct {
		name string
		at   Point
	}
	cases := []struct {
		mapName string
		origins []origin
		full    bool // 全遮挡：360° 也只有观察点自身
	}{
		{"单墙", []origin{{"西侧", Point{0, 2}}, {"墙后", Point{7, 2}}, {"上方", Point{5, 0}}}, false},
		{"走廊", []origin{{"中部", Point{3, 2}}, {"墙角", Point{1, 1}}}, false},
		{"柱状", []origin{{"正对", Point{4, 0}}, {"侧面", Point{0, 2}}, {"贴柱", Point{3, 2}}}, false},
		{"角落", []origin{{"内角", Point{1, 2}}, {"角外", Point{0, 0}}}, false},
		{"全遮挡", []origin{{"正对", Point{2, 2}}}, true},
	}

	angles := []float64{30, 90, 180, 270, 360}
	centers := []float64{0, 45, 90, 180, 270}

	for _, tc := range cases {
		g, err := Parse(castMaps[tc.mapName])
		if err != nil {
			t.Fatalf("%s: %v", tc.mapName, err)
		}
		for _, o := range tc.origins {
			for _, angle := range angles {
				for _, center := range centers {
					for radius := 1; radius <= g.MaxRadius(); radius++ {
						cfg := CastConfig{Origin: o.at, CenterDeg: center, AngleDeg: angle, Radius: radius}
						p, err := g.Project(cfg)
						if err != nil {
							t.Fatalf("%s/%s：%v", tc.mapName, o.name, err)
						}

						// 第一项：与逐格射线逐格全等，单独核验。
						if err := p.CheckConsistency(); err != nil {
							t.Fatalf("%s/%s %+v：%v", tc.mapName, o.name, cfg, err)
						}
						want := testPerCellSet(t, g, cfg)
						if !sameSet(testPointSet(p.Visible()), want) {
							t.Fatalf("%s/%s %+v：可见集与独立重算不一致\n投射=%v\n基准=%v",
								tc.mapName, o.name, cfg, p.Visible(), want)
						}

						// 第二项：扫掠射线不跨墙，单独核验。
						if err := p.CheckNoWallCrossing(); err != nil {
							t.Fatalf("%s/%s %+v：%v", tc.mapName, o.name, cfg, err)
						}

						// 射线数：实际不得超过由张角反解的值。
						if err := p.CheckBudget(); err != nil {
							t.Fatalf("%s/%s %+v：%v", tc.mapName, o.name, cfg, err)
						}
						if p.RaysUsed() > RayBudget(angle, radius) {
							t.Fatalf("%s/%s：实际 %d 条 > 反解 %d 条",
								tc.mapName, o.name, p.RaysUsed(), RayBudget(angle, radius))
						}

						// 距离场规模与可见集一致。
						if p.RangeField().Size() != len(p.Visible()) {
							t.Fatalf("%s/%s：距离场规模 %d != 可见格 %d",
								tc.mapName, o.name, p.RangeField().Size(), len(p.Visible()))
						}
						for _, c := range p.Visible() {
							if _, ok := p.RangeField().Dist(c); !ok {
								t.Fatalf("%s/%s：可见格 %v 不在距离场中", tc.mapName, o.name, c)
							}
						}

						// 全遮挡场景：横墙之后的整排格永远不可见。
						if tc.full {
							for _, c := range p.Visible() {
								if c.Y < 1 {
									t.Fatalf("%s/%s：墙后格 %v 不应可见", tc.mapName, o.name, c)
								}
							}
						}

						// 观察点必在可见集内，且墙格不得出现。
						if !testPointSet(p.Visible())[o.at] {
							t.Fatalf("%s/%s：可见集不含观察点", tc.mapName, o.name)
						}
						for _, c := range p.Visible() {
							if !g.cells[c.Y][c.X] {
								t.Fatalf("%s/%s：可见集含墙格 %v", tc.mapName, o.name, c)
							}
						}
					}
				}
			}
		}
	}
}

// TestOccludedExplanation 断言每个被遮挡格都能给出挡墙，且墙确实在逐格射线上。
func TestOccludedExplanation(t *testing.T) {
	g, _ := Parse(castMaps["单墙"])
	p, err := g.Project(CastConfig{Origin: Point{3, 0}, CenterDeg: 90, AngleDeg: 90, Radius: 5})
	if err != nil {
		t.Fatal(err)
	}
	occluded := p.Occluded()
	if len(occluded) == 0 {
		t.Fatal("单墙场景应有被遮挡格")
	}
	for _, oc := range occluded {
		if g.cells[oc.Cell.Y][oc.Cell.X] {
			// Cell 本身是可走格才对。
		} else {
			t.Fatalf("被遮挡格 %v 不应是墙", oc.Cell)
		}
		if g.cells[oc.Wall.Y][oc.Wall.X] {
			t.Fatalf("挡墙 %v 不是墙格", oc.Wall)
		}
		wall, found := g.firstWallOnLine(Point{3, 0}, oc.Cell)
		if !found || wall != oc.Wall {
			t.Fatalf("被遮挡格 %v 的挡墙 %v 与逐格射线 %v 不符", oc.Cell, oc.Wall, wall)
		}
	}
}

// TestSweepAgreementReported 断言原始扫掠的一致率如实报告，且回退次数
// 等于原始扫掠与逐格射线的对称差。
func TestSweepAgreementReported(t *testing.T) {
	g, _ := Parse(castMaps["柱状"])
	p, err := g.Project(CastConfig{Origin: Point{4, 0}, CenterDeg: 90, AngleDeg: 360, Radius: 6})
	if err != nil {
		t.Fatal(err)
	}
	st := p.Stats()
	if st.Rays != st.Budget {
		t.Fatalf("实际射线数 %d 应等于反解值 %d", st.Rays, st.Budget)
	}
	if st.Fallback != st.Extra+st.Missing {
		t.Fatalf("回退 %d != 多出 %d + 漏掉 %d", st.Fallback, st.Extra, st.Missing)
	}
	if st.Fallback != len(p.FallbackCells()) {
		t.Fatalf("回退次数 %d 与回退格列表 %d 不一致", st.Fallback, len(p.FallbackCells()))
	}
	if st.RawAgreement <= 0 || st.RawAgreement > 1 {
		t.Fatalf("一致率越界：%v", st.RawAgreement)
	}
	// 最终可见集与逐格射线全等，所以最终一致率必须为 1。
	if len(p.Visible()) > 0 && st.Matched != st.Cells {
		t.Fatalf("匹配 %d != 最终可见 %d", st.Matched, st.Cells)
	}
}

// TestRayBudget 检查反解公式的单调性与边界。
func TestRayBudget(t *testing.T) {
	if got := RayBudget(360, 1); got != int(math.Ceil(2*math.Pi*1.5)) {
		t.Fatalf("360°/R=1 反解 %d", got)
	}
	if got := RayBudget(90, 4); got != int(math.Ceil(math.Pi/2*4.5)) {
		t.Fatalf("90°/R=4 反解 %d", got)
	}
	prev := 0
	for _, angle := range []float64{1, 10, 45, 90, 180, 360} {
		cur := RayBudget(angle, 6)
		if cur < prev {
			t.Fatalf("反解未随张角单调不减：%v 时 %d < %d", angle, cur, prev)
		}
		prev = cur
	}
	for _, bad := range []float64{0, -1, math.NaN()} {
		if got := RayBudget(bad, 3); got != 0 {
			t.Fatalf("非法张角 %v 的反解应为 0，实际 %d", bad, got)
		}
	}
}

// TestNoDiagonalSlip 核验射线不得从两堵墙的对角缝隙里斜穿过去。
// 地图中央有一处只有对角接触的墙缝，观察点在对角一侧。
func TestNoDiagonalSlip(t *testing.T) {
	g, err := Parse([]string{
		"......",
		"..#...",
		"...#..",
		"......",
		"......",
	})
	if err != nil {
		t.Fatal(err)
	}
	cfg := CastConfig{Origin: Point{0, 0}, CenterDeg: 45, AngleDeg: 360, Radius: 5}
	p, err := g.Project(cfg)
	if err != nil {
		t.Fatal(err)
	}
	if err := p.CheckNoWallCrossing(); err != nil {
		t.Fatal(err)
	}
	if err := p.CheckConsistency(); err != nil {
		t.Fatal(err)
	}
	// 墙缝另一侧 (4,3) 与 (3,4) 在逐格射线下都应不可见，最终可见集也不得含它们。
	for _, behind := range []Point{{4, 3}, {3, 4}, {5, 4}, {4, 5}} {
		if ok, err := g.Walkable(behind); err != nil || !ok {
			continue
		}
		ok, err := g.Visible(cfg.Origin, behind)
		if err != nil {
			t.Fatal(err)
		}
		if ok {
			continue // 该图里若本就可见，跳过（不是本用例要断言的格）。
		}
		if testPointSet(p.Visible())[behind] {
			t.Fatalf("墙缝后的格 %v 不应出现在最终可见集", behind)
		}
	}
	// 任何射线的格序列里，墙格之后不得再有格（撞墙即停）。
	for i, ray := range p.Rays() {
		seenWall := false
		for _, c := range ray.Cells {
			if !g.cells[c.Y][c.X] {
				if seenWall {
					t.Fatalf("第 %d 条射线跨过墙体", i)
				}
				seenWall = true
			}
		}
	}
}

// TestProjectDeterministic 同一配置重复投射必须给出完全相同的格集与射线。
func TestProjectDeterministic(t *testing.T) {
	g, _ := Parse(castMaps["角落"])
	cfg := CastConfig{Origin: Point{1, 2}, CenterDeg: 45, AngleDeg: 180, Radius: 5}
	a, err := g.Project(cfg)
	if err != nil {
		t.Fatal(err)
	}
	b, err := g.Project(cfg)
	if err != nil {
		t.Fatal(err)
	}
	av, bv := a.Visible(), b.Visible()
	if len(av) != len(bv) {
		t.Fatalf("两次投射可见格数不同：%d vs %d", len(av), len(bv))
	}
	for i := range av {
		if av[i] != bv[i] {
			t.Fatalf("第 %d 个可见格不同：%v vs %v", i, av[i], bv[i])
		}
	}
	if a.Fallbacks() != b.Fallbacks() {
		t.Fatalf("两次投射回退次数不同：%d vs %d", a.Fallbacks(), b.Fallbacks())
	}
}

// TestFallbackCountsBothDirections 在一个确实出现不一致的配置上，
// 核验回退计数与 Extra/Missing 两个方向自洽。
func TestFallbackCountsBothDirections(t *testing.T) {
	// 角落里放一面斜墙，掠射方向容易出现多算或漏算；遍历多个配置直到
	// 至少出现一次不一致，从而真正覆盖 Extra/Missing 两个方向。
	g, _ := Parse(castMaps["角落"])
	var p *Projection
	var cfg CastConfig
	found := false
	for _, center := range []float64{0, 30, 45, 60, 90} {
		for _, angle := range []float64{45, 90, 135, 180, 360} {
			for radius := 2; radius <= 5 && !found; radius++ {
				c := CastConfig{Origin: Point{1, 2}, CenterDeg: center, AngleDeg: angle, Radius: radius}
				q, err := g.Project(c)
				if err != nil {
					t.Fatal(err)
				}
				if q.Fallbacks() > 0 {
					p, cfg, found = q, c, true
				}
			}
			if found {
				break
			}
		}
		if found {
			break
		}
	}
	if !found {
		t.Fatal("角落里没有找到任何扫掠与逐格射线不一致的配置")
	}
	_ = cfg
	st := p.Stats()
	if st.Fallback != st.Extra+st.Missing {
		t.Fatalf("回退 %d != 多出 %d + 漏掉 %d", st.Fallback, st.Extra, st.Missing)
	}
	// 多出的格必在原始扫掠里且不在最终可见集；漏掉的格反之。
	rawSet := testPointSet(p.RawVisible())
	visSet := testPointSet(p.Visible())
	for _, c := range p.FallbackCells() {
		inRaw, inVis := rawSet[c], visSet[c]
		if inRaw == inVis {
			t.Fatalf("回退格 %v 应恰好属于原始扫掠与最终可见之一", c)
		}
	}
}
