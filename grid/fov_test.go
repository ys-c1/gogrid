package grid

import (
	"errors"
	"math"
	"math/rand"
	"strings"
	"testing"
)

func mustParse(t *testing.T, rows []string) *Grid {
	t.Helper()
	g, err := Parse(rows)
	if err != nil {
		t.Fatalf("Parse: %v", err)
	}
	return g
}

func pointsSet(pts []Point) map[Point]bool {
	m := make(map[Point]bool, len(pts))
	for _, p := range pts {
		m[p] = true
	}
	return m
}

func walkablePoints(g *Grid) []Point {
	var out []Point
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if ok, _ := g.Walkable(Point{X: x, Y: y}); ok {
				out = append(out, Point{X: x, Y: y})
			}
		}
	}
	return out
}

// TestFOVValidation 覆盖张角为零、半径超界、朝向非法，以及观察点非法。
func TestFOVValidation(t *testing.T) {
	g := mustParse(t, []string{".....", "..#..", "....."})
	origin := Point{X: 0, Y: 0}
	base := FOVConfig{Origin: origin, Facing: East, AngleDeg: 90, Radius: 3}

	cases := []struct {
		name    string
		mutate  func(*FOVConfig)
		wantErr error
		param   string
	}{
		{"张角为零", func(c *FOVConfig) { c.AngleDeg = 0 }, ErrAngle, "angle"},
		{"张角为负", func(c *FOVConfig) { c.AngleDeg = -45 }, ErrAngle, "angle"},
		{"张角超 360", func(c *FOVConfig) { c.AngleDeg = 360.5 }, ErrAngle, "angle"},
		{"张角 NaN", func(c *FOVConfig) { c.AngleDeg = math.NaN() }, ErrAngle, "angle"},
		{"张角 Inf", func(c *FOVConfig) { c.AngleDeg = math.Inf(1) }, ErrAngle, "angle"},
		{"半径为零", func(c *FOVConfig) { c.Radius = 0 }, ErrRadius, "radius"},
		{"半径为负", func(c *FOVConfig) { c.Radius = -1 }, ErrRadius, "radius"},
		{"半径超界", func(c *FOVConfig) { c.Radius = g.MaxRadius() + 1 }, ErrRadius, "radius"},
		{"朝向越上界", func(c *FOVConfig) { c.Facing = Facing(4) }, ErrFacing, "facing"},
		{"朝向为负", func(c *FOVConfig) { c.Facing = Facing(-1) }, ErrFacing, "facing"},
		{"观察点是墙", func(c *FOVConfig) { c.Origin = Point{X: 2, Y: 1} }, ErrOrigin, "origin"},
		{"观察点越界", func(c *FOVConfig) { c.Origin = Point{X: -1, Y: 0} }, ErrOrigin, "origin"},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			cfg := base
			tc.mutate(&cfg)
			_, err := g.FOV(cfg)
			if err == nil {
				t.Fatalf("期望报错，实际通过")
			}
			if !errors.Is(err, tc.wantErr) {
				t.Fatalf("错误类型 %v，期望 %v", err, tc.wantErr)
			}
			if !strings.Contains(err.Error(), tc.param) {
				t.Fatalf("错误信息未带参数名 %q：%s", tc.param, err)
			}
			// 参数校验错误必须不产生可见格。
			if got, _ := g.FOV(cfg); got != nil {
				t.Fatalf("报错时仍返回可见格 %v", got)
			}
			// ValidateFOV 与 FOV 的错误口径一致。
			if verr := g.ValidateFOV(cfg); !errors.Is(verr, tc.wantErr) {
				t.Fatalf("ValidateFOV 错误 %v，期望 %v", verr, tc.wantErr)
			}
		})
	}

	// 边界合法值不得报错。
	for _, cfg := range []FOVConfig{
		{Origin: origin, Facing: North, AngleDeg: 360, Radius: g.MaxRadius()},
		{Origin: origin, Facing: West, AngleDeg: 0.001, Radius: 1},
	} {
		if _, err := g.FOV(cfg); err != nil {
			t.Fatalf("合法参数 %+v 报错：%v", cfg, err)
		}
	}
}

// TestFOVFullEqualsVisibleSet 断言 360° 与全开 VisibleSet 完全一致（含顺序），
// 且任意张角下可见集都是 VisibleSet 的子集。
func TestFOVFullEqualsVisibleSet(t *testing.T) {
	maps := [][]string{
		{".........", "..#......", ".....@...", ".........", "....###..", "........."},
		{"###", "#.#", "###"},
		{"#....", ".....", "..#..", ".....", "....#"},
		{"#####", "#...#", "#.#.#", "#...#", "#####"},
	}
	for mi, rows := range maps {
		g := mustParse(t, rows)
		origins := walkablePoints(g)
		if len(origins) == 0 {
			t.Fatalf("地图 %d 没有可走格", mi)
		}
		for _, origin := range origins {
			for radius := 1; radius <= g.MaxRadius(); radius++ {
				full, err := g.VisibleSet(origin, radius)
				if err != nil {
					t.Fatalf("地图 %d 原点 %v 半径 %d: %v", mi, origin, radius, err)
				}
				if !pointsSet(full)[origin] {
					t.Fatalf("地图 %d 原点 %v 半径 %d：全开集不含观察点", mi, origin, radius)
				}
				wide, err := g.FOV(FOVConfig{Origin: origin, Facing: South, AngleDeg: 360, Radius: radius})
				if err != nil {
					t.Fatalf("360° 报错：%v", err)
				}
				if len(wide) != len(full) {
					t.Fatalf("地图 %d 原点 %v 半径 %d：360° %d 格 != 全开 %d 格",
						mi, origin, radius, len(wide), len(full))
				}
				for i := range full {
					if full[i] != wide[i] {
						t.Fatalf("地图 %d 原点 %v 半径 %d：360° 与全开顺序不一致：%v vs %v",
							mi, origin, radius, wide[i], full[i])
					}
				}
				for _, angle := range []float64{1, 45, 90, 180, 270, 359.9} {
					vis, err := g.FOV(FOVConfig{Origin: origin, Facing: East, AngleDeg: angle, Radius: radius})
					if err != nil {
						t.Fatalf("角度 %g 报错：%v", angle, err)
					}
					diff := DiffSets(full, vis)
					if len(diff.OnlyB) != 0 {
						t.Fatalf("地图 %d 原点 %v 半径 %d 角度 %g：FOV 含非全开可见格 %v",
							mi, origin, radius, angle, diff.OnlyB)
					}
				}
			}
		}
	}
}

// TestFOVMonotoneInAngle 断言可见格数随张角单调不减，且张角覆盖 360° 时等于全开。
func TestFOVMonotoneInAngle(t *testing.T) {
	g := mustParse(t, []string{
		"...........",
		"..##.......",
		"..##...##..",
		".......##..",
		"...........",
	})
	origins := walkablePoints(g)
	for _, origin := range origins {
		prev := -1
		for _, angle := range []float64{10, 30, 60, 90, 120, 180, 270, 359, 360} {
			vis, err := g.FOV(FOVConfig{Origin: origin, Facing: North, AngleDeg: angle, Radius: 5})
			if err != nil {
				t.Fatalf("原点 %v 角度 %g：%v", origin, angle, err)
			}
			if len(vis) < prev {
				t.Fatalf("原点 %v：角度 %g 可见 %d 格，少于更窄角的 %d 格",
					origin, angle, len(vis), prev)
			}
			prev = len(vis)
		}
	}
}

// TestFOVNarrowAndWide 用开阔地图核对扇形掩码的几何边界（含等号容差）。
func TestFOVNarrowAndWide(t *testing.T) {
	g := mustParse(t, []string{
		".......",
		".......",
		".......",
		".......",
		".......",
		".......",
		".......",
	})
	origin := Point{X: 3, Y: 3}
	radius := 3

	narrow, err := g.FOV(FOVConfig{Origin: origin, Facing: East, AngleDeg: 90, Radius: radius})
	if err != nil {
		t.Fatal(err)
	}
	narrowSet := pointsSet(narrow)
	// 正东与 ±45° 边界（含等号容差）可见；更靠北/南/西的格在扇形之外。
	for _, p := range []Point{{X: 6, Y: 3}, {X: 5, Y: 1}, {X: 5, Y: 5}, {X: 4, Y: 4}} {
		if !narrowSet[p] {
			t.Errorf("90° 东向应含 %v", p)
		}
	}
	for _, p := range []Point{{X: 0, Y: 3}, {X: 3, Y: 0}, {X: 4, Y: 1}, {X: 3, Y: 6}} {
		if narrowSet[p] {
			t.Errorf("90° 东向不应含 %v", p)
		}
	}
	if !narrowSet[origin] {
		t.Error("观察点必须在可见集内")
	}

	wide, err := g.FOV(FOVConfig{Origin: origin, Facing: East, AngleDeg: 270, Radius: radius})
	if err != nil {
		t.Fatal(err)
	}
	wideSet := pointsSet(wide)
	// 270° 张角保留背后 ±45° 以外的方向：正北、正南与 ±135° 边界可见，正西不可见。
	for _, p := range []Point{{X: 3, Y: 0}, {X: 3, Y: 6}, {X: 1, Y: 1}, {X: 1, Y: 5}} {
		if !wideSet[p] {
			t.Errorf("270° 东向应含 %v", p)
		}
	}
	if wideSet[Point{X: 0, Y: 3}] || wideSet[Point{X: 1, Y: 2}] {
		t.Error("270° 东向不应含正西及其附近的格")
	}

	// 窄角可见集必须是宽角可见集的子集。
	if d := DiffSets(wide, narrow); len(d.OnlyB) != 0 {
		t.Errorf("窄角含宽角之外的格：%v", d.OnlyB)
	}
}

// TestFOVWallHug 覆盖贴墙情形：整面墙只在扇形一侧裁掉视线。
func TestFOVWallHug(t *testing.T) {
	g := mustParse(t, []string{
		"..........",
		"..........",
		".....#####",
		"....@.....",
		"..........",
		"..........",
	})
	origin := Point{X: 4, Y: 3}
	vis, err := g.FOV(FOVConfig{Origin: origin, Facing: North, AngleDeg: 90, Radius: 6})
	if err != nil {
		t.Fatal(err)
	}
	set := pointsSet(vis)
	// 正前方空旷：可见到半径边缘。
	if !set[Point{X: 4, Y: 0}] {
		t.Error("正前方开阔格应可见")
	}
	// 墙直接挡住东北方向：墙本身不可见，墙后格被遮挡。
	for _, p := range []Point{{X: 5, Y: 2}, {X: 6, Y: 1}, {X: 7, Y: 1}} {
		if set[p] {
			t.Errorf("%v 在墙内或墙后，不应可见", p)
		}
	}
	// 扇形之外的西侧格不可见。
	if set[Point{X: 0, Y: 2}] {
		t.Error("扇形外格 (0,2) 不应可见")
	}
	// 全开时墙后同样不可见，故墙后格不在差集中。
	full, _ := g.VisibleSet(origin, 6)
	if d := DiffSets(full, vis); len(d.OnlyB) != 0 {
		t.Fatalf("贴墙：可见集不是全开子集 %v", d.OnlyB)
	}
	// 贴墙时全开集也应恰好是扇形外多出的那部分。
	if len(vis) >= len(full) {
		t.Errorf("90° 贴墙可见 %d 格，应少于全开 %d 格", len(vis), len(full))
	}
}

// TestFOVCorner 覆盖角落情形：观察点位于地图角与凹角。
func TestFOVCorner(t *testing.T) {
	open := mustParse(t, []string{".....", ".....", ".....", ".....", "....."})
	origin := Point{X: 0, Y: 0}
	vis, err := open.FOV(FOVConfig{Origin: origin, Facing: East, AngleDeg: 90, Radius: 6})
	if err != nil {
		t.Fatal(err)
	}
	set := pointsSet(vis)
	if !set[Point{X: 4, Y: 0}] || !set[Point{X: 4, Y: 3}] {
		t.Error("角落 90° 东向应含东向半轴与 45° 边界内的格")
	}
	if set[Point{X: 6, Y: 4}] || set[Point{X: 0, Y: 4}] {
		t.Error("角落 90° 东向不应含 45° 之外的东南格与背后格")
	}
	// 出界的扇形半边不产生越界格。
	for p := range set {
		if p.X < 0 || p.Y < 0 || p.X >= open.W || p.Y >= open.H {
			t.Fatalf("可见集含越界格 %v", p)
		}
	}

	// 凹角：背靠北墙与西墙，向南看时墙角外的西南方向被挡。
	notch := mustParse(t, []string{
		"..........",
		"..........",
		"..........",
		"....######",
		"....#.....",
		"....#.....",
	})
	nOrigin := Point{X: 5, Y: 4}
	nvis, err := notch.FOV(FOVConfig{Origin: nOrigin, Facing: South, AngleDeg: 90, Radius: 5})
	if err != nil {
		t.Fatal(err)
	}
	nset := pointsSet(nvis)
	if !nset[Point{X: 5, Y: 4}] {
		t.Error("凹角观察点自身应可见")
	}
	if !nset[Point{X: 6, Y: 5}] {
		t.Error("凹角 90° 南向应含 45° 边界内的东南格")
	}
	if nset[Point{X: 4, Y: 3}] || nset[Point{X: 5, Y: 3}] {
		t.Error("凹角墙内格不应可见")
	}
	// 墙角西侧在扇形之外，且被西侧墙挡住。
	if nset[Point{X: 4, Y: 5}] {
		t.Error("凹角 90° 南向不应含西侧墙角外的格")
	}
	// 沿墙面向东看时，东北一侧被北墙截断。
	east, err := notch.FOV(FOVConfig{Origin: nOrigin, Facing: East, AngleDeg: 90, Radius: 5})
	if err != nil {
		t.Fatal(err)
	}
	eset := pointsSet(east)
	if !eset[Point{X: 8, Y: 4}] {
		t.Error("凹角 90° 东向应含正东开阔格")
	}
	if eset[Point{X: 7, Y: 3}] {
		t.Error("凹角 90° 东向不应含北墙内格")
	}
}

// TestFOVFullyOccluded 覆盖全遮挡：紧贴一整排墙，扇形内只剩观察点。
func TestFOVFullyOccluded(t *testing.T) {
	g := mustParse(t, []string{
		".....",
		"#####",
		"..@..",
		".....",
		".....",
	})
	origin := Point{X: 2, Y: 2}
	vis, err := g.FOV(FOVConfig{Origin: origin, Facing: North, AngleDeg: 90, Radius: 5})
	if err != nil {
		t.Fatal(err)
	}
	if len(vis) != 1 || vis[0] != origin {
		t.Fatalf("全遮挡应只剩观察点，实际 %v", vis)
	}
	// 换个朝向、加宽张角，北侧依然一格不可见。
	for _, angle := range []float64{90, 180, 270, 360} {
		vis, err := g.FOV(FOVConfig{Origin: origin, Facing: North, AngleDeg: angle, Radius: 5})
		if err != nil {
			t.Fatal(err)
		}
		for _, p := range vis {
			if p.Y < origin.Y && p != origin {
				t.Fatalf("角度 %g：墙后格 %v 不应可见", angle, p)
			}
		}
	}
}

// TestFOVRandomSubset 用固定种子的随机地图对“FOV ⊆ VisibleSet”做批量对照，
// 并给出差集的规模，作为可见集合差的上界观察。
func TestFOVRandomSubset(t *testing.T) {
	rng := rand.New(rand.NewSource(20261124))
	for iter := 0; iter < 40; iter++ {
		w, h := 5+rng.Intn(8), 5+rng.Intn(8)
		rows := make([]string, h)
		for y := 0; y < h; y++ {
			b := make([]byte, w)
			for x := 0; x < w; x++ {
				if rng.Intn(100) < 25 {
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
		origins := walkablePoints(g)
		if len(origins) == 0 {
			continue
		}
		origin := origins[rng.Intn(len(origins))]
		radius := 1 + rng.Intn(g.MaxRadius())
		full, err := g.VisibleSet(origin, radius)
		if err != nil {
			t.Fatal(err)
		}
		fullSet := pointsSet(full)
		for _, facing := range []Facing{North, East, South, West} {
			for _, angle := range []float64{30, 90, 180, 270, 360} {
				vis, err := g.FOV(FOVConfig{Origin: origin, Facing: facing, AngleDeg: angle, Radius: radius})
				if err != nil {
					t.Fatalf("地图 %v 原点 %v：%v", rows, origin, err)
				}
				seen := pointsSet(vis)
				for p := range seen {
					if !fullSet[p] {
						t.Fatalf("地图 %v 原点 %v 朝向 %v 角度 %g 半径 %d：FOV 含非全开格 %v",
							rows, origin, facing, angle, radius, p)
					}
				}
				if !seen[origin] {
					t.Fatalf("地图 %v 原点 %v 朝 %v 角 %g：观察点缺失", rows, origin, facing, angle)
				}
			}
		}
	}
}

// TestSectorCellsUpperBound 断言 SectorCells 是 FOV 的无遮挡上界：
// FOV ⊆ SectorCells ⊆ 半径内可走格，且 360° 时 SectorCells 含全部半径内可走格。
func TestSectorCellsUpperBound(t *testing.T) {
	maps := [][]string{
		{".........", "..#......", ".........", "....###..", "........."},
		{"#####", "#...#", "#.#.#", "#...#", "#####"},
		{".....", ".....", "....."},
	}
	for mi, rows := range maps {
		g := mustParse(t, rows)
		for _, origin := range walkablePoints(g) {
			for radius := 1; radius <= g.MaxRadius(); radius++ {
				allInRadius := make(map[Point]bool)
				r2 := radius * radius
				for _, p := range walkablePoints(g) {
					dx, dy := p.X-origin.X, p.Y-origin.Y
					if dx*dx+dy*dy <= r2 {
						allInRadius[p] = true
					}
				}
				full360, err := g.SectorCells(FOVConfig{Origin: origin, Facing: North, AngleDeg: 360, Radius: radius})
				if err != nil {
					t.Fatal(err)
				}
				if len(full360) != len(allInRadius) {
					t.Fatalf("地图 %d 原点 %v 半径 %d：360° 扇形掩码 %d 格，半径内可走格 %d",
						mi, origin, radius, len(full360), len(allInRadius))
				}
				for _, facing := range []Facing{North, East, South, West} {
					for _, angle := range []float64{30, 90, 180, 270, 360} {
						cfg := FOVConfig{Origin: origin, Facing: facing, AngleDeg: angle, Radius: radius}
						sector, err := g.SectorCells(cfg)
						if err != nil {
							t.Fatal(err)
						}
						vis, err := g.FOV(cfg)
						if err != nil {
							t.Fatal(err)
						}
						sectorSet := pointsSet(sector)
						for _, p := range vis {
							if !sectorSet[p] {
								t.Fatalf("地图 %d 原点 %v 朝向 %v 角 %g：FOV 格 %v 不在扇形掩码内",
									mi, origin, facing, angle, p)
							}
						}
						for _, p := range sector {
							if !allInRadius[p] {
								t.Fatalf("地图 %d 原点 %v 半径 %d：扇形掩码含半径外格 %v",
									mi, origin, radius, p)
							}
						}
						// 张角单调性同样适用于扇形掩码。
						if angle == 270 {
							narrow, _ := g.SectorCells(FOVConfig{Origin: origin, Facing: facing, AngleDeg: 90, Radius: radius})
							if d := DiffSets(sector, narrow); len(d.OnlyB) != 0 {
								t.Fatalf("90° 扇形掩码含 270° 之外的格 %v", d.OnlyB)
							}
						}
					}
				}
			}
		}
	}
}

// TestFOVFullModeUnchanged 断言 360° 走的是与全开完全相同的路径：
// 结果与 VisibleSet 逐元素相等，且对非法参数仍按 VisibleSet 的口径报错。
func TestFOVFullModeUnchanged(t *testing.T) {
	g := mustParse(t, []string{".....", "..#..", ".....", "..#..", "....."})
	for _, origin := range walkablePoints(g) {
		for radius := 1; radius <= g.MaxRadius(); radius++ {
			want, err := g.VisibleSet(origin, radius)
			if err != nil {
				t.Fatal(err)
			}
			got, err := g.FOV(FOVConfig{Origin: origin, Facing: West, AngleDeg: 360, Radius: radius})
			if err != nil {
				t.Fatal(err)
			}
			if len(got) != len(want) {
				t.Fatalf("原点 %v 半径 %d：360°=%d，全开=%d", origin, radius, len(got), len(want))
			}
			for i := range want {
				if got[i] != want[i] {
					t.Fatalf("原点 %v 半径 %d：第 %d 个元素 %v != %v", origin, radius, i, got[i], want[i])
				}
			}
			// 全开模式下 Radius 错误与 VisibleSet 的错误类型一致。
			if _, err := g.FOV(FOVConfig{Origin: origin, Facing: West, AngleDeg: 360, Radius: radius + 1000}); !errors.Is(err, ErrRadius) {
				t.Fatalf("360° 半径超界应报 ErrRadius，实际 %v", err)
			}
		}
	}
}

func TestDiffSets(t *testing.T) {
	a := []Point{{X: 0, Y: 0}, {X: 1, Y: 0}, {X: 2, Y: 0}}
	b := []Point{{X: 1, Y: 0}, {X: 3, Y: 0}}
	d := DiffSets(a, b)
	if len(d.OnlyA) != 2 || d.OnlyA[0] != (Point{X: 0, Y: 0}) || d.OnlyA[1] != (Point{X: 2, Y: 0}) {
		t.Fatalf("OnlyA = %v", d.OnlyA)
	}
	if len(d.OnlyB) != 1 || d.OnlyB[0] != (Point{X: 3, Y: 0}) {
		t.Fatalf("OnlyB = %v", d.OnlyB)
	}
	if d := DiffSets(nil, nil); len(d.OnlyA) != 0 || len(d.OnlyB) != 0 {
		t.Fatalf("空集差集应为空：%+v", d)
	}
}

// TestVisibleSetRadius 核对半径按格心欧氏距离、边界含等号。
func TestVisibleSetRadius(t *testing.T) {
	g := mustParse(t, []string{".....", ".....", ".....", ".....", "....."})
	origin := Point{X: 0, Y: 0}
	vis, err := g.VisibleSet(origin, 2)
	if err != nil {
		t.Fatal(err)
	}
	for _, p := range []Point{{X: 2, Y: 0}, {X: 1, Y: 1}} {
		if !pointsSet(vis)[p] {
			t.Errorf("半径 2 应含 %v（距离 ≤ 2）", p)
		}
	}
	if pointsSet(vis)[Point{X: 2, Y: 2}] {
		t.Error("半径 2 不应含 (2,2)（距离 2.83 > 2）")
	}
	if _, err := g.VisibleSet(origin, 0); !errors.Is(err, ErrRadius) {
		t.Errorf("半径 0 应报 ErrRadius，实际 %v", err)
	}
	if _, err := g.VisibleSet(origin, g.MaxRadius()+1); !errors.Is(err, ErrRadius) {
		t.Errorf("半径超界应报 ErrRadius，实际 %v", err)
	}
}

// TestFacingString 保证朝向的可读输出覆盖全部合法值与非法值。
func TestFacingString(t *testing.T) {
	want := map[Facing]string{North: "N", East: "E", South: "S", West: "W"}
	for f, s := range want {
		if !f.Valid() || f.String() != s {
			t.Errorf("朝向 %d：Valid=%v String=%q", int(f), f.Valid(), f.String())
		}
	}
	if Facing(4).Valid() || Facing(-1).Valid() {
		t.Error("越界朝向不应 Valid")
	}
	if !strings.HasPrefix(Facing(9).String(), "Facing(") {
		t.Errorf("非法朝向字符串 %q", Facing(9).String())
	}
}
