package grid

import (
	"errors"
	"math"
	"testing"
)

// 粗档（约 5 格量级）五张用例图。
func coarseMaps() map[string][]string {
	return map[string][]string{
		"single": {
			".....",
			".....",
			".....",
			".....",
			".....",
		},
		"multi": {
			"..#...",
			"..#...",
			"######",
			"...#..",
			"...#..",
			"...#..",
		},
		"ring": {
			"#####",
			"#...#",
			"#.#.#",
			"#...#",
			"#####",
		},
		"thin": {"........"},
		"wall": {
			"####",
			"####",
			"####",
			"####",
		},
	}
}

// 细档（约 9 格量级）五张用例图。
func fineMaps() map[string][]string {
	return map[string][]string{
		"single": {
			".........",
			".........",
			".........",
			".........",
			".........",
			".........",
			".........",
			".........",
			".........",
		},
		"multi": {
			"..#......",
			"..#......",
			"..#......",
			"#########",
			"...#.....",
			"...#.....",
			"...#.....",
			"...#.....",
			"...#.....",
		},
		"ring": {
			"#########",
			"#.......#",
			"#.#####.#",
			"#.#####.#",
			"#.#####.#",
			"#.#####.#",
			"#.#####.#",
			"#.......#",
			"#########",
		},
		"thin": {"................"},
		"wall": {
			"#########",
			"#########",
			"#########",
			"#########",
			"#########",
			"#########",
			"#########",
			"#########",
			"#########",
		},
	}
}

func approx(a, b float64) bool { return math.Abs(a-b) < 1e-9 }

// assertRegions 对每张非全墙图做通用不变量校验。
func assertRegions(t *testing.T, g *Grid, regions []Region) {
	t.Helper()
	walkable := g.WalkableCount()
	totalArea := 0
	seen := map[Point]bool{}
	for i, r := range regions {
		if r.ID != i+1 {
			t.Fatalf("区域编号应连续，期望 %d 实际 %d", i+1, r.ID)
		}
		if r.Area != len(r.Cells) || r.Area == 0 {
			t.Fatalf("区域 %d 面积与格子数不一致", r.ID)
		}
		totalArea += r.Area
		// 质心必须是本区域内的可走格
		belongs := false
		var sumX, sumY float64
		for _, c := range r.Cells {
			if seen[c] {
				t.Fatalf("格子 (%d,%d) 被重复划分", c.X, c.Y)
			}
			seen[c] = true
			if ok, err := g.Walkable(c); err != nil || !ok {
				t.Fatalf("区域 %d 含不可走格 (%d,%d)", r.ID, c.X, c.Y)
			}
			if c == r.Centroid {
				belongs = true
			}
			sumX += float64(c.X)
			sumY += float64(c.Y)
		}
		if !belongs {
			t.Fatalf("区域 %d 的质心 (%d,%d) 不在本区域内", r.ID, r.Centroid.X, r.Centroid.Y)
		}
		if !approx(r.Mean[0], sumX/float64(r.Area)) || !approx(r.Mean[1], sumY/float64(r.Area)) {
			t.Fatalf("区域 %d 平均坐标错误: 得到 (%v,%v)", r.ID, r.Mean[0], r.Mean[1])
		}
		// 标注质心必须是距精确质心最近的区域格（平局规则确定）
		want := nearestCell(r.Cells, r.Mean)
		if want != r.Centroid {
			t.Fatalf("区域 %d 最近格应为 (%d,%d)，实际 (%d,%d)",
				r.ID, want.X, want.Y, r.Centroid.X, r.Centroid.Y)
		}
		// 实际偏差不得超过由面积反解的精度值
		if !r.WithinTolerance() {
			t.Fatalf("区域 %d 偏差 %v 超过反解精度 %v", r.ID, r.Deviation(), CentroidTolerance(r.Area))
		}
	}
	if totalArea != walkable {
		t.Fatalf("区域总面积 %d 与可走格数 %d 不符", totalArea, walkable)
	}
}

func TestLabelComponentsCoarse(t *testing.T) {
	maps := coarseMaps()
	wantCount := map[string]int{"single": 1, "multi": 4, "ring": 1, "thin": 1}
	for name, rows := range maps {
		g, err := Parse(rows)
		if err != nil {
			t.Fatalf("%s 解析失败: %v", name, err)
		}
		regions, err := g.LabelComponents()
		if name == "wall" {
			if !errors.Is(err, ErrNoRegion) {
				t.Fatalf("全墙应报 ErrNoRegion，实际 %v", err)
			}
			continue
		}
		if err != nil {
			t.Fatalf("%s 标记失败: %v", name, err)
		}
		if len(regions) != wantCount[name] {
			t.Fatalf("%s 区域数期望 %d 实际 %d", name, wantCount[name], len(regions))
		}
		assertRegions(t, g, regions)
	}
}

func TestLabelComponentsFine(t *testing.T) {
	maps := fineMaps()
	wantCount := map[string]int{"single": 1, "multi": 4, "ring": 1, "thin": 1}
	for name, rows := range maps {
		g, err := Parse(rows)
		if err != nil {
			t.Fatalf("%s 解析失败: %v", name, err)
		}
		regions, err := g.LabelComponents()
		if name == "wall" {
			if !errors.Is(err, ErrNoRegion) {
				t.Fatalf("全墙应报 ErrNoRegion，实际 %v", err)
			}
			continue
		}
		if err != nil {
			t.Fatalf("%s 标记失败: %v", name, err)
		}
		if len(regions) != wantCount[name] {
			t.Fatalf("%s 区域数期望 %d 实际 %d", name, wantCount[name], len(regions))
		}
		assertRegions(t, g, regions)
	}
}

func TestSingleRegionValues(t *testing.T) {
	g, _ := Parse(coarseMaps()["single"])
	regions, err := g.LabelComponents()
	if err != nil {
		t.Fatal(err)
	}
	r := regions[0]
	if r.Area != 25 || r.Centroid != (Point{2, 2}) || r.Deviation() != 0 {
		t.Fatalf("单区质心错误: %+v", r)
	}
	if !approx(r.Mean[0], 2) || !approx(r.Mean[1], 2) {
		t.Fatalf("单区平均坐标错误: %v", r.Mean)
	}
}

func TestRingCentroidSnapsToWalkable(t *testing.T) {
	// 粗档环：精确质心 (2,2) 落在墙洞，须吸附到最近可走格 (1,2)，偏差 1
	g, _ := Parse(coarseMaps()["ring"])
	regions, err := g.LabelComponents()
	if err != nil {
		t.Fatal(err)
	}
	r := regions[0]
	if r.Area != 8 {
		t.Fatalf("环面积期望 8，实际 %d", r.Area)
	}
	if ok, _ := g.Walkable(Point{2, 2}); ok {
		t.Fatal("(2,2) 应为墙洞")
	}
	if r.Centroid != (Point{1, 2}) {
		t.Fatalf("环形质心吸附错误，期望 (1,2) 实际 (%d,%d)", r.Centroid.X, r.Centroid.Y)
	}
	if !approx(r.Deviation(), 1) || !approx(CentroidTolerance(r.Area), math.Sqrt(8)) {
		t.Fatalf("环形偏差/精度错误: dev=%v tol=%v", r.Deviation(), CentroidTolerance(r.Area))
	}

	// 细档环：5x5 墙洞，偏差 3，面积 24，精度 √24 ≈ 4.899
	g2, _ := Parse(fineMaps()["ring"])
	rs2, err := g2.LabelComponents()
	if err != nil {
		t.Fatal(err)
	}
	r2 := rs2[0]
	if r2.Area != 24 || r2.Centroid != (Point{1, 4}) || !approx(r2.Deviation(), 3) {
		t.Fatalf("细档环形错误: %+v dev=%v", r2, r2.Deviation())
	}
}

func TestThinRegion(t *testing.T) {
	// 粗档 1x8：平均 x=3.5，平局取较小 X → (3,0)
	g, _ := Parse(coarseMaps()["thin"])
	regions, err := g.LabelComponents()
	if err != nil {
		t.Fatal(err)
	}
	r := regions[0]
	if r.Area != 8 || r.Centroid != (Point{3, 0}) || !approx(r.Deviation(), 0.5) {
		t.Fatalf("细长区域错误: %+v dev=%v", r, r.Deviation())
	}
}

func TestMultiRegionAreas(t *testing.T) {
	g, _ := Parse(coarseMaps()["multi"])
	regions, err := g.LabelComponents()
	if err != nil {
		t.Fatal(err)
	}
	wantAreas := []int{4, 6, 9, 6}
	wantCentroids := []Point{{0, 0}, {4, 0}, {1, 4}, {4, 4}}
	for i, r := range regions {
		if r.Area != wantAreas[i] {
			t.Fatalf("区域 %d 面积期望 %d 实际 %d", r.ID, wantAreas[i], r.Area)
		}
		if r.Centroid != wantCentroids[i] {
			t.Fatalf("区域 %d 质心期望 (%d,%d) 实际 (%d,%d)",
				r.ID, wantCentroids[i].X, wantCentroids[i].Y, r.Centroid.X, r.Centroid.Y)
		}
	}
}

func TestDeterministic(t *testing.T) {
	g, _ := Parse(fineMaps()["multi"])
	first, err := g.LabelComponents()
	if err != nil {
		t.Fatal(err)
	}
	second, err := g.LabelComponents()
	if err != nil {
		t.Fatal(err)
	}
	if len(first) != len(second) {
		t.Fatal("重复标记结果不稳定")
	}
	for i := range first {
		if first[i].Centroid != second[i].Centroid || first[i].Area != second[i].Area {
			t.Fatalf("区域 %d 重复标记结果不一致", i+1)
		}
	}
}

func TestEmptyGrid(t *testing.T) {
	if _, err := Parse(nil); !errors.Is(err, ErrEmpty) {
		t.Fatalf("空网格应报 ErrEmpty，实际 %v", err)
	}
	if _, err := Parse([]string{""}); !errors.Is(err, ErrEmpty) {
		t.Fatalf("零宽网格应报 ErrEmpty，实际 %v", err)
	}
	var nilGrid *Grid
	if _, err := nilGrid.LabelComponents(); !errors.Is(err, ErrEmpty) {
		t.Fatalf("nil 网格应报 ErrEmpty，实际 %v", err)
	}
}

func TestCentroidOfErrors(t *testing.T) {
	g, _ := Parse(coarseMaps()["single"])
	if _, err := g.CentroidOf(0); !errors.Is(err, ErrOOB) {
		t.Fatalf("编号 0 应报 ErrOOB，实际 %v", err)
	}
	if _, err := g.CentroidOf(2); !errors.Is(err, ErrOOB) {
		t.Fatalf("越界编号应报 ErrOOB，实际 %v", err)
	}
	// 全墙：无区域可用
	w, _ := Parse(coarseMaps()["wall"])
	if _, err := w.CentroidOf(1); !errors.Is(err, ErrNoRegion) {
		t.Fatalf("全墙应报 ErrNoRegion，实际 %v", err)
	}
}

func TestCentroidAtErrors(t *testing.T) {
	g, _ := Parse(coarseMaps()["ring"])
	// 越界坐标须随错误报出
	_, err := g.CentroidAt(Point{-1, 0})
	if !errors.Is(err, ErrOOB) {
		t.Fatalf("越界应报 ErrOOB，实际 %v", err)
	}
	if err == nil || err.Error() == "" {
		t.Fatal("越界错误应携带信息")
	}
	// 墙体坐标（区域为空的归属查询）须随错误报出
	_, err = g.CentroidAt(Point{2, 2})
	if !errors.Is(err, ErrNoRegion) {
		t.Fatalf("墙体查询应报 ErrNoRegion，实际 %v", err)
	}
	// 正常归属查询：环上格返回环的质心
	c, err := g.CentroidAt(Point{3, 2})
	if err != nil || c != (Point{1, 2}) {
		t.Fatalf("环上格归属质心错误: %v %v", c, err)
	}
}

func TestPathAPIUnchanged(t *testing.T) {
	// 回归：连通域功能加入后，原有寻路接口行为保持不变。
	g, _ := Parse([]string{"...", ".#.", "..."})
	path, _, err := g.Path(Point{0, 0}, Point{2, 2})
	if err != nil || len(path) != 5 {
		t.Fatalf("原有寻路接口被破坏: len=%d err=%v", len(path), err)
	}
	ok, err := g.Visible(Point{1, 0}, Point{1, 2})
	if err != nil || ok {
		t.Fatalf("原有可视接口被破坏: ok=%v err=%v", ok, err)
	}
}
