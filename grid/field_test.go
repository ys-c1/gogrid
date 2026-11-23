package grid

import (
	"errors"
	"fmt"
	"testing"
)

// followAndVerify 跑一个单位并逐格校验路线合法性，返回路径与回退次数。
func followAndVerify(t *testing.T, f *Field, from Point) ([]Point, int) {
	t.Helper()
	path, fb, err := f.Follow(from)
	if err != nil {
		t.Fatalf("Follow(%v) 出错: %v", from, err)
	}
	if len(path) == 0 || path[0] != from {
		t.Fatalf("路线起点错误: %v", path)
	}
	targets := f.Targets()
	for i, p := range path {
		if ok, err := f.g.Walkable(p); err != nil || !ok {
			t.Fatalf("路线上有不可走格 %v", p)
		}
		if i == 0 {
			continue
		}
		d := abs(p.X-path[i-1].X) + abs(p.Y-path[i-1].Y)
		if d != 1 {
			t.Fatalf("路线不连续: %v -> %v", path[i-1], p)
		}
	}
	end := path[len(path)-1]
	isTarget := false
	for _, tgt := range targets {
		if end == tgt {
			isTarget = true
		}
	}
	if !isTarget {
		t.Fatalf("终点 %v 不是目标格", end)
	}
	return path, fb
}

func allWalkable(g *Grid) []Point {
	var out []Point
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if g.cells[y][x] {
				out = append(out, Point{x, y})
			}
		}
	}
	return out
}

func TestFieldOpenGround(t *testing.T) {
	g, _ := Parse([]string{
		"........",
		"........",
		"........",
		"........",
	})
	f, err := BuildField(g, Point{7, 3})
	if err != nil {
		t.Fatal(err)
	}
	if err := f.CheckTriangle(); err != nil {
		t.Fatal(err)
	}
	if err := f.CheckDescent(); err != nil {
		t.Fatal(err)
	}
	if got, ok := f.Dist(Point{0, 0}); !ok || got != 10 {
		t.Fatalf("(0,0) 距离 = %d,%v，期望 10", got, ok)
	}
	// 开阔地每个单位自己算一次，路线长度必须等于曼哈顿距离且无回退。
	for _, s := range allWalkable(g) {
		path, fb := followAndVerify(t, f, s)
		want := abs(s.X-7) + abs(s.Y-3) + 1
		if len(path) != want || fb != 0 {
			t.Fatalf("从 %v 出发 长度 %d 期望 %d 回退 %d", s, len(path), want, fb)
		}
	}
	if f.Fallbacks() != 0 {
		t.Fatalf("开阔地不应有回退，实际 %d", f.Fallbacks())
	}
}

func TestFieldMaze(t *testing.T) {
	g, _ := Parse([]string{
		"#########",
		"#.......#",
		"#.#####.#",
		"#.#...#.#",
		"#.#.#.#.#",
		"#...#...#",
		"#########",
	})
	f, err := BuildField(g, Point{1, 1})
	if err != nil {
		t.Fatal(err)
	}
	if err := f.CheckTriangle(); err != nil {
		t.Fatal(err)
	}
	if err := f.CheckDescent(); err != nil {
		t.Fatal(err)
	}
	// 沿左侧通道直达：(1,5)->(1,1)，5 个格点。
	short, fb := followAndVerify(t, f, Point{1, 5})
	if fb != 0 || len(short) != 5 {
		t.Fatalf("迷宫左侧路线异常 len=%d fb=%d", len(short), fb)
	}
	// 从右下角必须绕经右侧竖井与顶部长廊，不能穿墙。
	long, fb := followAndVerify(t, f, Point{7, 5})
	if fb != 0 || len(long) != 11 {
		t.Fatalf("迷宫绕行路线异常 len=%d fb=%d", len(long), fb)
	}
	for _, p := range long {
		if p.X == 4 && p.Y != 1 {
			t.Fatalf("路线穿过了墙体: %v", long)
		}
	}
	// 墙格不在场中。
	if _, ok := f.Dist(Point{0, 0}); ok {
		t.Fatal("墙格不应在场中")
	}
}

func TestFieldNarrowGap(t *testing.T) {
	g, _ := Parse([]string{
		".....#.....",
		".....#.....",
		".....#.....",
		".....#.....",
		"...........",
		".....#.....",
		".....#.....",
	})
	f, err := BuildField(g, Point{10, 0})
	if err != nil {
		t.Fatal(err)
	}
	if err := f.CheckTriangle(); err != nil {
		t.Fatal(err)
	}
	if err := f.CheckDescent(); err != nil {
		t.Fatal(err)
	}
	// 左半区单位必须挤过唯一缺口 (5,4)；右半区无需过门。
	for _, s := range allWalkable(g) {
		path, fb := followAndVerify(t, f, s)
		if fb != 0 {
			t.Fatalf("窄门不应回退: %v fb=%d", s, fb)
		}
		if s.X >= 6 {
			continue
		}
		found := false
		for _, p := range path {
			if p == (Point{5, 4}) {
				found = true
			}
		}
		if !found {
			t.Fatalf("从 %v 出发的路线未经过窄门: %v", s, path)
		}
	}
}

func TestFieldMultiTarget(t *testing.T) {
	g, _ := Parse([]string{
		".........",
		".........",
		".........",
	})
	targets := []Point{{0, 0}, {8, 2}}
	f, err := BuildField(g, targets...)
	if err != nil {
		t.Fatal(err)
	}
	if err := f.CheckTriangle(); err != nil {
		t.Fatal(err)
	}
	if err := f.CheckDescent(); err != nil {
		t.Fatal(err)
	}
	if f.Stats().Targets != 2 {
		t.Fatalf("目标数 %d", f.Stats().Targets)
	}
	for _, s := range allWalkable(g) {
		path, fb := followAndVerify(t, f, s)
		if fb != 0 {
			t.Fatalf("%v 不应回退", s)
		}
		// 路线长度应等于到最近目标的曼哈顿距离 + 1。
		best := 1 << 30
		for _, tgt := range targets {
			if d := abs(s.X-tgt.X) + abs(s.Y-tgt.Y); d < best {
				best = d
			}
		}
		if len(path) != best+1 {
			t.Fatalf("%v 路线长度 %d，期望 %d", s, len(path), best+1)
		}
	}
}

func TestFieldAllWall(t *testing.T) {
	g, _ := Parse([]string{
		"###",
		"###",
		"###",
	})
	_, err := BuildField(g, Point{1, 1})
	if !errors.Is(err, ErrTargetBlocked) {
		t.Fatalf("全墙目标应报 ErrTargetBlocked，实际 %v", err)
	}
	var ce *CoordError
	if !errors.As(err, &ce) || ce.At != (Point{1, 1}) {
		t.Fatalf("应携带坐标 (1,1)，实际 %v", err)
	}
}

func TestFieldErrors(t *testing.T) {
	g, _ := Parse([]string{"...", ".#.", "..."})

	// 空网格。
	var empty Grid
	if _, err := BuildField(&empty, Point{0, 0}); !errors.Is(err, ErrEmpty) {
		t.Fatalf("空网格应报 ErrEmpty，实际 %v", err)
	}
	if _, err := BuildField(nil, Point{0, 0}); !errors.Is(err, ErrEmpty) {
		t.Fatalf("nil 网格应报 ErrEmpty，实际 %v", err)
	}
	if _, err := Parse(nil); !errors.Is(err, ErrEmpty) {
		t.Fatalf("Parse(nil) 应报 ErrEmpty，实际 %v", err)
	}

	// 目标不可走（墙格）。
	_, err := BuildField(g, Point{1, 1})
	var ce *CoordError
	if !errors.Is(err, ErrTargetBlocked) || !errors.As(err, &ce) || ce.At != (Point{1, 1}) {
		t.Fatalf("墙目标应报坐标 (1,1)，实际 %v", err)
	}

	// 目标越界。
	if _, err := BuildField(g, Point{5, 0}); !errors.Is(err, ErrOOB) {
		t.Fatalf("越界目标应报 ErrOOB，实际 %v", err)
	}

	// 多目标中有一个不可走，定位到那个目标而非第一个。
	_, err = BuildField(g, Point{0, 0}, Point{1, 1})
	if !errors.As(err, &ce) || ce.At != (Point{1, 1}) || !errors.Is(err, ErrTargetBlocked) {
		t.Fatalf("应定位到坏目标 (1,1)，实际 %v", err)
	}

	// 无目标。
	if _, err := BuildField(g); !errors.Is(err, ErrNoTargets) {
		t.Fatalf("无目标应报 ErrNoTargets，实际 %v", err)
	}

	// 梯度为零：墙格出发、越界出发、以及孤立可达域中的格。
	f, err := BuildField(g, Point{0, 0})
	if err != nil {
		t.Fatal(err)
	}
	if _, _, err := f.Follow(Point{1, 1}); !errors.Is(err, ErrZeroGradient) ||
		!errors.As(err, &ce) || ce.At != (Point{1, 1}) {
		t.Fatalf("墙格出发应报 ErrZeroGradient 及坐标，实际 %v", err)
	}
	if _, _, err := f.Follow(Point{-1, 0}); !errors.Is(err, ErrOOB) {
		t.Fatalf("越界出发应报 ErrOOB，实际 %v", err)
	}

	g2, _ := Parse([]string{"...", "###", "..."})
	f2, err := BuildField(g2, Point{0, 0})
	if err != nil {
		t.Fatal(err)
	}
	if _, _, err := f2.Follow(Point{0, 2}); !errors.Is(err, ErrZeroGradient) ||
		!errors.As(err, &ce) || ce.At != (Point{0, 2}) {
		t.Fatalf("不可达区域应报 ErrZeroGradient 及坐标，实际 %v", err)
	}
}

// forcedField 构造一个没有更近邻居的格，用于验证回退路径。
func forcedField(t *testing.T) *Field {
	t.Helper()
	g, _ := Parse([]string{"...", "...", "..."})
	return &Field{
		g:        g,
		dist:     []int{0, 1, 2, 1, 2, 3, 2, 2, 3},
		isTarget: map[Point]struct{}{{0, 0}: {}},
		targets:  []Point{{0, 0}},
		size:     9,
	}
}

func TestFollowFallbackLocalMin(t *testing.T) {
	f := forcedField(t)
	// (1,2)=2 的三个邻居 (0,2)=2,(2,2)=3,(1,1)=2 都不更小，构成局部极小。
	if _, ok := f.lowerNeighbor(Point{1, 2}); ok {
		t.Fatal("构造应产生局部极小")
	}
	path, fb, err := f.Follow(Point{1, 2})
	if err != nil {
		t.Fatal(err)
	}
	if fb != 1 || f.Fallbacks() != 1 {
		t.Fatalf("应记录 1 次回退，实际 %d / %d", fb, f.Fallbacks())
	}
	if path[len(path)-1] != (Point{0, 0}) {
		t.Fatalf("回退后终点错误: %v", path)
	}
	for i := 1; i < len(path); i++ {
		if abs(path[i].X-path[i-1].X)+abs(path[i].Y-path[i-1].Y) != 1 {
			t.Fatalf("回退路线不连续: %v", path)
		}
	}
	// 正常格不受影响，场级计数累加。
	if _, fb2, err := f.Follow(Point{2, 0}); err != nil || fb2 != 0 {
		t.Fatalf("正常格不应回退: %v %d", err, fb2)
	}
	if f.Fallbacks() != 1 {
		t.Fatalf("总回退应为 1，实际 %d", f.Fallbacks())
	}
}

func TestFieldSizeAndStats(t *testing.T) {
	g, _ := Parse([]string{
		".....",
		".###.",
		".....",
	})
	f, err := BuildField(g, Point{0, 0})
	if err != nil {
		t.Fatal(err)
	}
	st := f.Stats()
	if st.Walkable != 12 || st.Covered != 12 || st.Covered != f.Size() || st.Targets != 1 {
		t.Fatalf("统计异常: %+v size=%d", st, f.Size())
	}
	if err := f.CheckTriangle(); err != nil {
		t.Fatal(err)
	}
	if err := f.CheckDescent(); err != nil {
		t.Fatal(err)
	}
	// 每个格子的距离必须等于到目标的 BFS 距离（用 A* 路径长度-1 交叉验证）。
	for _, s := range allWalkable(g) {
		p, _, err := g.Path(s, Point{0, 0})
		if err != nil {
			t.Fatalf("Path(%v) 失败: %v", s, err)
		}
		if d, _ := f.Dist(s); d != len(p)-1 {
			t.Fatalf("%v 场距离 %d 与 A* 距离 %d 不一致", s, d, len(p)-1)
		}
	}
}

func TestFieldManyUnitsIndependent(t *testing.T) {
	rows := make([]string, 12)
	for y := range rows {
		line := make([]byte, 12)
		for x := range line {
			if x == 6 && y != 6 {
				line[x] = '#'
			} else {
				line[x] = '.'
			}
		}
		rows[y] = string(line)
	}
	g, err := Parse(rows)
	if err != nil {
		t.Fatal(err)
	}
	f, err := BuildField(g, Point{11, 11})
	if err != nil {
		t.Fatal(err)
	}
	if err := f.CheckDescent(); err != nil {
		t.Fatal(err)
	}
	units := 0
	for _, s := range allWalkable(g) {
		followAndVerify(t, f, s)
		units++
	}
	if units != 133 {
		t.Fatalf("单位数 %d", units)
	}
	if f.Fallbacks() != 0 {
		t.Fatalf("回退 %d", f.Fallbacks())
	}
	fmt.Printf("距离场规模=%d 回退次数=%d\n", f.Size(), f.Fallbacks())
}
