package grid

import (
	"errors"
	"fmt"
	"math"
	"testing"
)

// 样例：宽廊、窄门、拐角、死路、开阔地。
var (
	// 宽廊：9 行宽的长廊，中央一根柱子。半径 1 时必须绕开柱子。
	wideCorridorRows = []string{
		"#############",
		"#...........#",
		"#...........#",
		"#...........#",
		"#.....#.....#",
		"#...........#",
		"#...........#",
		"#...........#",
		"#############",
	}
	// 窄门：上下两室由 5 格宽的门相连；半径 1 时只剩中间 3 格能过。
	narrowDoorRows = []string{
		"###########",
		"#.........#",
		"#.........#",
		"###.....###",
		"#.........#",
		"#.........#",
		"#.........#",
		"#.........#",
		"###########",
	}
	// 拐角：宽度 5 的 L 形走廊，半径 1 时走内侧中线转弯。
	cornerRows = []string{
		"############",
		"#.....######",
		"#.....######",
		"#.....######",
		"#.....######",
		"#.....######",
		"#..........#",
		"#..........#",
		"#..........#",
		"#..........#",
		"#..........#",
		"############",
	}
	// 死路：主厅下方一条 3 格宽的盲道；尽头格距墙 1 格，
	// 原网格可达，但半径 1 膨胀后尽头被堵，实体进不去。
	deadEndRows = []string{
		"###########",
		"#.........#",
		"#.........#",
		"#.........#",
		"#.........#",
		"####...####",
		"####...####",
		"####...####",
		"####...####",
		"####...####",
		"####...####",
		"###########",
	}
	// 开阔地：15x15，中央单柱，半径 2。
	openFieldRows = []string{
		"###############",
		"#.............#",
		"#.............#",
		"#.............#",
		"#.............#",
		"#.............#",
		"#.............#",
		"#......#......#",
		"#.............#",
		"#.............#",
		"#.............#",
		"#.............#",
		"#.............#",
		"#.............#",
		"###############",
	}
)

func mustParse(t *testing.T, rows []string) *Grid {
	t.Helper()
	g, err := Parse(rows)
	if err != nil {
		t.Fatalf("解析失败: %v", err)
	}
	return g
}

func TestRoundsForRadius(t *testing.T) {
	cases := []struct {
		radius float64
		want   int
	}{
		{0, 0}, {0.2, 1}, {1, 1}, {1.01, 2}, {2, 2}, {2.5, 3},
	}
	for _, c := range cases {
		got, err := RoundsForRadius(c.radius)
		if err != nil {
			t.Fatalf("半径 %g 不应报错: %v", c.radius, err)
		}
		if got != c.want {
			t.Errorf("半径 %g 折算轮数 = %d, 期望 %d", c.radius, got, c.want)
		}
	}
	for _, bad := range []float64{-1, -0.001, math.NaN(), math.Inf(1)} {
		_, err := RoundsForRadius(bad)
		if !errors.Is(err, ErrRadius) {
			t.Errorf("半径 %g 应返回 ErrRadius, 实际 %v", bad, err)
		}
		var re *RadiusError
		if !errors.As(err, &re) {
			t.Errorf("半径 %g 应带 RadiusError", bad)
		}
	}
}

func TestInflateRoundsAndCount(t *testing.T) {
	g := mustParse(t, []string{"#####", ".....", "..#..", ".....", "#####"})
	before := g.WalkableCount()
	inflated, rounds, err := g.Inflate(1)
	if err != nil {
		t.Fatal(err)
	}
	want, _ := RoundsForRadius(1)
	if rounds != want {
		t.Fatalf("膨胀轮数 %d != 折算 %d", rounds, want)
	}
	if inflated.WalkableCount() >= before {
		t.Fatalf("膨胀后应更小: %d -> %d", before, inflated.WalkableCount())
	}
	if g.WalkableCount() != before {
		t.Fatal("膨胀不应修改原网格")
	}
	if _, _, err := (*Grid)(nil).Inflate(1); !errors.Is(err, ErrEmpty) {
		t.Fatalf("空网格应报 ErrEmpty, 实际 %v", err)
	}
	// 实测轮次不得多于折算值；手工传入更大值应被 CheckRounds 拒绝。
	if _, err := CheckRounds(rounds, 1); err != nil {
		t.Fatalf("实测轮次等于折算值应通过: %v", err)
	}
	if _, err := CheckRounds(rounds+1, 1); !errors.Is(err, ErrRounds) {
		t.Fatalf("实测轮次多于折算应报 ErrRounds, 实际 %v", err)
	}
	// 极大半径饱和截断，仍不多于折算值。
	if _, rounds, err := g.Inflate(1e9); err != nil || rounds > g.W+g.H {
		t.Fatalf("极大半径应安全饱和: rounds=%d err=%v", rounds, err)
	}
}

func TestPathWideSamples(t *testing.T) {
	cases := []struct {
		name           string
		rows           []string
		from, to       Point
		radius, detour float64
		wantErr        error
	}{
		{"宽廊", wideCorridorRows, Point{2, 4}, Point{10, 4}, 1, 0.5, nil},
		{"窄门", narrowDoorRows, Point{5, 2}, Point{5, 6}, 1, 0.5, nil},
		{"拐角", cornerRows, Point{3, 2}, Point{8, 8}, 1, 0.5, nil},
		{"死路", deadEndRows, Point{2, 2}, Point{5, 10}, 1, 0.5, ErrBlocked},
		{"开阔地", openFieldRows, Point{3, 3}, Point{11, 11}, 2, 0.5, nil},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			g := mustParse(t, c.rows)
			res, err := g.PathWide(c.from, c.to, c.radius, c.detour)
			if c.wantErr != nil {
				if !errors.Is(err, c.wantErr) {
					t.Fatalf("期望 %v, 实际 %v (res=%+v)", c.wantErr, err, res)
				}
				var be *BlockedError
				if !errors.As(err, &be) || be.Point != c.to {
					t.Fatalf("堵塞应报终点坐标 %v, 实际 %v", c.to, err)
				}
				return
			}
			if err != nil {
				t.Fatalf("不应报错: %v", err)
			}
			wantRounds, _ := RoundsForRadius(c.radius)
			if res.Rounds != wantRounds {
				t.Fatalf("轮数 %d != 折算 %d", res.Rounds, wantRounds)
			}
			if _, err := CheckRounds(res.Rounds, c.radius); err != nil {
				t.Fatalf("轮次核验失败: %v", err)
			}
			if res.MinDistance < c.radius-1e-9 {
				t.Fatalf("最小距离 %.4f < 半径 %g", res.MinDistance, c.radius)
			}
			if res.Detour > c.detour+1e-9 {
				t.Fatalf("增幅 %.4f > 允许 %g", res.Detour, c.detour)
			}
			// 路径每一步都必须落在膨胀后网格的可走格上。
			inflated, _, _ := g.Inflate(c.radius)
			for _, p := range res.Path {
				ok, err := inflated.Walkable(p)
				if err != nil || !ok {
					t.Fatalf("路径经过膨胀后不可走格 %v", p)
				}
			}
			// 两条不变量各自独立复核一遍。
			if d, err := CheckClearance(g, res.Path, c.radius); err != nil || d != res.MinDistance {
				t.Fatalf("独立核验最小距离不一致: %v %g", err, d)
			}
			if ratio, err := CheckDetour(res.NarrowSteps, res.WideSteps, c.detour); err != nil || ratio != res.Detour {
				t.Fatalf("独立核验增幅不一致: %v %g", err, ratio)
			}
		})
	}
}

func TestPathWideNarrowCorridorCentered(t *testing.T) {
	// 3 格宽长廊，半径 1 时必须走正中一行，不能擦墙。
	g := mustParse(t, []string{
		"#############",
		"#...........#",
		"#...........#",
		"#...........#",
		"#############",
	})
	res, err := g.PathWide(Point{2, 2}, Point{10, 2}, 1, 0)
	if err != nil {
		t.Fatal(err)
	}
	for _, p := range res.Path {
		if p.Y != 2 {
			t.Fatalf("应贴中线行驶，实际经过 %v", p)
		}
	}
	if res.MinDistance < 1 {
		t.Fatalf("最小距离 %.4f < 1", res.MinDistance)
	}
}

func TestPathWideErrorsWithCoords(t *testing.T) {
	g := mustParse(t, []string{"...", ".#.", "..."})

	// 半径为负。
	var re *RadiusError
	if _, err := g.PathWide(Point{0, 0}, Point{2, 2}, -1, 0.5); !errors.As(err, &re) || re.Radius != -1 {
		t.Fatalf("负半径应带 RadiusError, 实际 %v", err)
	}

	// 空网格：nil、零尺寸、无行。
	if _, err := (*Grid)(nil).PathWide(Point{0, 0}, Point{0, 0}, 1, 0.5); !errors.Is(err, ErrEmpty) {
		t.Fatalf("nil 网格应报 ErrEmpty, 实际 %v", err)
	}
	if _, err := (&Grid{}).PathWide(Point{0, 0}, Point{0, 0}, 1, 0.5); !errors.Is(err, ErrEmpty) {
		t.Fatalf("零尺寸网格应报 ErrEmpty, 实际 %v", err)
	}

	// 越界：起点与终点分别报出坐标。
	for _, c := range []struct {
		from, to Point
		want     Point
	}{
		{Point{-1, 0}, Point{0, 0}, Point{-1, 0}},
		{Point{0, 0}, Point{3, 2}, Point{3, 2}},
	} {
		var be *BoundsError
		_, err := g.PathWide(c.from, c.to, 0, 0.5)
		if !errors.As(err, &be) || be.Point != c.want {
			t.Fatalf("越界应报坐标 %v, 实际 %v", c.want, err)
		}
		if !errors.Is(err, ErrOOB) {
			t.Fatalf("错误码应为 ErrOOB: %v", err)
		}
	}

	// 堵塞：原网格里端点就是墙，错误带出堵点坐标。
	var blockedErr *BlockedError
	_, err := g.PathWide(Point{0, 0}, Point{1, 1}, 0, 0.5)
	if !errors.Is(err, ErrBlocked) || !errors.As(err, &blockedErr) || blockedErr.Point != (Point{1, 1}) {
		t.Fatalf("终点被堵应报坐标 (1,1), 实际 %v", err)
	}

	// 堵塞：起终点都被堵时两条错误各自带坐标。
	_, err = g.PathWide(Point{1, 1}, Point{1, 1}, 0, 0.5)
	if !errors.Is(err, ErrBlocked) {
		t.Fatalf("堵点应报 ErrBlocked, 实际 %v", err)
	}
	if got := len(collectBlocked(err)); got != 2 {
		t.Fatalf("起终点都被堵应报 2 个坐标, 实际 %d: %v", got, err)
	}
}

func TestPathWideInflatedBlockedAndDisconnected(t *testing.T) {
	// 端点原可走，膨胀后被堵。
	g := mustParse(t, []string{"#####", "#...#", "#####"})
	_, err := g.PathWide(Point{2, 1}, Point{1, 1}, 2, 1)
	if !errors.Is(err, ErrBlocked) {
		t.Fatalf("膨胀后堵塞应报 ErrBlocked, 实际 %v", err)
	}
	if got := collectBlocked(err); !containsPoint(got, Point{1, 1}) {
		t.Fatalf("膨胀后堵塞应带终点坐标 (1,1): %v", err)
	}

	// 端点都在膨胀后存活，但柱子膨胀成整宽屏障导致不连通。
	barrier := mustParse(t, []string{
		"###########",
		"###########",
		"#.........#",
		"#....#....#",
		"#.........#",
		"###########",
	})
	res, err := barrier.PathWide(Point{2, 3}, Point{8, 3}, 1, 10)
	if !errors.Is(err, ErrNoPath) {
		t.Fatalf("膨胀后不连通应报 ErrNoPath, 实际 %v", err)
	}
	if res == nil || res.Path != nil || res.WideSteps != 0 {
		t.Fatalf("不连通时结果应可读且无路径: %+v", res)
	}
}

func TestCheckClearanceAndDetourIndependent(t *testing.T) {
	g := mustParse(t, []string{"#####", "#...#", "#####"})
	// 擦墙路径：最小距离 1 < 半径 1.5。
	path := []Point{{1, 1}, {2, 1}, {3, 1}}
	if d, err := CheckClearance(g, path, 1.5); !errors.Is(err, ErrClearance) || d != 1 {
		t.Fatalf("应报距离不足, d=%g err=%v", d, err)
	}
	if d, err := CheckClearance(g, path, 1); err != nil || d != 1 {
		t.Fatalf("半径 1 应通过, d=%g err=%v", d, err)
	}
	// 增幅：10 -> 16 为 60%，允许 50% 时应失败。
	if r, err := CheckDetour(10, 16, 0.5); !errors.Is(err, ErrDetour) || math.Abs(r-0.6) > 1e-9 {
		t.Fatalf("应报增幅超限, r=%g err=%v", r, err)
	}
	if _, err := CheckDetour(10, 15, 0.5); err != nil {
		t.Fatalf("50%% 应通过: %v", err)
	}
	if _, err := CheckDetour(0, 0, 0); err != nil {
		t.Fatalf("零步应通过: %v", err)
	}
}

func TestNarrowAPIUnchanged(t *testing.T) {
	g := mustParse(t, []string{"...", ".#.", "..."})
	path, visited, err := g.Path(Point{0, 0}, Point{2, 2})
	if err != nil {
		t.Fatal(err)
	}
	if len(path) != 5 || visited == 0 {
		t.Fatalf("窄身接口行为变化: %v %d", path, visited)
	}
	if _, err := Parse([]string{"..", "..."}); !errors.Is(err, ErrShape) {
		t.Fatal("ErrShape 语义变化")
	}
	if _, err := Parse(nil); !errors.Is(err, ErrEmpty) {
		t.Fatal("ErrEmpty 语义变化")
	}
}

func collectBlocked(err error) []*BlockedError {
	var out []*BlockedError
	for _, e := range flatten(err) {
		var be *BlockedError
		if errors.As(e, &be) {
			out = append(out, be)
		}
	}
	return out
}

func containsPoint(list []*BlockedError, p Point) bool {
	for _, be := range list {
		if be.Point == p {
			return true
		}
	}
	return false
}

func flatten(err error) []error {
	if err == nil {
		return nil
	}
	if joined, ok := err.(interface{ Unwrap() []error }); ok {
		var out []error
		for _, e := range joined.Unwrap() {
			out = append(out, flatten(e)...)
		}
		return out
	}
	return []error{err}
}

// 输出宽度约束样例的最小距离与轮数表。
// 运行：go test ./... -run Example_wideSamples -v
func Example_wideSamples() {
	samples := []struct {
		name     string
		rows     []string
		from, to Point
		radius   float64
	}{
		{"宽廊", wideCorridorRows, Point{2, 4}, Point{10, 4}, 1},
		{"窄门", narrowDoorRows, Point{5, 2}, Point{5, 6}, 1},
		{"拐角", cornerRows, Point{3, 2}, Point{8, 8}, 1},
		{"死路", deadEndRows, Point{2, 2}, Point{5, 10}, 1},
		{"开阔地", openFieldRows, Point{3, 3}, Point{11, 11}, 2},
	}
	for _, s := range samples {
		g, err := Parse(s.rows)
		if err != nil {
			panic(err)
		}
		res, err := g.PathWide(s.from, s.to, s.radius, 0.5)
		if err != nil {
			fmt.Printf("%s radius=%g rounds=%d blocked=%v\n",
				s.name, s.radius, derivedRounds(s.radius), err)
			continue
		}
		fmt.Printf("%s radius=%g rounds=%d min_dist=%.2f narrow=%d wide=%d\n",
			s.name, res.Radius, res.Rounds, res.MinDistance, res.NarrowSteps, res.WideSteps)
	}
	// Output:
	// 宽廊 radius=1 rounds=1 min_dist=2.00 narrow=10 wide=12
	// 窄门 radius=1 rounds=1 min_dist=2.00 narrow=4 wide=4
	// 拐角 radius=1 rounds=1 min_dist=2.00 narrow=11 wide=11
	// 死路 radius=1 rounds=1 blocked=起终点被堵: 膨胀后终点 (5,10)
	// 开阔地 radius=2 rounds=2 min_dist=3.00 narrow=16 wide=16
}

func derivedRounds(radius float64) int {
	n, _ := RoundsForRadius(radius)
	return n
}

func TestPathWideDetourExceeded(t *testing.T) {
	// 宽廊窄身 10 步、宽身 12 步，增幅 20%；允许 10% 时应报 ErrDetour，
	// 且结果仍带出实测距离与轮次，便于登记。
	g := mustParse(t, wideCorridorRows)
	res, err := g.PathWide(Point{2, 4}, Point{10, 4}, 1, 0.1)
	if !errors.Is(err, ErrDetour) {
		t.Fatalf("增幅超限应报 ErrDetour, 实际 %v", err)
	}
	if res == nil || res.MinDistance < 1 || res.Rounds != derivedRounds(1) {
		t.Fatalf("失败结果应可读: %+v", res)
	}
	if res.Detour <= 0.1 {
		t.Fatalf("实测增幅 %.4f 应大于 10%%", res.Detour)
	}
}
