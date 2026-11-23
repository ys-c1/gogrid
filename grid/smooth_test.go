package grid

import (
	"errors"
	"math"
	"math/rand"
	"testing"
)

// checkInvariants 核验平滑的全部硬性不变量。
func checkInvariants(t *testing.T, g *Grid, res *SmoothResult, kappaMax float64) {
	t.Helper()
	n := len(res.Original)
	// 1. 平滑路径是原始路径的子序列（端点不动）
	i, j := 0, 0
	for ; j < len(res.Smoothed); j++ {
		for i < n && res.Original[i] != res.Smoothed[j] {
			i++
		}
		if i >= n {
			t.Fatalf("平滑路径点 %v 不是原始路径的子序列", res.Smoothed[j])
		}
		i++
	}
	if res.Smoothed[0] != res.Original[0] || res.Smoothed[len(res.Smoothed)-1] != res.Original[n-1] {
		t.Fatal("平滑后起终点发生变化")
	}

	// 2. 长度不得增加
	if res.LengthAfter > res.LengthBefore+epsSmooth {
		t.Fatalf("长度增加: %.4f -> %.4f", res.LengthBefore, res.LengthAfter)
	}
	if got := polylineLength(res.Smoothed); math.Abs(got-res.LengthAfter) > epsSmooth {
		t.Fatalf("回填长度不一致 %.4f != %.4f", got, res.LengthAfter)
	}

	// 3. 每条新弦独立满足视线与间距（不依赖整体结论）
	blocked := g.blockedCells()
	d0 := res.ClearanceBefore
	for q := 1; q < len(res.Smoothed); q++ {
		ok, err := g.Visible(res.Smoothed[q-1], res.Smoothed[q])
		if err != nil || !ok {
			t.Fatalf("弦 %v->%v 视线不成立", res.Smoothed[q-1], res.Smoothed[q])
		}
		a, b := toFP(res.Smoothed[q-1]), toFP(res.Smoothed[q])
		d := math.Inf(1)
		for _, c := range blocked {
			d = math.Min(d, distSegmentSquare(a, b, c))
		}
		if d < d0-epsSmooth {
			t.Fatalf("弦间距 %.4f 小于原始最小间距 %.4f", d, d0)
		}
	}

	// 4. 整体精确间距不下降，且逐点采样核验通过
	if res.ClearanceAfter < d0-epsSmooth {
		t.Fatalf("整体间距变小: %.4f -> %.4f", d0, res.ClearanceAfter)
	}
	if !res.PointwiseOK || res.SampledMinClearance < d0-epsSmooth {
		t.Fatalf("逐点核验失败: ok=%v 采样最小间距=%.4f 原始=%.4f",
			res.PointwiseOK, res.SampledMinClearance, d0)
	}
	if res.SamplePoints < len(res.Smoothed) {
		t.Fatalf("采样点数异常 %d", res.SamplePoints)
	}

	// 5. 曲率：非几何受限的保留拐角必须满足上界；受限标记与数值自洽
	boundViolations := 0
	for _, st := range res.Corners {
		if !st.Retained {
			if st.KappaAfter != 0 || st.KUsed != 0 {
				t.Fatalf("消除拐角回填异常: %+v", st)
			}
			continue
		}
		if st.KappaAfter > kappaMax+epsSmooth {
			boundViolations++
			if !st.GeometryBound {
				t.Fatalf("拐角 %v 曲率 %.4f 超上界 %.4f 却未标记几何受限",
					st.Point, st.KappaAfter, kappaMax)
			}
			if st.KUsed >= st.KNeed {
				t.Fatalf("拐角 %v 步数已满足反解却仍超界: %+v", st.Point, st)
			}
		} else if st.GeometryBound {
			t.Fatalf("拐角 %v 未超界却被标记几何受限", st.Point)
		}
	}
	if boundViolations != res.GeometryBoundN {
		t.Fatalf("几何受限计数不一致 %d != %d", boundViolations, res.GeometryBoundN)
	}
	if res.GeometryBoundN == 0 && res.MaxKappaAfter > kappaMax+epsSmooth {
		t.Fatalf("最大曲率 %.4f 超过上界 %.4f", res.MaxKappaAfter, kappaMax)
	}
}

// 用例 1：直廊——无拐角，路径与长度保持不变。
func TestSmoothStraightCorridor(t *testing.T) {
	g, _ := Parse([]string{
		"#########",
		"#.......#",
		"#########",
	})
	path, _, err := g.Path(Point{1, 1}, Point{7, 1})
	if err != nil {
		t.Fatal(err)
	}
	res, err := g.Smooth(path)
	if err != nil {
		t.Fatal(err)
	}
	if len(res.Corners) != 0 {
		t.Fatalf("直廊不应产生拐角，实际 %d 个", len(res.Corners))
	}
	if len(res.Smoothed) != 2 {
		t.Fatalf("共线点应合并为一条直弦，实际保留 %d 个点", len(res.Smoothed))
	}
	if res.MaxKappaAfter != 0 {
		t.Fatalf("直弦曲率应为 0，实际 %.4f", res.MaxKappaAfter)
	}
	if math.Abs(res.LengthAfter-res.LengthBefore) > epsSmooth {
		t.Fatal("直廊拉直后长度应不变")
	}
	checkInvariants(t, g, res, DefaultKappaMax)
}

// 用例 2：拐角——L 形单宽走廊，直角拐角几何受限，必须保留并显式上报。
func TestSmoothCorner(t *testing.T) {
	g, _ := Parse([]string{
		"#####",
		"#...#",
		"###.#",
		"#...#",
		"#####",
	})
	path, _, err := g.Path(Point{1, 1}, Point{1, 3})
	if err != nil {
		t.Fatal(err)
	}
	res, err := g.Smooth(path)
	if err != nil {
		t.Fatal(err)
	}
	if len(res.Corners) != 2 {
		t.Fatalf("应有两个直角拐角，实际 %d", len(res.Corners))
	}
	for _, st := range res.Corners {
		if math.Abs(st.TurnAngle-math.Pi/2) > epsAngle {
			t.Fatalf("转角应为 π/2，实际 %.4f", st.TurnAngle)
		}
		if !st.Retained {
			t.Fatalf("窄廊拐角必须保留: %+v", st)
		}
		if st.KappaAfter >= st.KappaBefore-epsSmooth {
			t.Fatalf("拉直后曲率应严格下降: %.4f -> %.4f", st.KappaBefore, st.KappaAfter)
		}
		// 单宽走廊内无法继续拉直：要么已达上界，要么被几何强制受限
		if st.GeometryBound && st.KappaAfter <= DefaultKappaMax+epsSmooth {
			t.Fatalf("未超上界不应标记受限: %+v", st)
		}
		if !st.GeometryBound && st.KappaAfter > DefaultKappaMax+epsSmooth {
			t.Fatalf("超上界却未标记受限: %+v", st)
		}
	}
	checkInvariants(t, g, res, DefaultKappaMax)
}

// 用例 3：开阔地——阶梯路径被大幅拉直，长度严格下降，曲率全部满足上界。
func TestSmoothOpenField(t *testing.T) {
	g, _ := Parse([]string{
		"........",
		"........",
		"........",
		"........",
		"........",
		"........",
		"........",
		"........",
	})
	path, _, err := g.Path(Point{0, 0}, Point{7, 7})
	if err != nil {
		t.Fatal(err)
	}
	if len(path) < 10 {
		t.Fatalf("前置条件不成立：A* 应给出贴格阶梯路径，实际 %d 点", len(path))
	}
	res, err := g.Smooth(path)
	if err != nil {
		t.Fatal(err)
	}
	if res.LengthAfter >= res.LengthBefore-epsSmooth {
		t.Fatalf("开阔地应变短: %.4f -> %.4f", res.LengthBefore, res.LengthAfter)
	}
	if res.GeometryBoundN != 0 {
		t.Fatal("无障碍时不应存在几何受限拐角")
	}
	if len(res.Smoothed) != 2 {
		t.Fatalf("开阔地应收敛为起终点直弦，实际 %d 个点", len(res.Smoothed))
	}
	if res.MaxKappaAfter != 0 {
		t.Fatalf("直弦曲率应为 0，实际 %.4f", res.MaxKappaAfter)
	}
	checkInvariants(t, g, res, DefaultKappaMax)
}

// 用例 4：窄门——门口附近拐角受间距约束，间距不变量逐点成立。
func TestSmoothNarrowDoor(t *testing.T) {
	g, _ := Parse([]string{
		"#########",
		"#.......#",
		"#.......#",
		"####.####",
		"#.......#",
		"#.......#",
		"#########",
	})
	path, _, err := g.Path(Point{1, 1}, Point{7, 5})
	if err != nil {
		t.Fatal(err)
	}
	res, err := g.Smooth(path)
	if err != nil {
		t.Fatal(err)
	}
	door := Point{4, 3}
	through := false
	for q := 1; q < len(res.Smoothed); q++ {
		if distSegmentSquare(toFP(res.Smoothed[q-1]), toFP(res.Smoothed[q]), door) == 0 {
			through = true // 线段穿过门洞格方块
		}
	}
	if !through {
		t.Fatal("平滑后折线必须从窄门 (4,3) 通过")
	}
	// 门口受间距约束（弦不能压到 0.5 以下），但拉直后最大曲率应显著低于上界
	if res.MaxKappaAfter >= DefaultKappaMax {
		t.Fatalf("窄门拐角曲率 %.4f 不应超过上界 %.4f", res.MaxKappaAfter, DefaultKappaMax)
	}
	if res.MaxKappaAfter >= res.MaxKappaBefore-epsSmooth {
		t.Fatal("窄门场景平滑后曲率应下降")
	}
	if res.LengthAfter >= res.LengthBefore-epsSmooth {
		t.Fatal("窄门场景长度应下降")
	}
	// 门口贴墙，原始最小间距为 0.5，平滑后不得变小
	if res.ClearanceBefore < 0.5-epsSmooth || res.ClearanceAfter < 0.5-epsSmooth {
		t.Fatalf("间距异常: before=%.4f after=%.4f", res.ClearanceBefore, res.ClearanceAfter)
	}
	checkInvariants(t, g, res, DefaultKappaMax)
}

// genMaze 用递归回溯生成连通迷宫（奇数尺寸），保证任意空格之间有路。
func genMaze(w, h int, seed int64) []string {
	rng := rand.New(rand.NewSource(seed))
	cells := make([][]byte, h)
	for y := range cells {
		cells[y] = make([]byte, w)
		for x := range cells[y] {
			cells[y][x] = '#'
		}
	}
	var carve func(x, y int)
	carve = func(x, y int) {
		cells[y][x] = '.'
		dirs := [][2]int{{2, 0}, {-2, 0}, {0, 2}, {0, -2}}
		rng.Shuffle(4, func(i, j int) { dirs[i], dirs[j] = dirs[j], dirs[i] })
		for _, d := range dirs {
			nx, ny := x+d[0], y+d[1]
			if nx <= 0 || ny <= 0 || nx >= w-1 || ny >= h-1 || cells[ny][nx] == '.' {
				continue
			}
			cells[y+d[1]/2][x+d[0]/2] = '.'
			carve(nx, ny)
		}
	}
	carve(1, 1)
	rows := make([]string, h)
	for y := range cells {
		rows[y] = string(cells[y])
	}
	return rows
}

// 用例 5：迷宫——多组起终点，全部核验不变量与曲率上界。
func TestSmoothMaze(t *testing.T) {
	rows := genMaze(21, 15, 20261124)
	g, err := Parse(rows)
	if err != nil {
		t.Fatal(err)
	}
	var floors []Point
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if g.cells[y][x] {
				floors = append(floors, Point{x, y})
			}
		}
	}
	rng := rand.New(rand.NewSource(77))
	cases := 0
	for cases < 12 {
		a := floors[rng.Intn(len(floors))]
		b := floors[rng.Intn(len(floors))]
		if a == b {
			continue
		}
		path, _, perr := g.Path(a, b)
		if perr != nil {
			t.Fatalf("迷宫连通却报无路: %v", perr)
		}
		res, serr := g.Smooth(path)
		if serr != nil {
			t.Fatalf("平滑失败 (%v->%v): %v", a, b, serr)
		}
		checkInvariants(t, g, res, DefaultKappaMax)
		cases++
	}
}

// 随机网格模糊测试：任意可行路径平滑后不变量都必须成立。
func TestSmoothRandomGrids(t *testing.T) {
	rng := rand.New(rand.NewSource(42))
	for trial := 0; trial < 40; trial++ {
		w, h := 6+rng.Intn(14), 6+rng.Intn(14)
		rows := make([]string, h)
		for y := 0; y < h; y++ {
			b := make([]byte, w)
			for x := 0; x < w; x++ {
				if rng.Float64() < 0.25 {
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
		for attempt := 0; attempt < 6; attempt++ {
			from := Point{rng.Intn(w), rng.Intn(h)}
			to := Point{rng.Intn(w), rng.Intn(h)}
			path, _, perr := g.Path(from, to)
			if perr != nil {
				continue
			}
			res, serr := g.Smooth(path)
			if serr != nil {
				t.Fatalf("trial=%d 平滑失败: %v", trial, serr)
			}
			checkInvariants(t, g, res, DefaultKappaMax)
		}
	}
}

// 退化输入：空路径报错；单点与两点路径原样返回且核验通过。
func TestSmoothDegenerate(t *testing.T) {
	g, _ := Parse([]string{"...", "...", "..."})
	if _, err := g.Smooth(nil); err == nil {
		t.Fatal("空路径应报错")
	}
	res, err := g.Smooth([]Point{{X: 1, Y: 1}})
	if err != nil {
		t.Fatal(err)
	}
	if len(res.Smoothed) != 1 || !res.PointwiseOK {
		t.Fatalf("单点路径处理异常: %+v", res)
	}
	res, err = g.Smooth([]Point{{X: 0, Y: 0}, {X: 2, Y: 2}})
	if err != nil {
		t.Fatal(err)
	}
	if len(res.Smoothed) != 2 || !res.PointwiseOK {
		t.Fatalf("两点路径处理异常: %+v", res)
	}
	// 输入含不可走点应报带坐标错误
	gw, _ := Parse([]string{"...", ".#.", "..."})
	if _, err := gw.SmoothAt([]Point{{X: 0, Y: 0}, {X: 1, Y: 1}}, DefaultKappaMax); err == nil {
		t.Fatal("含墙格路径应报错")
	}
}

// 紧上界：L 形单宽走廊在 π/8 下无法满足，拐角必须标记几何受限并如实给出曲率。
func TestSmoothTightBoundGeometryLimit(t *testing.T) {
	g, _ := Parse([]string{
		"#####",
		"#...#",
		"###.#",
		"#...#",
		"#####",
	})
	path, _, err := g.Path(Point{X: 1, Y: 1}, Point{X: 1, Y: 3})
	if err != nil {
		t.Fatal(err)
	}
	tight := math.Pi / 8
	res, err := g.SmoothAt(path, tight)
	if err != nil {
		t.Fatal(err)
	}
	if res.GeometryBoundN == 0 {
		t.Fatal("紧上界下应出现几何受限拐角")
	}
	for _, st := range res.Corners {
		if st.Retained && st.KappaAfter > tight+epsSmooth && !st.GeometryBound {
			t.Fatalf("超界拐角必须标记几何受限: %+v", st)
		}
	}
	// 即便上界无法满足，硬性不变量仍然成立
	checkInvariants(t, g, res, tight)
}

// 自定义上界：更紧的上界只改变反解步数，不破坏不变量。
func TestSmoothCustomBound(t *testing.T) {
	g, _ := Parse([]string{
		"........",
		"........",
		"........",
		"........",
		"........",
		"........",
		"........",
		"........",
	})
	path, _, _ := g.Path(Point{0, 0}, Point{7, 7})
	for _, kmax := range []float64{math.Pi / 8, math.Pi / 4, math.Pi / 2} {
		res, err := g.SmoothAt(path, kmax)
		if err != nil {
			t.Fatalf("上界 %.4f 平滑失败: %v", kmax, err)
		}
		checkInvariants(t, g, res, kmax)
	}
	if _, err := g.SmoothAt(path, 0); err == nil {
		t.Fatal("非正上界应报错")
	}
}

// 错误上报：无路、起终点相同、必经点不可走/越界，均携带坐标。
func TestPathViaErrors(t *testing.T) {
	g, _ := Parse([]string{
		"#####",
		"##.##",
		"#.#.#",
		"#..##",
		"#####",
	})
	// (2,1) 是被墙体完全围死的孤立空格，与 (2,3) 无路可达
	_, _, err := g.PathVia(Point{2, 1}, Point{2, 3})
	var np *NoPathError
	if !errors.As(err, &np) {
		t.Fatalf("应返回 *NoPathError，实际 %v", err)
	}
	if np.From != (Point{2, 1}) || np.To != (Point{2, 3}) {
		t.Fatalf("无路坐标错误: %+v", np)
	}
	if !errors.Is(err, ErrNoPath) {
		t.Fatal("NoPathError 应兼容 ErrNoPath")
	}

	// 起终点相同
	_, _, err = g.PathVia(Point{2, 3}, Point{2, 3})
	var sp *SamePointError
	if !errors.As(err, &sp) || sp.Point != (Point{2, 3}) {
		t.Fatalf("同点错误坐标不对: %v", err)
	}

	// 必经点不可走（取墙格 (0,1)）
	_, _, err = g.PathVia(Point{1, 3}, Point{2, 3}, Point{0, 1})
	var bp *BlockedPointError
	if !errors.As(err, &bp) || bp.Point != (Point{0, 1}) || bp.OutOfBounds {
		t.Fatalf("不可走必经点坐标不对: %v", err)
	}
	if !errors.Is(err, ErrNoPath) {
		t.Fatal("BlockedPointError 应兼容 ErrNoPath")
	}

	// 必经点越界
	_, _, err = g.PathVia(Point{1, 3}, Point{9, 0})
	if !errors.As(err, &bp) || !bp.OutOfBounds || bp.Point != (Point{9, 0}) {
		t.Fatalf("越界坐标不对: %v", err)
	}
	if !errors.Is(err, ErrOOB) {
		t.Fatal("越界错误应兼容 ErrOOB")
	}

	// 原始接口行为保持：同点返回单点路径；不可走返回 ErrNoPath
	path, _, err := g.Path(Point{2, 3}, Point{2, 3})
	if err != nil || len(path) != 1 || path[0] != (Point{2, 3}) {
		t.Fatalf("原始同点语义被破坏: %v %v", path, err)
	}
	if _, _, err := g.Path(Point{2, 1}, Point{2, 3}); !errors.Is(err, ErrNoPath) {
		t.Fatalf("原始不可走语义被破坏: %v", err)
	}
}

// 必经点正常拼接：每段 A* 结果在必经点处衔接。
func TestPathViaHappy(t *testing.T) {
	g, _ := Parse([]string{
		".........",
		".........",
		".........",
		".........",
		".........",
	})
	vias := []Point{{0, 0}, {4, 2}, {8, 0}, {8, 4}}
	full, _, err := g.PathVia(vias...)
	if err != nil {
		t.Fatal(err)
	}
	if full[0] != vias[0] || full[len(full)-1] != vias[len(vias)-1] {
		t.Fatal("拼接端点错误")
	}
	seen := map[Point]bool{}
	for _, p := range full {
		seen[p] = true
	}
	for _, v := range vias {
		if !seen[v] {
			t.Fatalf("必经点 %v 未出现在拼接路径中", v)
		}
	}
	// 无重复点（段间共用点只保留一次）
	for i := 1; i < len(full); i++ {
		if full[i] == full[i-1] {
			t.Fatal("拼接路径出现相邻重复点")
		}
	}
	// 拼接路径整体可平滑且不变量成立
	res, err := g.Smooth(full)
	if err != nil {
		t.Fatal(err)
	}
	checkInvariants(t, g, res, DefaultKappaMax)
}
