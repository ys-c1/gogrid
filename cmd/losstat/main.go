// 命令 losstat 输出视线投射的收尾统计：射线数、一致率、回退次数与距离场规模。
//
// 用法：
//
//	go run ./cmd/losstat           # 全部样例 + 两档网格汇总
//	go run ./cmd/losstat -scenario 单墙 -x 0 -y 2 -angle 90 -center 0 -radius 5
//	go run ./cmd/losstat -map '.....,..#..,.....' -x 0 -y 0 -angle 90 -center 0 -radius 4
//
// 地图行用逗号分隔，# 为墙，其余为可走格。
// 每张地图都同时跑子集核验（逐格射线全等）与跨墙核验（射线停墙），
// 两项分开统计；每份投射的射线数都由张角反解并逐次核对上限。
package main

import (
	"flag"
	"fmt"
	"os"
	"sort"
	"strings"
	"time"

	"gogrid/grid"
)

type scenario struct {
	name    string
	kind    string
	rows    []string
	origin  grid.Point
	radius  int
	centers []float64
	angles  []float64
}

func scenarios() []scenario {
	return []scenario{
		{
			name: "单墙", kind: "单墙",
			rows: []string{
				"..........",
				"..........",
				"....###...",
				"..........",
				"..........",
			},
			origin: grid.Point{X: 0, Y: 2}, radius: 6,
			centers: []float64{0, 90}, angles: []float64{90, 180, 360},
		},
		{
			name: "走廊", kind: "走廊",
			rows: []string{
				"#######",
				"#.....#",
				"#.....#",
				"#.....#",
				"#######",
			},
			origin: grid.Point{X: 3, Y: 2}, radius: 4,
			centers: []float64{0, 90}, angles: []float64{90, 180, 360},
		},
		{
			name: "柱状", kind: "柱状",
			rows: []string{
				".........",
				".........",
				"....#....",
				".........",
				".........",
			},
			origin: grid.Point{X: 4, Y: 0}, radius: 6,
			centers: []float64{90, 0}, angles: []float64{90, 180, 360},
		},
		{
			name: "角落", kind: "角落",
			rows: []string{
				"......",
				"......",
				"..#...",
				".##...",
				"......",
			},
			origin: grid.Point{X: 1, Y: 2}, radius: 5,
			centers: []float64{0, 45, 90}, angles: []float64{90, 180, 360},
		},
		{
			name: "全遮挡", kind: "全遮挡",
			rows: []string{
				".....",
				"#####",
				".....",
			},
			origin: grid.Point{X: 2, Y: 2}, radius: 5,
			centers: []float64{270}, angles: []float64{90, 180, 360},
		},
	}
}

// mazeRows 是收尾单独说明用的迷宫样例：中央走廊 + 右侧竖井。
func mazeRows() []string {
	return []string{
		"#########",
		"#.......#",
		"#.#####.#",
		"#.#...#.#",
		"#.#.#.#.#",
		"#...#...#",
		"#########",
	}
}

// openRows 生成 n×n 地图：四周一圈墙，内部按 6 格间距散布单格柱子。
// 两档网格用它对比回退次数与距离场规模；柱子越密，掠射墙角的配置越多。
func openRows(n int) []string {
	rows := make([]string, n)
	for y := 0; y < n; y++ {
		line := make([]byte, n)
		for x := 0; x < n; x++ {
			line[x] = '.'
		}
		rows[y] = string(line)
	}
	mid := n / 2
	rows[0] = strings.Repeat("#", n)
	rows[n-1] = strings.Repeat("#", n)
	for y := 1; y < n-1; y++ {
		rows[y] = "#" + rows[y][1:n-1] + "#"
	}
	// 每 6 格放一根单格柱子，位置随尺寸一致，两档网格形状相似。
	for y := 4; y < n-1; y += 6 {
		for x := 4; x < n-1; x += 6 {
			if y == mid && x == mid {
				continue
			}
			b := []byte(rows[y])
			b[x] = '#'
			rows[y] = string(b)
		}
	}
	return rows
}

type totals struct {
	configs   int
	rays      int
	budget    int
	rawCells  int
	cells     int
	matched   int
	fallbacks int
	occluded  int
	rangeSum  int
	rangeMax  int
	elapsed   time.Duration
}

func (t *totals) add(st grid.CastStats, d time.Duration) {
	t.configs++
	t.rays += st.Rays
	t.budget += st.Budget
	t.rawCells += st.RawCells
	t.cells += st.Cells
	t.matched += st.Matched
	t.fallbacks += st.Fallback
	t.occluded += st.Occluded
	t.rangeSum += st.Range
	if st.Range > t.rangeMax {
		t.rangeMax = st.Range
	}
	t.elapsed += d
}

// merge 把另一份汇总并入本汇总（用于合计行）。
func (t *totals) merge(o totals) {
	t.configs += o.configs
	t.rays += o.rays
	t.budget += o.budget
	t.rawCells += o.rawCells
	t.cells += o.cells
	t.matched += o.matched
	t.fallbacks += o.fallbacks
	t.occluded += o.occluded
	t.rangeSum += o.rangeSum
	if o.rangeMax > t.rangeMax {
		t.rangeMax = o.rangeMax
	}
	t.elapsed += o.elapsed
}

// agreement 返回按格汇总的一致率：匹配格 / 原始扫掠与最终可见的并集。
func (t *totals) agreement() float64 {
	union := t.rawCells + t.cells - t.matched
	if union == 0 {
		return 1
	}
	return float64(t.matched) / float64(union)
}

func (t *totals) meanRange() float64 {
	if t.configs == 0 {
		return 0
	}
	return float64(t.rangeSum) / float64(t.configs)
}

func parseMap(s string) []string { return strings.Split(s, ",") }

func runProjection(g *grid.Grid, cfg grid.CastConfig) (grid.CastStats, time.Duration, error) {
	start := time.Now()
	p, err := g.Project(cfg)
	elapsed := time.Since(start)
	if err != nil {
		return grid.CastStats{}, 0, err
	}
	if err := p.CheckConsistency(); err != nil {
		return grid.CastStats{}, 0, err
	}
	if err := p.CheckNoWallCrossing(); err != nil {
		return grid.CastStats{}, 0, err
	}
	if err := p.CheckBudget(); err != nil {
		return grid.CastStats{}, 0, err
	}
	return p.Stats(), elapsed, nil
}

// runScenario 跑一张地图的全部配置，打印每图一行。
// pad 按终端显示宽度补空格：CJK 字符按 2 格算。
func pad(s string, width int) string {
	w := 0
	for _, r := range s {
		if r >= 0x1100 {
			w += 2
		} else {
			w++
		}
	}
	for ; w < width; w++ {
		s += " "
	}
	return s
}

func runScenario(sc scenario) (totals, error) {
	g, err := grid.Parse(sc.rows)
	if err != nil {
		return totals{}, fmt.Errorf("%s: %w", sc.name, err)
	}
	var t totals
	for _, center := range sc.centers {
		for _, angle := range sc.angles {
			cfg := grid.CastConfig{Origin: sc.origin, CenterDeg: center, AngleDeg: angle, Radius: sc.radius}
			if err := g.ValidateCast(cfg); err != nil {
				return totals{}, fmt.Errorf("%s: %w", sc.name, err)
			}
			st, d, err := runProjection(g, cfg)
			if err != nil {
				return totals{}, fmt.Errorf("%s %+v: %w", sc.name, cfg, err)
			}
			t.add(st, d)
		}
	}
	fmt.Printf("%s %s %-9s %s %4d %5d %5d %5d %5d %5d %5d %7.2f%% %10.1f\n",
		pad(sc.name, 8), pad(sc.kind, 8),
		fmt.Sprintf("%d×%d", g.W, g.H), pad(fmt.Sprintf("(%d,%d)", sc.origin.X, sc.origin.Y), 9),
		t.configs, t.rays, t.budget, t.cells, t.rawCells, t.fallbacks, t.occluded,
		t.agreement()*100, t.meanRange())
	return t, nil
}

func scenarioHeader() string {
	return fmt.Sprintf("%s %s %-9s %s %4s %5s %5s %5s %5s %5s %5s %8s %10s\n",
		pad("场景", 8), pad("情形", 8), "网格", pad("观察点", 9),
		"配置", "射线", "上限", "可见", "原始", "回退", "遮挡", "一致率", "距离场均值")
}

func printScenarioTable() (totals, error) {
	fmt.Println("=== 样例（逐图一行）===")
	fmt.Print(scenarioHeader())
	var all totals
	for _, sc := range scenarios() {
		t, err := runScenario(sc)
		if err != nil {
			return totals{}, err
		}
		all.merge(t)
	}
	fmt.Printf("%s %s %-9s %s %4d %5d %5d %5d %5d %5d %5d %7.2f%% %10.1f\n",
		pad("合计", 8), pad("", 8), "", pad("", 9),
		all.configs, all.rays, all.budget, all.cells, all.rawCells, all.fallbacks,
		all.occluded, all.agreement()*100, all.meanRange())
	return all, nil
}

// printSweepOfOrigin 对一个观察点跑全部张角，用于迷宫的单独说明。
func printSweepOfOrigin(g *grid.Grid, origin grid.Point, radius int, withMask bool) error {
	fmt.Printf("观察点 (%d,%d)，半径 %d\n", origin.X, origin.Y, radius)
	fmt.Printf("%8s %6s %6s %6s %6s %8s %10s\n",
		"张角", "射线", "上限", "原始", "最终", "回退", "一致率")
	for _, angle := range []float64{45, 90, 180, 360} {
		cfg := grid.CastConfig{Origin: origin, CenterDeg: 0, AngleDeg: angle, Radius: radius}
		st, _, err := runProjection(g, cfg)
		if err != nil {
			return err
		}
		fmt.Printf("%8.0f %6d %6d %6d %6d %8d %9.2f%%\n",
			angle, st.Rays, st.Budget, st.RawCells, st.Cells, st.Fallback, st.RawAgreement*100)
		if withMask && angle == 360 {
			p, err := g.Project(cfg)
			if err != nil {
				return err
			}
			fmt.Println()
			fmt.Print(render2(g, cfg.Origin, p.Visible(), p.Occluded()))
			fmt.Printf("最终可见 %d 格；被遮挡 %d 格；回退格 %v（由逐格射线裁定）\n",
				len(p.Visible()), len(p.Occluded()), p.FallbackCells())
			fmt.Println()
		}
	}
	return nil
}

// render2 画最终可见图，并把被遮挡的可走格标成 '·' 以示区别。
func render2(g *grid.Grid, origin grid.Point, visible []grid.Point, occluded []grid.Occluded) string {
	seen := make(map[grid.Point]bool, len(visible))
	for _, p := range visible {
		seen[p] = true
	}
	occludedSet := make(map[grid.Point]bool, len(occluded))
	for _, oc := range occluded {
		occludedSet[oc.Cell] = true
	}
	var b strings.Builder
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			p := grid.Point{X: x, Y: y}
			ok, _ := g.Walkable(p)
			switch {
			case p == origin:
				b.WriteByte('@')
			case !ok:
				b.WriteByte('#')
			case seen[p]:
				b.WriteByte('.')
			case occludedSet[p]:
				b.WriteByte('o')
			default:
				b.WriteByte(' ')
			}
		}
		b.WriteByte('\n')
	}
	return b.String()
}

func printMaze() error {
	g, err := grid.Parse(mazeRows())
	if err != nil {
		return err
	}
	fmt.Println("=== 迷宫（单独说明）===")
	fmt.Println("地图 9×7：中央走廊 + 右侧竖井。观察点 (1,1)，半径 8。")
	for _, line := range mazeRows() {
		fmt.Println("  " + line)
	}
	fmt.Println()
	return printSweepOfOrigin(g, grid.Point{X: 1, Y: 1}, 8, true)
}

// printGridScales 在两档网格上各列一行：回退次数与距离场规模。
func printGridScales() error {
	fmt.Println()
	fmt.Println("=== 两档网格（各列一行）===")
	fmt.Printf("%-10s %-4s %-4s %-6s %-6s %-5s %-5s %-6s %-8s %s\n",
		"网格", "边长", "格数", "观察点", "配置", "射线", "回退", "距离场总", "距离场均值", "耗时")
	for _, n := range []int{21, 41} {
		rows := openRows(n)
		g, err := grid.Parse(rows)
		if err != nil {
			return err
		}
		origin := grid.Point{X: 1, Y: 1}
		var t totals
		radius := n / 4
		for _, angle := range []float64{45, 90, 180, 360} {
			cfg := grid.CastConfig{Origin: origin, CenterDeg: 45, AngleDeg: angle, Radius: radius}
			if err := g.ValidateCast(cfg); err != nil {
				return err
			}
			st, d, err := runProjection(g, cfg)
			if err != nil {
				return err
			}
			t.add(st, d)
		}
		fmt.Printf("%-10s %-4d %-4d %-6s %-6d %-5d %-5d %-8d %-8.1f %v\n",
			fmt.Sprintf("%d×%d", n, n), n, g.WalkableCount(),
			fmt.Sprintf("(%d,%d)", origin.X, origin.Y),
			t.configs, t.rays, t.fallbacks, t.rangeSum, t.meanRange(), t.elapsed.Round(time.Microsecond))
	}
	return nil
}

func main() {
	mapFlag := flag.String("map", "", "逗号分隔的自定义地图（# 为墙）")
	scenarioFlag := flag.String("scenario", "", "只跑样例集中的某个场景")
	ox := flag.Int("x", 0, "观察点 x")
	oy := flag.Int("y", 0, "观察点 y")
	angle := flag.Float64("angle", 90, "张角（度）")
	center := flag.Float64("center", 0, "中线角度（度）")
	radius := flag.Int("radius", 4, "半径（格）")
	flag.Parse()

	if *mapFlag != "" {
		g, err := grid.Parse(parseMap(*mapFlag))
		if err != nil {
			fmt.Fprintln(os.Stderr, "错误：", err)
			os.Exit(1)
		}
		cfg := grid.CastConfig{Origin: grid.Point{X: *ox, Y: *oy}, CenterDeg: *center, AngleDeg: *angle, Radius: *radius}
		st, d, err := runProjection(g, cfg)
		if err != nil {
			fmt.Fprintln(os.Stderr, "错误：", err)
			os.Exit(1)
		}
		reportOne(g, cfg, st, d)
		return
	}

	if *scenarioFlag != "" {
		for _, sc := range scenarios() {
			if sc.name != *scenarioFlag {
				continue
			}
			g, err := grid.Parse(sc.rows)
			if err != nil {
				fmt.Fprintln(os.Stderr, "错误：", err)
				os.Exit(1)
			}
			cfg := grid.CastConfig{Origin: sc.origin, CenterDeg: *center, AngleDeg: *angle, Radius: sc.radius}
			st, d, err := runProjection(g, cfg)
			if err != nil {
				fmt.Fprintln(os.Stderr, "错误：", err)
				os.Exit(1)
			}
			reportOne(g, cfg, st, d)
			return
		}
		fmt.Fprintf(os.Stderr, "错误：没有名为 %q 的场景\n", *scenarioFlag)
		os.Exit(1)
	}

	if _, err := printScenarioTable(); err != nil {
		fmt.Fprintln(os.Stderr, "错误：", err)
		os.Exit(1)
	}
	fmt.Println()
	fmt.Println("口径：")
	fmt.Println("  射线   = 实际投射的扫掠条数（由张角反解，逐次核验不超过上限）")
	fmt.Println("  一致率 = 原始扫掠与逐格 Bresenham 射线的一致率（交并比）")
	fmt.Println("  回退   = 原始扫掠与逐格射线不一致、由逐格射线裁定的格数")
	fmt.Println("  遮挡   = 扇形几何掩码内、由最终可见集反推出的被遮挡可走格数")
	fmt.Println("  距离场 = 最终可见格数（每个可见格一条距离记录）")
	fmt.Println()
	if err := printMaze(); err != nil {
		fmt.Fprintln(os.Stderr, "错误：", err)
		os.Exit(1)
	}
	if err := printGridScales(); err != nil {
		fmt.Fprintln(os.Stderr, "错误：", err)
		os.Exit(1)
	}
}

func reportOne(g *grid.Grid, cfg grid.CastConfig, st grid.CastStats, d time.Duration) {
	fmt.Printf("地图 %d×%d，观察点 (%d,%d)，中线 %.1f°，张角 %.1f°，半径 %d\n",
		g.W, g.H, cfg.Origin.X, cfg.Origin.Y, cfg.CenterDeg, cfg.AngleDeg, cfg.Radius)
	fmt.Printf("射线 %d 条（反解 %d），可见 %d 格，原始扫掠 %d 格，回退 %d，一致率 %.2f%%，距离场 %d，耗时 %v\n",
		st.Rays, st.Budget, st.Cells, st.RawCells, st.Fallback, st.RawAgreement*100, st.Range, d)
	p, err := g.Project(cfg)
	if err == nil {
		fmt.Print(render(g, cfg.Origin, p.Visible()))
		fmt.Printf("被遮挡 %d 格（扇形内的可走格）\n", len(p.Occluded()))
		occluded := p.Occluded()
		sort.Slice(occluded, func(i, j int) bool {
			if occluded[i].Cell.Y != occluded[j].Cell.Y {
				return occluded[i].Cell.Y < occluded[j].Cell.Y
			}
			return occluded[i].Cell.X < occluded[j].Cell.X
		})
		for i, oc := range occluded {
			if i >= 8 {
				fmt.Printf("  ……另有 %d 格\n", len(occluded)-i)
				break
			}
			fmt.Printf("  (%d,%d) 被墙 (%d,%d) 挡住\n", oc.Cell.X, oc.Cell.Y, oc.Wall.X, oc.Wall.Y)
		}
	}
}

// render 画可见图：@ 观察点，# 墙，. 可见，空格 不可见。
func render(g *grid.Grid, origin grid.Point, visible []grid.Point) string {
	seen := make(map[grid.Point]bool, len(visible))
	for _, p := range visible {
		seen[p] = true
	}
	var b strings.Builder
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			p := grid.Point{X: x, Y: y}
			ok, _ := g.Walkable(p)
			switch {
			case p == origin:
				b.WriteByte('@')
			case !ok:
				b.WriteByte('#')
			case seen[p]:
				b.WriteByte('.')
			default:
				b.WriteByte(' ')
			}
		}
		b.WriteByte('\n')
	}
	return b.String()
}
