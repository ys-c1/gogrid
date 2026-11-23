// 命令 fovstat 输出扇形视野统计，供人工核对与逐格视线基准的耗时对照。
//
// 用法：
//
//	go run ./cmd/fovstat                       # 跑地图集并打印汇总表
//	go run ./cmd/fovstat -map '.....,..#..,.....' -x 0 -y 0 -facing E -angle 90 -radius 6
//
// 地图用逗号分隔行，# 为墙，其余为可走格。
package main

import (
	"flag"
	"fmt"
	"os"
	"sort"
	"strconv"
	"strings"
	"time"

	"gogrid/grid"
)

type scenario struct {
	name   string
	kind   string // "贴墙" / "角落" / "遮挡"
	rows   []string
	origin grid.Point
	facing grid.Facing
	radius int
	tag    string
}

var scenarios = []scenario{
	{
		name: "wall-hug", kind: "贴墙",
		rows: []string{
			"..........",
			"..........",
			".....#####",
			"....@.....",
			"..........",
			"..........",
		},
		origin: grid.Point{X: 4, Y: 3}, facing: grid.North, radius: 6,
		tag: "观察点紧贴一段墙的西南端，墙只裁掉扇形的东北半边",
	},
	{
		name: "wall-corridor", kind: "贴墙",
		rows: []string{
			"#######",
			"#.....#",
			"#..@..#",
			"#.....#",
			"#######",
		},
		origin: grid.Point{X: 3, Y: 2}, facing: grid.East, radius: 4,
		tag: "1 格宽走廊，观察点贴南墙，正前方两格后被东墙截断",
	},
	{
		name: "corner-notch", kind: "角落",
		rows: []string{
			"..........",
			"..........",
			"..........",
			"....######",
			"....#.....",
			"....#.....",
			"....#.....",
		},
		origin: grid.Point{X: 5, Y: 4}, facing: grid.South, radius: 5,
		tag: "观察点位于北墙与西墙形成的凹角，背靠两面墙向南看",
	},
	{
		name: "corner-map", kind: "角落",
		rows: []string{
			"..........",
			"..........",
			"..........",
			"..........",
			"..........",
		},
		origin: grid.Point{X: 0, Y: 0}, facing: grid.East, radius: 6,
		tag: "观察点位于地图左上角，扇形的西北半边出界",
	},
	{
		name: "occluded", kind: "遮挡",
		rows: []string{
			".....",
			"#####",
			"..@..",
			".....",
			".....",
		},
		origin: grid.Point{X: 2, Y: 2}, facing: grid.North, radius: 5,
		tag: "正前方 1 格即整排墙，扇形内除观察点外无可见格",
	},
}

func parseFacing(s string) grid.Facing {
	switch strings.ToUpper(strings.TrimSpace(s)) {
	case "N":
		return grid.North
	case "E":
		return grid.East
	case "S":
		return grid.South
	case "W":
		return grid.West
	}
	return grid.Facing(-1)
}

// render 画地图：@ 观察点，# 墙，. 可见，空格 不可见。
func render(rows []string, origin grid.Point, visible map[grid.Point]bool) string {
	var b strings.Builder
	for y, row := range rows {
		for x := 0; x < len(row); x++ {
			p := grid.Point{X: x, Y: y}
			switch {
			case p == origin:
				b.WriteByte('@')
			case row[x] == '#':
				b.WriteByte('#')
			case visible[p]:
				b.WriteByte('.')
			default:
				b.WriteByte(' ')
			}
		}
		b.WriteByte('\n')
	}
	return b.String()
}

func setOf(pts []grid.Point) map[grid.Point]bool {
	m := make(map[grid.Point]bool, len(pts))
	for _, p := range pts {
		m[p] = true
	}
	return m
}

func cellList(pts []grid.Point) string {
	if len(pts) == 0 {
		return "（空）"
	}
	parts := make([]string, len(pts))
	for i, p := range pts {
		parts[i] = fmt.Sprintf("(%d,%d)", p.X, p.Y)
	}
	return strings.Join(parts, " ")
}

func bucketLabel(angle float64) string {
	switch angle {
	case 90:
		return "窄角90°"
	case 180:
		return "中角180°"
	case 360:
		return "宽角360°"
	}
	return fmt.Sprintf("%g°", angle)
}

// timeit 运行 f 共 51 次，返回单次耗时的中位数，避免单次抖动误导。
func timeit(f func()) time.Duration {
	const n = 51
	samples := make([]time.Duration, n)
	for i := 0; i < n; i++ {
		start := time.Now()
		f()
		samples[i] = time.Since(start)
	}
	sort.Slice(samples, func(i, j int) bool { return samples[i] < samples[j] })
	return samples[n/2]
}

// runOne 打印单个场景的可见图、分档统计、差集与两档半径耗时。
func runOne(sc scenario, angles []float64) error {
	g, err := grid.Parse(sc.rows)
	if err != nil {
		return fmt.Errorf("%s: %w", sc.name, err)
	}
	// 先校验全部参数，避免打印半截结果后再报错。
	for _, angle := range angles {
		cfg := grid.FOVConfig{Origin: sc.origin, Facing: sc.facing, AngleDeg: angle, Radius: sc.radius}
		if err := g.ValidateFOV(cfg); err != nil {
			return err
		}
	}
	fmt.Printf("场景 %s（%s）—— %s\n", sc.name, sc.kind, sc.tag)
	fmt.Printf("地图 %dx%d，观察点 (%d,%d)，朝向 %v，基准半径 %d\n\n",
		g.W, g.H, sc.origin.X, sc.origin.Y, sc.facing, sc.radius)
	fmt.Print(render(sc.rows, sc.origin, nil))
	fmt.Println()

	full, err := g.VisibleSet(sc.origin, sc.radius)
	if err != nil {
		return err
	}
	fmt.Printf("全开（逐格视线基准，半径 %d）：%d 格\n", sc.radius, len(full))
	for _, angle := range angles {
		cfg := grid.FOVConfig{Origin: sc.origin, Facing: sc.facing, AngleDeg: angle, Radius: sc.radius}
		vis, err := g.FOV(cfg)
		if err != nil {
			return err
		}
		sector, err := g.SectorCells(cfg)
		if err != nil {
			return err
		}
		// 上界：FOV ⊆ 全开。实测差集必为空，这里如实打印以自证。
		violation := grid.DiffSets(full, vis).OnlyB
		// 遮挡：扇形几何内、但被逐格视线挡掉的格，即 FOV 相对扇形掩码的损失。
		occluded := grid.DiffSets(sector, vis).OnlyA
		fmt.Printf("  %-9s 可见 %2d 格；扇形掩码 %2d 格；被遮挡 %2d 格 %s；FOV ⊄ 全开 的格数 %d\n",
			bucketLabel(angle), len(vis), len(sector), len(occluded), cellList(occluded), len(violation))
	}
	fmt.Println()
	fmt.Print(render(sc.rows, sc.origin, setOf(mustFOV(g, grid.FOVConfig{
		Origin: sc.origin, Facing: sc.facing, AngleDeg: 90, Radius: sc.radius,
	}))))

	// 两档半径：基准 R 与 max(1, R/2)，均用 90° 窄角，各报可见格数与耗时。
	rNear := sc.radius / 2
	if rNear < 1 {
		rNear = 1
	}
	if rNear == sc.radius {
		rNear = sc.radius // 半径已为 1，两档必然相同，如实标注
	}
	for _, r := range []int{sc.radius, rNear} {
		cfg := grid.FOVConfig{Origin: sc.origin, Facing: sc.facing, AngleDeg: 90, Radius: r}
		vis, err := g.FOV(cfg)
		if err != nil {
			return err
		}
		tBase := timeit(func() { _, _ = g.VisibleSet(sc.origin, r) })
		tFOV := timeit(func() { _, _ = g.FOV(cfg) })
		fmt.Printf("  半径 %d（90°）：可见 %2d 格；逐格基准 %-10v；扇形 FOV %v\n",
			r, len(vis), tBase, tFOV)
	}
	fmt.Println()
	return nil
}

func mustFOV(g *grid.Grid, cfg grid.FOVConfig) []grid.Point {
	vis, err := g.FOV(cfg)
	if err != nil {
		panic(err)
	}
	return vis
}

func printTable() error {
	const radius = 5
	fmt.Printf("=== 汇总表（基准半径 %d，按张角分档）===\n", radius)
	fmt.Printf("%-14s %-4s %-4s %6s %6s %6s %6s %7s %7s %7s\n",
		"场景", "情形", "朝向", "全开", "窄角90", "中角180", "宽角360", "遮挡90", "遮挡180", "遮挡360")
	for _, sc := range scenarios {
		g, err := grid.Parse(sc.rows)
		if err != nil {
			return err
		}
		r := radius
		if r > g.MaxRadius() {
			r = g.MaxRadius()
		}
		full, err := g.VisibleSet(sc.origin, r)
		if err != nil {
			return err
		}
		var counts [3]int
		var losses [3]int
		for i, angle := range []float64{90, 180, 360} {
			cfg := grid.FOVConfig{Origin: sc.origin, Facing: sc.facing, AngleDeg: angle, Radius: r}
			vis, err := g.FOV(cfg)
			if err != nil {
				return err
			}
			sector, err := g.SectorCells(cfg)
			if err != nil {
				return err
			}
			counts[i] = len(vis)
			// 扇形几何内被遮挡而损失的格数（相对无遮挡上界）。
			losses[i] = len(grid.DiffSets(sector, vis).OnlyA)
		}
		fmt.Printf("%-14s %-4s %-4v %6d %6d %6d %6d %6d %6d %6d\n",
			sc.name, sc.kind, sc.facing, len(full), counts[0], counts[1], counts[2],
			losses[0], losses[1], losses[2])
	}
	fmt.Println("\n口径：")
	fmt.Println("  全开        = 半径内、逐格 Bresenham 视线可见的可走格（VisibleSet）")
	fmt.Println("  窄/中/宽角  = FOV 可见格数；宽角 360 必须等于全开（全开模式行为不变）")
	fmt.Println("  遮挡损失    = 扇形掩码(SectorCells) \\ FOV，即落在张角内、但被墙挡掉的格数")
	fmt.Println("  可见集差上界 = FOV 相对全开集的差集，恒为空；测试与上表逐项核对")
	return nil
}

func main() {
	rows := flag.String("map", "", "逗号分隔的自定义地图（# 为墙）；留空则跑地图集")
	anglesFlag := flag.String("angles", "90,180,360", "要对比的张角列表（度）")
	radius := flag.Int("radius", 6, "自定义地图的半径")
	facingStr := flag.String("facing", "N", "自定义地图的朝向：N/E/S/W")
	ox := flag.Int("x", 0, "自定义地图的观察点 x")
	oy := flag.Int("y", 0, "自定义地图的观察点 y")
	only := flag.String("scenario", "", "只跑地图集里的某个场景（名字见下表）；留空跑全部")
	table := flag.Bool("table", true, "跑地图集时打印汇总表")
	flag.Parse()

	angles, err := parseAngles(*anglesFlag)
	if err != nil {
		fmt.Fprintln(os.Stderr, "错误：-angles ", err)
		os.Exit(1)
	}

	if *rows != "" {
		sc := scenario{
			name:   "custom",
			kind:   "自定义",
			rows:   strings.Split(*rows, ","),
			origin: grid.Point{X: *ox, Y: *oy},
			facing: parseFacing(*facingStr),
			radius: *radius,
			tag:    "自定义地图",
		}
		if err := runOne(sc, angles); err != nil {
			fmt.Fprintln(os.Stderr, "错误：", err)
			os.Exit(1)
		}
		return
	}

	ran := 0
	for _, sc := range scenarios {
		if *only != "" && sc.name != *only {
			continue
		}
		if err := runOne(sc, angles); err != nil {
			fmt.Fprintln(os.Stderr, "错误：", err)
			os.Exit(1)
		}
		ran++
	}
	if ran == 0 {
		fmt.Fprintf(os.Stderr, "错误：没有名为 %q 的场景\n", *only)
		os.Exit(1)
	}
	if *table {
		if err := printTable(); err != nil {
			fmt.Fprintln(os.Stderr, "错误：", err)
			os.Exit(1)
		}
	}
}

func parseAngles(s string) ([]float64, error) {
	var out []float64
	for _, part := range strings.Split(s, ",") {
		part = strings.TrimSpace(part)
		if part == "" {
			continue
		}
		a, err := strconv.ParseFloat(part, 64)
		if err != nil {
			return nil, fmt.Errorf("无法解析 %q：%w", part, err)
		}
		out = append(out, a)
	}
	if len(out) == 0 {
		return nil, fmt.Errorf("张角列表为空")
	}
	return out, nil
}
