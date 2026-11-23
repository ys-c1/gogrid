package main

import (
	"fmt"
	"math"
	"time"

	"gogrid/grid"
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

type shapeResult struct {
	name    string
	regions []grid.Region
	first   time.Duration
	average time.Duration
	repeats int
	err     error
}

const benchRepeats = 2000

func runTier(tierName string, maps map[string][]string) []shapeResult {
	order := []string{"single", "multi", "ring", "thin", "wall"}
	results := make([]shapeResult, 0, len(order))
	for _, name := range order {
		g, perr := grid.Parse(maps[name])
		if perr != nil {
			results = append(results, shapeResult{name: name, err: perr})
			continue
		}
		start := time.Now()
		regions, lerr := g.LabelComponents()
		first := time.Since(start)
		if lerr != nil {
			results = append(results, shapeResult{name: name, err: lerr, first: first})
			continue
		}
		// 多次重复取平均，抑制调度抖动。
		start = time.Now()
		for i := 0; i < benchRepeats; i++ {
			if _, err := g.LabelComponents(); err != nil {
				panic(err)
			}
		}
		avg := time.Since(start) / benchRepeats
		results = append(results, shapeResult{
			name: name, regions: regions, first: first,
			average: avg, repeats: benchRepeats,
		})
	}
	return results
}

func fmtMean(m [2]float64) string {
	return fmt.Sprintf("(%.3f, %.3f)", m[0], m[1])
}

func main() {
	tiers := []struct {
		name string
		maps map[string][]string
	}{
		{"粗档网格(5格量级)", coarseMaps()},
		{"细档网格(9格量级)", fineMaps()},
	}

	for _, tier := range tiers {
		fmt.Printf("================ %s ================\n", tier.name)
		results := runTier(tier.name, tier.maps)
		for _, r := range results {
			if r.err != nil {
				fmt.Printf("[%-6s] 错误: %v | 单次耗时 %v\n", r.name, r.err, r.first)
				continue
			}
			fmt.Printf("[%-6s] 区域数=%d  单次耗时=%v  %d次平均=%v\n",
				r.name, len(r.regions), r.first, r.repeats, r.average)
			for _, reg := range r.regions {
				ok := reg.WithinTolerance()
				fmt.Printf("    区域%d 面积N=%2d  精确质心=%s  标注格=(%d,%d)  偏差d=%.4f  ε(N)=√N=%.4f  校验 d≤ε: %v\n",
					reg.ID, reg.Area, fmtMean(reg.Mean),
					reg.Centroid.X, reg.Centroid.Y,
					reg.Deviation(), grid.CentroidTolerance(reg.Area), ok)
			}
		}
		fmt.Println()
	}

	// 错误路径演示：空网格 / 全墙 / 越界 / 墙体坐标，错误均携带坐标。
	fmt.Println("================ 错误路径（均报出坐标/编号） ================")
	if _, err := grid.Parse(nil); err != nil {
		fmt.Printf("空网格:   %v\n", err)
	}
	wallG, _ := grid.Parse(coarseMaps()["wall"])
	if _, err := wallG.LabelComponents(); err != nil {
		fmt.Printf("全墙:     %v（区域为空）\n", err)
	}
	ringG, _ := grid.Parse(coarseMaps()["ring"])
	if _, err := ringG.CentroidAt(grid.Point{X: 9, Y: 0}); err != nil {
		fmt.Printf("越界坐标: %v\n", err)
	}
	if _, err := ringG.CentroidAt(grid.Point{X: 2, Y: 2}); err != nil {
		fmt.Printf("墙体坐标: %v\n", err)
	}
	if _, err := ringG.CentroidOf(7); err != nil {
		fmt.Printf("编号越界: %v\n", err)
	}
	fmt.Println()

	// 收尾：质心与精度表（每档一行汇总偏差）。
	fmt.Println("================ 质心与精度表（质心偏差按两档各列一行） ================")
	fmt.Println("网格档位        用例     区数  各区域(面积N:标注质心:偏差d/精度ε)                              档内最大d   档内最大ε   结论")
	for _, tier := range tiers {
		results := runTier(tier.name, tier.maps)
		maxD, maxE := 0.0, 0.0
		nRegions := 0
		detail := ""
		allPass := true
		for _, r := range results {
			if r.err != nil {
				detail += fmt.Sprintf("%s=无区;", r.name)
				continue
			}
			nRegions += len(r.regions)
			for _, reg := range r.regions {
				detail += fmt.Sprintf("%s-R%d(N%d:(%d,%d):%.2f/%.2f);",
					r.name, reg.ID, reg.Area,
					reg.Centroid.X, reg.Centroid.Y,
					reg.Deviation(), grid.CentroidTolerance(reg.Area))
				maxD = math.Max(maxD, reg.Deviation())
				maxE = math.Max(maxE, grid.CentroidTolerance(reg.Area))
				if !reg.WithinTolerance() {
					allPass = false
				}
			}
		}
		verdict := "通过"
		if !allPass {
			verdict = "超限"
		}
		fmt.Printf("%-14s  %-6s  %2d    %-62s  %8.4f   %8.4f   %s\n",
			tier.name, "五用例", nRegions, detail, maxD, maxE, verdict)
	}
}
