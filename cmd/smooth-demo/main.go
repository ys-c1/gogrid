// 命令行演示：五类用例的寻路 + 曲率受限平滑，输出曲率上界与间距对照。
package main

import (
	"errors"
	"fmt"
	"math"
	"math/rand"
	"strings"

	"gogrid/grid"
)

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

func render(g *grid.Grid, original, smoothed []grid.Point) string {
	onPath := map[grid.Point]bool{}
	for _, p := range original {
		onPath[p] = true
	}
	onSmooth := map[grid.Point]bool{}
	for _, p := range smoothed {
		onSmooth[p] = true
	}
	var b strings.Builder
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			p := grid.Point{X: x, Y: y}
			switch {
			case onSmooth[p]:
				b.WriteByte('*')
			case onPath[p]:
				b.WriteByte('+')
			default:
				ok, _ := g.Walkable(p)
				if ok {
					b.WriteByte('.')
				} else {
					b.WriteByte('#')
				}
			}
		}
		b.WriteByte('\n')
	}
	return b.String()
}

func runCase(name string, rows []string, from, to grid.Point) {
	fmt.Printf("================ %s ================\n", name)
	g, err := grid.Parse(rows)
	if err != nil {
		fmt.Println("解析失败:", err)
		return
	}
	path, _, err := g.Path(from, to)
	if err != nil {
		fmt.Println("寻路失败:", err)
		return
	}
	res, err := g.Smooth(path)
	if err != nil {
		fmt.Println("平滑失败:", err)
		return
	}
	fmt.Print(render(g, res.Original, res.Smoothed))
	fmt.Printf("图例: * 平滑后路径点  + 仅原始路径点\n")
	fmt.Printf("点数: %d -> %d    长度: %.3f -> %.3f (Δ=%+.3f)\n",
		len(res.Original), len(res.Smoothed),
		res.LengthBefore, res.LengthAfter, res.LengthAfter-res.LengthBefore)
	if math.IsInf(res.ClearanceBefore, 1) {
		fmt.Printf("最小间距: 无障碍(+Inf)  采样核验点=%d 逐点通过=%v\n",
			res.SamplePoints, res.PointwiseOK)
	} else {
		fmt.Printf("最小间距: %.4f -> %.4f (Δ=%+.4f)  采样核验点=%d 逐点通过=%v\n",
			res.ClearanceBefore, res.ClearanceAfter,
			res.ClearanceAfter-res.ClearanceBefore, res.SamplePoints, res.PointwiseOK)
	}
	fmt.Printf("曲率上界 κmax=%.4f  最大曲率: %.4f -> %.4f\n",
		res.KappaMax, res.MaxKappaBefore, res.MaxKappaAfter)
	fmt.Println("拐角明细 (坐标 | 转角° | 反解k* | 几何k | 实拉k | 平滑后κ | 处置):")
	for _, st := range res.Corners {
		action := "消除"
		if st.Retained {
			action = "保留"
			if st.GeometryBound {
				action = "保留(几何受限)"
			}
		}
		fmt.Printf("  (%d,%d) | %5.1f | %d | %d | %d | %.4f | %s\n",
			st.Point.X, st.Point.Y, st.TurnAngle*180/math.Pi,
			st.KNeed, st.KFeas, st.KUsed, st.KappaAfter, action)
	}
	fmt.Println()
}

func showErrors() {
	fmt.Println("================ 异常坐标上报 ================")
	g, _ := grid.Parse([]string{
		"#####",
		"##.##",
		"#.#.#",
		"#..##",
		"#####",
	})
	probes := []struct {
		label string
		vias  []grid.Point
	}{
		{"无路", []grid.Point{{X: 2, Y: 1}, {X: 2, Y: 3}}},
		{"起终点相同", []grid.Point{{X: 2, Y: 3}, {X: 2, Y: 3}}},
		{"必经点不可走", []grid.Point{{X: 1, Y: 3}, {X: 2, Y: 3}, {X: 0, Y: 1}}},
		{"必经点越界", []grid.Point{{X: 1, Y: 3}, {X: 9, Y: 0}}},
	}
	for _, pr := range probes {
		_, _, err := g.PathVia(pr.vias...)
		sentinel := "-"
		switch {
		case errors.Is(err, grid.ErrNoPath):
			sentinel = "ErrNoPath"
		case errors.Is(err, grid.ErrOOB):
			sentinel = "ErrOOB"
		}
		fmt.Printf("%-12s -> %v (哨兵: %s)\n", pr.label, err, sentinel)
	}
	fmt.Println()
}

func main() {
	runCase("用例1 直廊", []string{
		"#########",
		"#.......#",
		"#########",
	}, grid.Point{X: 1, Y: 1}, grid.Point{X: 7, Y: 1})

	runCase("用例2 拐角(L 形单宽走廊)", []string{
		"#####",
		"#...#",
		"###.#",
		"#...#",
		"#####",
	}, grid.Point{X: 1, Y: 1}, grid.Point{X: 1, Y: 3})

	open := make([]string, 8)
	for i := range open {
		open[i] = strings.Repeat(".", 8)
	}
	runCase("用例3 开阔地", open, grid.Point{X: 0, Y: 0}, grid.Point{X: 7, Y: 7})

	runCase("用例4 窄门", []string{
		"#########",
		"#.......#",
		"#.......#",
		"####.####",
		"#.......#",
		"#.......#",
		"#########",
	}, grid.Point{X: 1, Y: 1}, grid.Point{X: 7, Y: 5})

	runCase("用例5 迷宫(21x15)", genMaze(21, 15, 20261124),
		grid.Point{X: 1, Y: 1}, grid.Point{X: 19, Y: 13})

	showErrors()
}
