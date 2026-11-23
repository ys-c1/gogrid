// 代价场增量更新与全量重算对照演示：
//
//	go run ./cmd/costdemo
//
// 覆盖泥地、草地、混合、无路、起终点相同，并在每次权重变更后
// 用一份独立克隆做全量重算，逐格核对增量结果，打印重算格数、耗时与边界格。
package main

import (
	"errors"
	"fmt"
	"time"

	"gogrid/grid"
)

func main() {
	terrain := map[byte]int{'.': 1, 'g': 2, 'm': 5}

	fmt.Println("================ 场景 1：泥地绕行 ================")
	rows := []string{
		".....",
		".....",
		".mmm.",
		".....",
		".....",
	}
	g := build(rows, terrain)
	from, to := grid.Point{X: 0, Y: 2}, grid.Point{X: 4, Y: 2}
	f := mustField(g, from)
	printPath(f, to, "初始代价场（应绕开泥列）")

	fmt.Println("\n================ 场景 2：草地权衡 ================")
	g2 := build([]string{"...", ".g.", "..."}, terrain)
	f2 := mustField(g2, grid.Point{X: 0, Y: 1})
	printPath(f2, grid.Point{X: 2, Y: 1}, "草权重 2：短穿更划算")
	compare(f2, g2, grid.Point{X: 0, Y: 1}, grid.Point{X: 1, Y: 1}, 5)
	printPath(f2, grid.Point{X: 2, Y: 1}, "草权重升到 5：改为绕行")

	fmt.Println("\n================ 场景 3：混合地形 ================")
	g3 := build([]string{".g.", "m#m", "..."}, terrain)
	f3 := mustField(g3, grid.Point{X: 0, Y: 0})
	for _, p := range []grid.Point{
		{X: 1, Y: 0}, {X: 2, Y: 0}, {X: 0, Y: 1}, {X: 2, Y: 1}, {X: 2, Y: 2},
	} {
		c, _ := f3.CostAt(p)
		fmt.Printf("  格 %s 代价=%d\n", p, c)
	}
	compare(f3, g3, grid.Point{X: 0, Y: 0}, grid.Point{X: 0, Y: 1}, 2) // 泥地降权
	compare(f3, g3, grid.Point{X: 0, Y: 0}, grid.Point{X: 1, Y: 0}, 6) // 草地升权

	fmt.Println("\n================ 场景 4：无路（不可达） ================")
	g4 := build([]string{".#.", ".#.", ".#."}, terrain)
	f4 := mustField(g4, grid.Point{X: 0, Y: 0})
	c, _ := f4.CostAt(grid.Point{X: 2, Y: 0})
	fmt.Printf("  被墙隔开的 (2,0) 代价=%d（-1 表示不可达）\n", c)
	if _, _, err := f4.PathTo(grid.Point{X: 2, Y: 0}); err != nil {
		fmt.Printf("  PathTo 报错（带坐标）: %v\n", err)
	}

	fmt.Println("\n================ 场景 5：起终点相同 ================")
	g5 := build([]string{"...", "..."}, terrain)
	f5 := mustField(g5, grid.Point{X: 1, Y: 1})
	printPath(f5, grid.Point{X: 1, Y: 1}, "源点即终点")

	fmt.Println("\n================ 场景 6：错误输入（带坐标） ================")
	if err := g5.SetWeight(grid.Point{X: 0, Y: 0}, -1); err != nil {
		fmt.Printf("  负权重:   %v\n", err)
	}
	if _, err := grid.Parse(nil); err != nil {
		fmt.Printf("  空网格:   %v\n", err)
	}
	if _, _, err := f5.PathTo(grid.Point{X: 9, Y: 9}); err != nil {
		fmt.Printf("  终点越界: %v\n", err)
	}
	g6 := build([]string{".#."}, terrain)
	f6 := mustField(g6, grid.Point{X: 0, Y: 0})
	_, _, errBlocked := f6.PathTo(grid.Point{X: 1, Y: 0})
	if errBlocked != nil {
		fmt.Printf("  终点不可走: %v\n", errBlocked)
		fmt.Printf("  错误分类 errors.Is(ErrBlocked)=%v\n", errors.Is(errBlocked, grid.ErrBlocked))
	}

	fmt.Println("\n================ 场景 7：越界回退全量重算 ================")
	stats, err := f5.UpdateOrRecompute(grid.Point{X: 99, Y: 99}, 3)
	fmt.Printf("  越界返回: %v\n", err)
	fmt.Printf("  FellBack=%v 全量重算格数=%d（全图可走=%d）\n",
		stats.FellBack, stats.Recomputed, g5.WalkableCount())

	fmt.Println("\n================ 两档网格：代价与耗时对照 ================")
	for _, size := range []int{20, 100} {
		bench(size)
	}
}

func build(rows []string, weights map[byte]int) *grid.Grid {
	g, err := grid.Parse(rows)
	if err != nil {
		panic(err)
	}
	for y, row := range rows {
		for x, ch := range []byte(row) {
			if w, ok := weights[ch]; ok {
				if err := g.SetWeight(grid.Point{X: x, Y: y}, w); err != nil {
					panic(err)
				}
			}
		}
	}
	return g
}

func mustField(g *grid.Grid, src grid.Point) *grid.CostField {
	f, err := grid.NewCostField(g, src)
	if err != nil {
		panic(err)
	}
	return f
}

func printPath(f *grid.CostField, to grid.Point, title string) {
	path, cost, err := f.PathTo(to)
	if err != nil {
		fmt.Printf("  %s -> 无路: %v\n", title, err)
		return
	}
	fmt.Printf("  %s\n", title)
	fmt.Printf("    路径(%d 格): %v  总代价=%d\n", len(path), path, cost)
}

// compare 在增量场上更新权重，同时用一份独立克隆做全量重算，逐格核对并打印对照。
func compare(inc *grid.CostField, g *grid.Grid, src, p grid.Point, newW int) {
	fmt.Printf("\n  -- 变更 %s 权重 -> %d --\n", p, newW)

	// 参考场：克隆当前网格（含当前权重），先全量建场，再施加同一变更后全量重算。
	refGrid := g.Clone()
	ref := mustField(refGrid, src)

	t0 := time.Now()
	stats, err := inc.UpdateWeight(p, newW)
	incDur := time.Since(t0)
	if err != nil {
		fmt.Printf("    增量更新失败: %v\n", err)
		return
	}

	t1 := time.Now()
	if err := refGrid.SetWeight(p, newW); err != nil {
		fmt.Printf("    参考场设权重失败: %v\n", err)
		return
	}
	fullN := ref.FullRecompute()
	fullDur := time.Since(t1)

	mark := "✗ 不一致"
	if inc.Equals(ref) {
		mark = "✓ 逐格相同"
	}
	fmt.Printf("    增量 vs 全量: %s\n", mark)
	fmt.Printf("    增量: 重算 %3d 格, 耗时 %v\n", stats.Recomputed, incDur)
	fmt.Printf("    全量: 重算 %3d 格, 耗时 %v\n", fullN, fullDur)
	fmt.Printf("    边界格(%d): %v\n", len(stats.Boundary), stats.Boundary)
}

func bench(size int) {
	rows := make([]string, size)
	for y := 0; y < size; y++ {
		b := make([]byte, size)
		for x := 0; x < size; x++ {
			if y == size/2 && x != size-1 {
				b[x] = '#'
			} else {
				b[x] = '.'
			}
		}
		rows[y] = string(b)
	}
	g := build(rows, map[byte]int{'.': 1})
	for y := size / 3; y < 2*size/3; y++ {
		for x := size / 3; x < 2*size/3; x++ {
			if w, _ := g.Weight(grid.Point{X: x, Y: y}); w == 1 {
				_ = g.SetWeight(grid.Point{X: x, Y: y}, 5)
			}
		}
	}

	t := time.Now()
	f, _ := grid.NewCostField(g, grid.Point{X: 0, Y: 0})
	fullDur := time.Since(t)

	p := grid.Point{X: size / 3, Y: size / 4}
	t = time.Now()
	stats, _ := f.UpdateWeight(p, 2)
	incDur := time.Since(t)

	fmt.Printf("  %3dx%-3d 可走=%d | 全量 %d 格 %v | 增量 %d 格 %v | 边界格 %d 个\n",
		size, size, g.WalkableCount(), g.WalkableCount(), fullDur,
		stats.Recomputed, incDur, len(stats.Boundary))
}
