package grid

import (
	"errors"
	"fmt"
	"math"
)

// nearestNeighborOrder 从第一个目标出发，每次走向最近的未访问目标。
func nearestNeighborOrder(d [][]int) []int {
	n := len(d)
	order := make([]int, 0, n)
	used := make([]bool, n)
	cur := 0
	order = append(order, cur)
	used[cur] = true
	for len(order) < n {
		next, best := -1, math.MaxInt
		for j := 0; j < n; j++ {
			if used[j] {
				continue
			}
			if d[cur][j] < best {
				best = d[cur][j]
				next = j
			}
		}
		used[next] = true
		order = append(order, next)
		cur = next
	}
	return order
}

func orderLength(order []int, d [][]int) int {
	total := 0
	for k := 0; k+1 < len(order); k++ {
		total += d[order[k]][order[k+1]]
	}
	return total
}

// exchangeDelta 计算交换 order 中下标 i、j 两个元素的长度变化。
func exchangeDelta(order []int, d [][]int, i, j int) int {
	n := len(order)
	if i > j {
		i, j = j, i
	}
	if i == j || j >= n || i < 0 {
		return 0
	}
	a, b := order[i], order[j]
	delta := 0
	if j == i+1 {
		// 相邻：只有左右两条边被替换。
		if i > 0 {
			delta += d[order[i-1]][b] - d[order[i-1]][a]
		}
		if j+1 < n {
			delta += d[order[j+1]][a] - d[order[j+1]][b]
		}
		return delta
	}
	// 不相邻：各自的前驱与后继共四条边被替换。
	if i > 0 {
		delta += d[order[i-1]][b] - d[order[i-1]][a]
	}
	delta += d[order[i+1]][b] - d[order[i+1]][a]
	delta += d[order[j-1]][a] - d[order[j-1]][b]
	if j+1 < n {
		delta += d[order[j+1]][a] - d[order[j+1]][b]
	}
	return delta
}

// reversalDelta 计算把 order 的 [i..j] 区间整体反转的长度变化。
// 反转只改动区间两端的各一条边，内部方向对称、长度不变。
func reversalDelta(order []int, d [][]int, i, j int) int {
	n := len(order)
	if i >= j || i < 0 || j >= n {
		return 0
	}
	delta := 0
	if i > 0 {
		delta += d[order[i-1]][order[j]] - d[order[i-1]][order[i]]
	}
	if j+1 < n {
		delta += d[order[j+1]][order[i]] - d[order[j+1]][order[j]]
	}
	return delta
}

func reverse(order []int, i, j int) {
	for i < j {
		order[i], order[j] = order[j], order[i]
		i++
		j--
	}
}

// improveOrder 用二次交换改进访问顺序：相邻/非相邻两点交换，
// 以及区间反转（2-opt）。返回改进后的顺序、扫描轮数与接受的交换次数。
//
// 接受准则决定哪些移动会被执行：
//   - AcceptStrict 只接受 delta < 0 的移动；
//   - AcceptNonWorsening 还接受 delta == 0 的横向移动，用于在平台区探索。
//
// 两种准则都绝不接受 delta > 0 的移动，因此总长度单调不增。
// 只有出现严格变短的轮次才会继续下一轮；一轮里若只做了等长移动，
// 该轮结束后即停止，避免横向移动无限打转。轮数另受 MaxPasses 约束。
func improveOrder(d [][]int, order []int, opts TourOptions) ([]int, int, int) {
	n := len(order)
	maxPasses := opts.MaxPasses
	if maxPasses == 0 {
		maxPasses = DefaultTourPasses
	}
	sideways := opts.Accept == AcceptNonWorsening
	passes, swaps := 0, 0
	for passes < maxPasses {
		passes++
		progressed := false
		for i := 0; i+1 < n; i++ {
			for j := i + 1; j < n; j++ {
				if opts.MaxSwaps > 0 && swaps >= opts.MaxSwaps {
					return order, passes, swaps
				}
				if delta := exchangeDelta(order, d, i, j); delta < 0 || (sideways && delta == 0) {
					order[i], order[j] = order[j], order[i]
					swaps++
					if delta < 0 {
						progressed = true
					}
					continue
				}
				if delta := reversalDelta(order, d, i, j); delta < 0 || (sideways && delta == 0) {
					reverse(order, i, j)
					swaps++
					if delta < 0 {
						progressed = true
					}
				}
			}
		}
		if !progressed {
			break
		}
	}
	return order, passes, swaps
}

func pointsAt(stops []Point, order []int) []Point {
	out := make([]Point, len(order))
	for k, idx := range order {
		out[k] = stops[idx]
	}
	return out
}

// verifyOrderWithDist 校验 order 恰是 stops 的一个排列，并返回其总长度。
// 任何重复、缺漏或外来坐标都会返回 ErrTourInvariant。
func verifyOrderWithDist(stops, order []Point, d [][]int) (int, error) {
	if len(order) != len(stops) {
		return 0, fmt.Errorf("%w: 顺序长度 %d 与点集大小 %d 不符",
			ErrTourInvariant, len(order), len(stops))
	}
	index := make(map[Point]int, len(stops))
	for i, p := range stops {
		index[p] = i
	}
	seen := make([]bool, len(stops))
	length := 0
	for k, p := range order {
		idx, ok := index[p]
		if !ok {
			return 0, fmt.Errorf("%w: 顺序中的坐标(%d,%d)不在点集内",
				ErrTourInvariant, p.X, p.Y)
		}
		if seen[idx] {
			return 0, fmt.Errorf("%w: 坐标(%d,%d)被重复访问",
				ErrTourInvariant, p.X, p.Y)
		}
		seen[idx] = true
		if k > 0 {
			length += d[index[order[k-1]]][idx]
		}
	}
	return length, nil
}

// optimalWithDist 暴力枚举全排列，返回最优顺序与长度。
func optimalWithDist(d [][]int) ([]int, int, error) {
	n := len(d)
	if n > BruteForceLimit {
		return nil, 0, fmt.Errorf("%w: %d 个点超过上限 %d", ErrTooManyStops, n, BruteForceLimit)
	}
	order := make([]int, n)
	used := make([]bool, n)
	bestOrder := make([]int, n)
	bestLen := math.MaxInt
	var walk func(k, length int)
	walk = func(k, length int) {
		if length >= bestLen {
			return
		}
		if k == n {
			bestLen = length
			copy(bestOrder, order)
			return
		}
		for j := 0; j < n; j++ {
			if used[j] {
				continue
			}
			used[j] = true
			order[k] = j
			add := 0
			if k > 0 {
				add = d[order[k-1]][j]
			}
			walk(k+1, length+add)
			used[j] = false
		}
	}
	walk(0, 0)
	if bestLen == math.MaxInt {
		return nil, 0, errors.New("暴力枚举失败")
	}
	return bestOrder, bestLen, nil
}
