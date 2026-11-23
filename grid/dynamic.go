package grid

import (
	"container/heap"
	"fmt"
)

// MaxRecomputeRatio 是重算格数占全量格数的预算上限：三成。
const MaxRecomputeRatio = 0.30

// Change 描述一次障碍变化：Walkable=true 开门，false 关门。
type Change struct {
	P        Point
	Walkable bool
}

// Stats 汇报一次增量重规划的工作量。
type Stats struct {
	// Touched 本次重算实际重新赋值距离的格子数，含变化格本身。
	// 仅被访问但距离未变的格子不计入——这些格子的距离场被原样复用。
	Touched int
	// FullCount 变化后网格的可行走格总数，即一次全量重算需要处理的规模。
	FullCount int
}

// Ratio 重算格数 / 全量格数。
func (s Stats) Ratio() float64 {
	if s.FullCount == 0 {
		return 0
	}
	return float64(s.Touched) / float64(s.FullCount)
}

// OverBudget 是否超过三成预算。
func (s Stats) OverBudget() bool { return s.Ratio() > MaxRecomputeRatio }

// distNode 是多源 Dijkstra 最小堆的元素。
type distNode struct {
	id   int
	dist int
}

type distHeap []*distNode

func (h distHeap) Len() int            { return len(h) }
func (h distHeap) Less(i, j int) bool  { return h[i].dist < h[j].dist }
func (h distHeap) Swap(i, j int)       { h[i], h[j] = h[j], h[i] }
func (h *distHeap) Push(x interface{}) { *h = append(*h, x.(*distNode)) }
func (h *distHeap) Pop() interface{} {
	old := *h
	n := old[len(old)-1]
	*h = old[:len(old)-1]
	return n
}

// Update 对一批障碍变化做动态重规划，只重算受影响区域，其余距离场原值复用。
//
// 纯局部算法，不做遍历整个连通分量的全量扫描，共三步：
//
//  1. 应用变化。关门成墙的格距离立即置 Inf；开门格放行。
//  2. 不动点区域生长求重算区 R：R 初始为开门格与关门格的可行走邻居；
//     一个 R 外格只要还存在"R 外、有限且 dist[u]+1==dist[v]"的最短父节点
//     u，其旧距离就被佐证仍成立并冻结，否则纳入 R。该判据同时覆盖三种
//     情况——关门切断（→Inf）、绕路变长（距离抬升）、开门连通（Inf→有限）。
//     R 只单调增长，生长过程仅入队 R 的邻居，天然局限在变化周边。
//  3. 单次多源 Dijkstra：以 R 边界（拥有 R 外有限邻居的 R 格，键值取
//     邻居距离+1）与终点（在 R 内时键值 0）为种子，做"只变短"松弛。
//     R 格先清空为 Inf 后被重新填充，R 外格距离已是最优、不会被改善，
//     开门产生的捷径则沿同一松弛过程自然向外传播。一次遍历同时完成
//     抬升重建与捷径下降。
//
// 结果与从终点全量 BFS 逐格相同（由测试逐格对照保证）。
// 连续多批调用即支持连续变化；越界坐标返回带坐标的错误。
func (f *Field) Update(changes []Change) (Stats, error) {
	w, h := f.g.W, f.g.H
	f.ensureBuffers()
	f.tick++
	tick := f.tick
	f.growQ = f.growQ[:0]
	f.touched = 0
	if f.heap == nil {
		f.heap = &distHeap{}
	}
	f.heap = &distHeap{}
	heap.Init(f.heap)

	// 同一格在一批中多次变化时去重，以最后一次状态为准。
	var opened, closed []int
	changed := false
	for _, c := range changes {
		if c.P.X < 0 || c.P.Y < 0 || c.P.X >= w || c.P.Y >= h {
			return Stats{}, fmt.Errorf("%w: 变化坐标 (%d,%d) 不在 %dx%d 网格内", ErrOOB, c.P.X, c.P.Y, w, h)
		}
		id := c.P.Y*w + c.P.X
		if f.seen[id] == tick {
			continue
		}
		f.seen[id] = tick
		if f.g.cells[c.P.Y][c.P.X] == c.Walkable {
			continue // 状态未变，空操作
		}
		f.g.cells[c.P.Y][c.P.X] = c.Walkable
		changed = true
		if c.Walkable {
			opened = append(opened, id)
			f.openMark[id] = tick
		} else {
			closed = append(closed, id)
			f.dist[id] = Inf // 关门成墙：距离立即失效，与全量 BFS 的墙语义一致
		}
	}
	if !changed {
		return Stats{Touched: 0, FullCount: f.g.WalkableCount()}, nil
	}

	baseline := f.g.WalkableCount()
	goalID := f.goal.Y*w + f.goal.X
	touch := func(id int) {
		if f.touchedMark[id] != tick {
			f.touchedMark[id] = tick
			f.touched++
		}
	}

	// 终点被堵：距离场失去根，所有可行走格都不可达，整体置 Inf。
	// 这是唯一无法局部化的变化，重算比例必然为 100%。
	if !f.g.cells[f.goal.Y][f.goal.X] {
		for i := range f.dist {
			if f.g.cells[i/w][i%w] {
				f.dist[i] = Inf
				touch(i)
			}
		}
		return Stats{Touched: f.touched, FullCount: baseline}, nil
	}

	// 步骤二：求重算区 R。
	addR := func(id int) {
		if f.inR[id] != tick {
			f.inR[id] = tick
			f.growQ = append(f.growQ, id)
		}
	}
	for _, id := range opened {
		addR(id) // 开门格本身距离待定
	}
	for _, id := range closed {
		for _, np := range f.neighbors(id) {
			addR(np) // 关门格的邻居父节点受影响
		}
	}
	// 不动点生长：R 外格失去全部 R 外最短父节点 => 旧距离不再可信，纳入 R。
	for head := 0; head < len(f.growQ); head++ {
		id := f.growQ[head]
		for _, np := range f.neighbors(id) {
			if f.inR[np] == tick {
				continue
			}
			if f.hasFrozenParent(np, tick) {
				continue
			}
			addR(np)
		}
	}

	// 步骤三：收集多源 Dijkstra 种子（在清空 R 之前，先借 R 外锚点定值）。
	type seed struct{ id, d int }
	var seeds []seed
	if f.inR[goalID] == tick {
		seeds = append(seeds, seed{goalID, 0}) // 终点重开：距离 0 的根
	}
	for i := range f.dist {
		if f.inR[i] != tick {
			continue
		}
		best := Inf
		for _, np := range f.neighbors(i) {
			if f.inR[np] == tick || f.dist[np] == Inf {
				continue
			}
			if shorter(f.dist[np]+1, best) {
				best = f.dist[np] + 1
			}
		}
		if best != Inf {
			seeds = append(seeds, seed{i, best})
		}
	}
	// R 内旧距离整体清空：抬升场景下旧的较小残留值无法被"只变短"松弛覆盖。
	for i := range f.dist {
		if f.inR[i] == tick {
			f.dist[i] = Inf
		}
	}
	// 多源 Dijkstra：在全图上做只变短松弛。R 格被填充；R 外格已是最优不受影响；
	// 开门捷径沿同一过程传播到任何真正变短的格子。
	for _, s := range seeds {
		if shorter(s.d, f.dist[s.id]) {
			f.dist[s.id] = s.d
			heap.Push(f.heap, &distNode{id: s.id, dist: s.d})
			touch(s.id)
		}
	}
	for f.heap.Len() > 0 {
		top := heap.Pop(f.heap).(*distNode)
		id, d := top.id, top.dist
		if d != f.dist[id] || f.done[id] == tick {
			continue
		}
		f.done[id] = tick
		for _, np := range f.neighbors(id) {
			nd := d + 1
			if shorter(nd, f.dist[np]) {
				f.dist[np] = nd
				touch(np)
				heap.Push(f.heap, &distNode{id: np, dist: nd})
			}
		}
	}
	// R 内仍为 Inf 的格即新图不可达，保持 Inf，计入重算量。
	for i := range f.dist {
		if f.inR[i] == tick {
			touch(i)
		}
	}

	return Stats{Touched: f.touched, FullCount: baseline}, nil
}

// hasFrozenParent 判断 R 外的格 v 是否仍有 R 外、有限且满足
// dist[u]+1 == dist[v] 的最短父节点 u。存在则 v 的旧最短距离在新图中仍成立。
// 特殊情况：Inf 格（含此前密封房间）没有有限父节点，需纳入 R 重算；
// dist==0 的格即终点，其距离恒为 0（只要仍可行走），自证冻结、无需父节点，
// 否则终点会被错误吸入 R，进而把整个连通分量连锁纳入、重算量虚高到整图。
func (f *Field) hasFrozenParent(id, tick int) bool {
	dv := f.dist[id]
	if dv == Inf {
		return false
	}
	if dv == 0 {
		return true
	}
	for _, np := range f.neighbors(id) {
		if f.inR[np] != tick && f.dist[np] != Inf && f.dist[np]+1 == dv {
			return true
		}
	}
	return false
}

// shorter 判定候选距离 cand 是否严格优于当前距离 cur。
// Inf(-1) 代表不可达，任何有限候选都比它短；不能直接用整数比较，
// 否则负数哨兵会让所有有限值都"大于" Inf，松弛永远不触发。
func shorter(cand, cur int) bool {
	if cur == Inf {
		return cand != Inf
	}
	if cand == Inf {
		return false
	}
	return cand < cur
}

// ensureBuffers 惰性分配增量算法所需的队列与戳记数组。
func (f *Field) ensureBuffers() {
	n := len(f.dist)
	if len(f.inR) == n {
		return
	}
	f.growQ = make([]int, 0, 64)
	f.inR = make([]int, n)
	f.done = make([]int, n)
	f.touchedMark = make([]int, n)
	f.seen = make([]int, n)
	f.openMark = make([]int, n)
}

// neighbors 枚举某格当前可行走的四邻域 id。
func (f *Field) neighbors(id int) []int {
	w, h := f.g.W, f.g.H
	x, y := id%w, id/w
	out := make([]int, 0, 4)
	for _, dir := range dirs {
		nx, ny := x+dir.X, y+dir.Y
		if nx < 0 || ny < 0 || nx >= w || ny >= h || !f.g.cells[ny][nx] {
			continue
		}
		out = append(out, ny*w+nx)
	}
	return out
}
