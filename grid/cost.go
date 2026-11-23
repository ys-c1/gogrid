package grid

import (
	"container/heap"
	"fmt"
)

// 代价场错误复用 grid.go 的基础哨兵（ErrEmpty/ErrBlocked/ErrNegativeWeight），
// 在各调用点用 fmt.Errorf %w 包裹并附上坐标，可用 errors.Is 判断类别。

const inf = -1 // dist 中表示不可达

// UpdateStats 报告一次权重变更的增量更新结果。
type UpdateStats struct {
	Changed    Point   // 发生权重变更的格点
	OldWeight  int     // 变更前权重
	NewWeight  int     // 变更后权重
	Recomputed int     // 实际重算（重新赋距）的格数
	Boundary   []Point // 边界格：受影响区域外缘、距离保持不变、作为增量 Dijkstra 种子的格点
	Affected   []Point // 受影响区域内被重算的格点
	FellBack   bool    // 是否因无法限定受影响区域而回退到全量重算
}

// CostField 是以 Source 为源点的单源地形代价场。
// dist[y][x] 为从 Source 进入该格的最小累计代价，源点自身为 0，不可达为 -1。
type CostField struct {
	g      *Grid
	Source Point
	dist   [][]int
	parent [][]int // 一维下标，-1 表示无前驱（源点或不可达）
}

type costNode struct {
	idx int // 一维下标 y*W+x
	g   int
	pos int // 堆内位置
}

type costPQ []*costNode

func (q costPQ) Len() int            { return len(q) }
func (q costPQ) Less(i, j int) bool  { return q[i].g < q[j].g }
func (q costPQ) Swap(i, j int)       { q[i], q[j] = q[j], q[i]; q[i].pos, q[j].pos = i, j }
func (q *costPQ) Push(x interface{}) { n := x.(*costNode); n.pos = len(*q); *q = append(*q, n) }
func (q *costPQ) Pop() interface{} {
	old := *q
	n := old[len(old)-1]
	old[len(old)-1] = nil
	*q = old[:len(old)-1]
	return n
}

func (f *CostField) id(p Point) int { return p.Y*f.g.W + p.X }

func (f *CostField) at(id int) Point { return Point{X: id % f.g.W, Y: id / f.g.W} }

// 四邻域：返回可走邻居的一维下标（调用方保证 v 在界内）。
func (f *CostField) neighbors(v int) [4]int {
	w, h := f.g.W, f.g.H
	x, y := v%w, v/w
	out := [4]int{-1, -1, -1, -1}
	i := 0
	put := func(nx, ny int) {
		if nx >= 0 && nx < w && ny >= 0 && ny < h && f.g.cells[ny][nx] {
			out[i] = ny*w + nx
			i++
		}
	}
	put(x+1, y)
	put(x-1, y)
	put(x, y+1)
	put(x, y-1)
	return out
}

// NewCostField 按当前权重对全图求一次完整代价场。
// 网格为空、源点越界/不可走、存在负权重时返回带坐标的错误。
func NewCostField(g *Grid, source Point) (*CostField, error) {
	if g == nil || g.H == 0 || g.W == 0 {
		return nil, fmt.Errorf("%w: 无法建立代价场", ErrEmpty)
	}
	if source.X < 0 || source.Y < 0 || source.X >= g.W || source.Y >= g.H {
		return nil, fmt.Errorf("%w: 源点坐标%s", ErrOOB, source)
	}
	if !g.cells[source.Y][source.X] {
		return nil, fmt.Errorf("%w: 源点坐标%s 不可走", ErrBlocked, source)
	}
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if g.cells[y][x] && g.weights[y][x] < 0 {
				return nil, fmt.Errorf("%w: 坐标%s 权重=%d", ErrNegativeWeight, Point{x, y}, g.weights[y][x])
			}
		}
	}
	f := &CostField{
		g:      g,
		Source: source,
		dist:   make([][]int, g.H),
		parent: make([][]int, g.H),
	}
	for y := range f.dist {
		f.dist[y] = make([]int, g.W)
		f.parent[y] = make([]int, g.W)
		for x := range f.dist[y] {
			f.dist[y][x] = inf
			f.parent[y][x] = -1
		}
	}
	f.fullDijkstra()
	return f, nil
}

// fullDijkstra 全量重算：对全图每个可走格求一次最小代价，返回首次定距的格数。
func (f *CostField) fullDijkstra() int {
	src := f.id(f.Source)
	f.dist[f.Source.Y][f.Source.X] = 0
	q := &costPQ{}
	heap.Init(q)
	heap.Push(q, &costNode{idx: src, g: 0})
	settled := make([]bool, f.g.W*f.g.H) // 每个格只在首次定距（出堆）时计数一次
	recomputed := 0
	for q.Len() > 0 {
		cur := heap.Pop(q).(*costNode)
		p := f.at(cur.idx)
		if cur.g != f.dist[p.Y][p.X] || settled[cur.idx] {
			continue // 堆中的过期/重复条目
		}
		settled[cur.idx] = true
		recomputed++
		for _, nb := range f.neighbors(cur.idx) {
			if nb < 0 {
				continue
			}
			np := f.at(nb)
			ng := cur.g + f.g.weights[np.Y][np.X]
			old := f.dist[np.Y][np.X]
			// 严格更短，或等代价但前驱下标更小（确定性 tie-break，全量/增量逐格一致）。
			if old == inf || ng < old || (ng == old && cur.idx < f.parent[np.Y][np.X]) {
				f.dist[np.Y][np.X] = ng
				f.parent[np.Y][np.X] = cur.idx
				heap.Push(q, &costNode{idx: nb, g: ng})
			}
		}
	}
	return recomputed
}

// Recompute 强制全量重算，返回重算格数（全图可走格数）。
func (f *CostField) Recompute() int {
	for y := range f.dist {
		for x := range f.dist[y] {
			f.dist[y][x] = inf
			f.parent[y][x] = -1
		}
	}
	return f.fullDijkstra()
}

// CostAt 返回某格的全图代价，不可达返回 -1。
func (f *CostField) CostAt(p Point) (int, error) {
	if p.X < 0 || p.Y < 0 || p.X >= f.g.W || p.Y >= f.g.H {
		return 0, fmt.Errorf("%w: 坐标%s", ErrOOB, p)
	}
	return f.dist[p.Y][p.X], nil
}

// PathTo 返回到目标格的最低代价路径（含起点与终点）及路径总代价。
// 目标越界、不可走或不可达时返回带坐标的错误。
func (f *CostField) PathTo(to Point) ([]Point, int, error) {
	if to.X < 0 || to.Y < 0 || to.X >= f.g.W || to.Y >= f.g.H {
		return nil, 0, fmt.Errorf("%w: 终点坐标%s", ErrOOB, to)
	}
	if !f.g.cells[to.Y][to.X] {
		return nil, 0, fmt.Errorf("%w: 终点坐标%s 不可走", ErrBlocked, to)
	}
	if f.dist[to.Y][to.X] == inf {
		return nil, 0, fmt.Errorf("%w: 终点坐标%s 不可达", ErrNoPath, to)
	}
	path := []Point{}
	for cur := f.id(to); ; {
		p := f.at(cur)
		path = append([]Point{p}, path...)
		if cur == f.id(f.Source) {
			break
		}
		par := f.parent[p.Y][p.X]
		if par < 0 { // 理论上不可达已在上面拦截，防御性处理
			return nil, 0, fmt.Errorf("%w: 终点坐标%s 前驱缺失", ErrNoPath, to)
		}
		cur = par
	}
	return path, f.dist[to.Y][to.X], nil
}

// UpdateWeight 修改 p 的地形权重并增量刷新代价场。
// 负权重、越界、墙体均返回带坐标的错误且场保持不变。
// 当受影响区域无法被边界封死时自动回退全量重算（见 stats.FellBack）。
func (f *CostField) UpdateWeight(p Point, newWeight int) (UpdateStats, error) {
	stats := UpdateStats{Changed: p, NewWeight: newWeight}
	if p.X < 0 || p.Y < 0 || p.X >= f.g.W || p.Y >= f.g.H {
		return stats, fmt.Errorf("%w: 坐标%s", ErrOOB, p)
	}
	if newWeight < 0 {
		return stats, fmt.Errorf("%w: 坐标%s 权重=%d", ErrNegativeWeight, p, newWeight)
	}
	if !f.g.cells[p.Y][p.X] {
		return stats, fmt.Errorf("%w: 坐标%s 为墙体不可走", ErrBlocked, p)
	}
	old := f.g.weights[p.Y][p.X]
	stats.OldWeight = old
	if old == newWeight {
		return stats, nil
	}
	// 源点自身权重不进入任何距离公式（源点距离恒为 0），改了也不产生影响。
	if p == f.Source {
		f.g.weights[p.Y][p.X] = newWeight
		return stats, nil
	}
	// 变更格当前不可达（与源点不连通的孤岛）：它不参与任何源点最短路径，
	// 单格权重变更不会改变任何可达格的代价，直接落权重、零重算即可。
	if f.dist[p.Y][p.X] == inf {
		f.g.weights[p.Y][p.X] = newWeight
		return stats, nil
	}
	if newWeight > old {
		f.updateIncrease(p, old, newWeight, &stats)
	} else {
		f.updateDecrease(p, old, newWeight, &stats)
	}
	return stats, nil
}

// UpdateOrRecompute 是 UpdateWeight 的安全包装：
// 仅当坐标越界（调用方无法保证变更位置）时回退一次全量重算；
// 负权重/墙体等非法输入仍然原样报错，不做回退。
func (f *CostField) UpdateOrRecompute(p Point, newWeight int) (UpdateStats, error) {
	stats, err := f.UpdateWeight(p, newWeight)
	if err != nil && errorIs(err, ErrOOB) {
		stats.FellBack = true
		stats.Recomputed = f.Recompute()
		return stats, fmt.Errorf("%w；已回退全量重算", err)
	}
	return stats, err
}

func errorIs(err, target error) bool {
	type unwrapper interface{ Unwrap() error }
	for err != nil {
		if err == target {
			return true
		}
		u, ok := err.(unwrapper)
		if !ok {
			return false
		}
		err = u.Unwrap()
	}
	return false
}

// 权重上升：找出所有最短路径经过 p 的格点（受影响区域），
// 用外缘边界格做种子重新跑有界 Dijkstra。
func (f *CostField) updateIncrease(p Point, old, newW int, stats *UpdateStats) {
	w := f.g.W
	target := f.id(p)

	// 标记受影响区域：沿“最短路径前驱指向当前格”的边反向传播。
	// 不变量：权重只在可走格上变更且始终有限，不改变墙体与连通性，因此可达格集合
	// 在代价场生命周期内恒定；且 UpdateWeight 已对不可达变更格短路。所以此处能到达
	// 的全部是可达格，不存在“原本不可达、借本区域重新连通”的情况，无需可达性闭包。
	mark := make([]int, f.g.W*f.g.H) // 0 未标记，1 受影响
	mark[target] = 1
	queue := []int{target}
	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		for _, nb := range f.neighbors(cur) {
			if nb < 0 || mark[nb] == 1 {
				continue
			}
			np := f.at(nb)
			// nb 的最短路径经过 cur（即经过 p），才会被波及。
			if f.parent[np.Y][np.X] == cur && f.dist[np.Y][np.X] != inf {
				mark[nb] = 1
				queue = append(queue, nb)
			}
		}
	}

	// 收集边界格：受影响格的、自身不在区域内的可走邻居，其距离保持旧值。
	// 边界格都是可达格（不可达格不与可达格邻接，否则它本就可达），可直接作为种子。
	boundSet := map[int]bool{}
	for id, m := range mark {
		if m != 1 {
			continue
		}
		for _, nb := range f.neighbors(id) {
			if nb < 0 || mark[nb] == 1 {
				continue
			}
			nbp := f.at(nb)
			if f.dist[nbp.Y][nbp.X] != inf {
				boundSet[nb] = true
			}
		}
	}
	stats.Boundary = sortedPoints(boundSet, w)
	stats.Affected = sortedMarked(mark, w)

	// 没有任何边界种子且区域不含源点（理论上不会发生：可达变更格必属于源点连通块，
	// 其受影响区域要么含源点、要么外缘有可达边界）。作为防御性兜底，回退全量重算。
	srcInRegion := mark[f.id(f.Source)] == 1
	if len(boundSet) == 0 && !srcInRegion {
		f.g.weights[p.Y][p.X] = newW
		stats.FellBack = true
		stats.Recomputed = f.Recompute()
		stats.Boundary = nil
		stats.Affected = nil
		return
	}

	// 应用新权重并清空受影响格的距离。
	f.g.weights[p.Y][p.X] = newW
	q := &costPQ{}
	heap.Init(q)
	for id, m := range mark {
		if m == 1 {
			pp := f.at(id)
			f.dist[pp.Y][pp.X] = inf
			f.parent[pp.Y][pp.X] = -1
		}
	}
	if srcInRegion {
		heap.Push(q, &costNode{idx: f.id(f.Source), g: 0})
	}
	for b := range boundSet {
		bp := f.at(b)
		heap.Push(q, &costNode{idx: b, g: f.dist[bp.Y][bp.X]})
	}
	f.runBounded(q, mark, stats)
}

// 权重下降：先重算 p 自身的最短距离与前驱，再以 p 为种子松弛后继，
// 只在真正变短（重赋距）时计数。
func (f *CostField) updateDecrease(p Point, old, newW int, stats *UpdateStats) {
	w := f.g.W
	target := f.id(p)
	f.g.weights[p.Y][p.X] = newW

	mark := make([]int, f.g.W*f.g.H) // 1 表示已重算（进入过有界区域）
	mark[target] = 1
	stats.Affected = append(stats.Affected, p)
	stats.Recomputed = 1

	// 先重算 p 自身：在所有邻居里找 dist(u)+新权重 的最小值（源点距离恒 0）。
	// 必须在 p 入堆之前完成，否则它会以旧距离先出堆、提前锁定前驱。
	best := f.dist[p.Y][p.X]
	bestParent := f.parent[p.Y][p.X]
	for _, nb := range f.neighbors(target) {
		if nb < 0 {
			continue
		}
		np := f.at(nb)
		d := f.dist[np.Y][np.X]
		if d == inf {
			continue
		}
		cand := d + newW
		if best == inf || cand < best || (cand == best && nb < bestParent) {
			best = cand
			bestParent = nb
		}
	}
	f.dist[p.Y][p.X] = best
	f.parent[p.Y][p.X] = bestParent

	q := &costPQ{}
	heap.Init(q)
	heap.Push(q, &costNode{idx: target, g: best})
	for q.Len() > 0 {
		cur := heap.Pop(q).(*costNode)
		cp := f.at(cur.idx)
		if cur.g != f.dist[cp.Y][cp.X] {
			continue
		}
		for _, nb := range f.neighbors(cur.idx) {
			if nb < 0 {
				continue
			}
			np := f.at(nb)
			ng := cur.g + f.g.weights[np.Y][np.X]
			oldDist := f.dist[np.Y][np.X]
			if oldDist == inf || ng < oldDist || (ng == oldDist && cur.idx < f.parent[np.Y][np.X]) {
				f.dist[np.Y][np.X] = ng
				f.parent[np.Y][np.X] = cur.idx
				if mark[nb] == 0 {
					mark[nb] = 1
					stats.Affected = append(stats.Affected, np)
					stats.Recomputed++
				}
				heap.Push(q, &costNode{idx: nb, g: ng})
			}
		}
	}
	// 下降场景的“边界”：被重算格的外缘邻居（距离未变），单独列出供核对。
	boundSet := map[int]bool{}
	for id, m := range mark {
		if m != 1 {
			continue
		}
		for _, nb := range f.neighbors(id) {
			if nb >= 0 && mark[nb] == 0 {
				boundSet[nb] = true
			}
		}
	}
	stats.Boundary = sortedPoints(boundSet, w)
}

// runBounded 以边界格为种子，只在受影响区域（mark==1）内松弛赋距。
func (f *CostField) runBounded(q *costPQ, mark []int, stats *UpdateStats) {
	for q.Len() > 0 {
		cur := heap.Pop(q).(*costNode)
		cp := f.at(cur.idx)
		if cur.g != f.dist[cp.Y][cp.X] {
			continue
		}
		for _, nb := range f.neighbors(cur.idx) {
			if nb < 0 || mark[nb] != 1 {
				continue // 只重算受影响区域内部
			}
			np := f.at(nb)
			ng := cur.g + f.g.weights[np.Y][np.X]
			old := f.dist[np.Y][np.X]
			if old == inf || ng < old || (ng == old && cur.idx < f.parent[np.Y][np.X]) {
				f.dist[np.Y][np.X] = ng
				f.parent[np.Y][np.X] = cur.idx
				heap.Push(q, &costNode{idx: nb, g: ng})
			}
		}
	}
	recomputed := 0
	for id, m := range mark {
		if m == 1 {
			pp := f.at(id)
			if f.dist[pp.Y][pp.X] != inf {
				recomputed++
			}
		}
	}
	stats.Recomputed = recomputed
}

func sortedPoints(set map[int]bool, w int) []Point {
	out := make([]Point, 0, len(set))
	for id := range set {
		out = append(out, Point{X: id % w, Y: id / w})
	}
	sortPoints(out)
	return out
}

func sortedMarked(mark []int, w int) []Point {
	out := []Point{}
	for id, m := range mark {
		if m == 1 {
			out = append(out, Point{X: id % w, Y: id / w})
		}
	}
	sortPoints(out)
	return out
}

func sortPoints(ps []Point) {
	for i := 1; i < len(ps); i++ {
		for j := i; j > 0; j-- {
			a, b := ps[j-1], ps[j]
			if a.Y < b.Y || (a.Y == b.Y && a.X < b.X) {
				break
			}
			ps[j-1], ps[j] = b, a
		}
	}
}

// Equals 逐格比较两个代价场的代价与前驱是否完全相同，用于增量与全量结果对照。
func (f *CostField) Equals(other *CostField) bool {
	if f.g.W != other.g.W || f.g.H != other.g.H {
		return false
	}
	for y := 0; y < f.g.H; y++ {
		for x := 0; x < f.g.W; x++ {
			if f.dist[y][x] != other.dist[y][x] || f.parent[y][x] != other.parent[y][x] {
				return false
			}
		}
	}
	return true
}

// FullRecompute 强制全量重算并返回重算格数（实际定距的可走格数）。
func (f *CostField) FullRecompute() int { return f.Recompute() }

// Clone 深拷贝网格（含墙体与权重），供对照实验在同一份输入上分别建场。
func (g *Grid) Clone() *Grid {
	ng := &Grid{
		W:       g.W,
		H:       g.H,
		cells:   make([][]bool, g.H),
		weights: make([][]int, g.H),
	}
	for y := range g.cells {
		ng.cells[y] = append([]bool(nil), g.cells[y]...)
		ng.weights[y] = append([]int(nil), g.weights[y]...)
	}
	return ng
}
