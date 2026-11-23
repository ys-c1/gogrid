package grid

import (
	"fmt"
)

// Inf 表示该格无法到达终点。
const Inf = -1

// dirs 四邻域，顺序固定，保证逐格比对与路径复现的确定性。
var dirs = [4]Point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}

// Field 是以终点 goal 为根的 BFS 距离场。
// dist[y*W+x] 为该格到 goal 的最短步数，Inf 表示不可达。
// 距离场可在障碍变化时做局部增量更新，未受影响的格子保持原值复用。
type Field struct {
	g    *Grid
	goal Point
	dist []int

	// 增量重规划的工作队列与戳记（惰性分配，见 ensureBuffers）。
	growQ         []int
	inR           []int // 重算区域 R 的成员戳记
	done          []int // Dijkstra 定稿戳记
	touchedMark   []int // 重算格计数戳记
	seen          []int // 本批变化去重戳记
	openMark      []int // 开门格戳记
	heap          *distHeap
	tick, touched int
}

// NewField 构建初始全量距离场。
// 网格为空、终点越界或终点不可走时返回带坐标的错误。
func NewField(g *Grid, goal Point) (*Field, error) {
	if g == nil || g.W == 0 || g.H == 0 || len(g.cells) == 0 {
		return nil, fmt.Errorf("%w: 无法构建距离场", ErrEmpty)
	}
	ok, err := g.Walkable(goal)
	if err != nil {
		return nil, fmt.Errorf("终点 %w: 坐标 (%d,%d) 不在 %dx%d 网格内", ErrOOB, goal.X, goal.Y, g.W, g.H)
	}
	if !ok {
		return nil, fmt.Errorf("%w: 终点坐标 (%d,%d) 不可走", ErrNoPath, goal.X, goal.Y)
	}
	f := &Field{g: g, goal: goal, dist: make([]int, g.W*g.H)}
	f.fullBFS()
	return f, nil
}

// Goal 返回距离场根节点（终点）。
func (f *Field) Goal() Point { return f.goal }

// Grid 返回距离场绑定的网格。
func (f *Field) Grid() *Grid { return f.g }

// Dist 查询某格到终点的最短步数，Inf 为不可达。越界返回带坐标的错误。
func (f *Field) Dist(p Point) (int, error) {
	if p.X < 0 || p.Y < 0 || p.X >= f.g.W || p.Y >= f.g.H {
		return 0, fmt.Errorf("%w: 坐标 (%d,%d) 不在 %dx%d 网格内", ErrOOB, p.X, p.Y, f.g.W, f.g.H)
	}
	return f.dist[p.Y*f.g.W+p.X], nil
}

// Distances 返回距离场快照的拷贝，供逐格对照使用。
func (f *Field) Distances() []int {
	out := make([]int, len(f.dist))
	copy(out, f.dist)
	return out
}

// fullBFS 从终点做一次全量 BFS，是正确性基准与初始构建方式。
// 返回到达（赋值过距离）的格子数。终点自身不可走时全部格子保持 Inf。
func (f *Field) fullBFS() int {
	for i := range f.dist {
		f.dist[i] = Inf
	}
	if !f.g.cells[f.goal.Y][f.goal.X] {
		return 0
	}
	w, h := f.g.W, f.g.H
	root := f.goal.Y*w + f.goal.X
	f.dist[root] = 0
	queue := make([]int, 0, f.g.WalkableCount())
	queue = append(queue, root)
	reached := 0
	for head := 0; head < len(queue); head++ {
		id := queue[head]
		reached++
		x, y := id%w, id/w
		d := f.dist[id]
		for _, dir := range dirs {
			nx, ny := x+dir.X, y+dir.Y
			if nx < 0 || ny < 0 || nx >= w || ny >= h || !f.g.cells[ny][nx] {
				continue
			}
			nid := ny*w + nx
			if f.dist[nid] != Inf {
				continue
			}
			f.dist[nid] = d + 1
			queue = append(queue, nid)
		}
	}
	return reached
}

// FullRecompute 在不改变当前距离场的前提下做一次全量重算，
// 返回重算结果快照与被赋值（可达）的格子数，用于逐格对照与耗时基准。
func (f *Field) FullRecompute() ([]int, int) {
	saved := f.Distances()
	reached := f.fullBFS()
	out := f.Distances()
	copy(f.dist, saved)
	return out, reached
}

// Clone 连同网格一起深拷贝，用于计时时反复从同一状态出发。
func (f *Field) Clone() *Field {
	return &Field{
		g:    f.g.Clone(),
		goal: f.goal,
		dist: append([]int(nil), f.dist...),
	}
}

// Path 沿距离场从 from 下降到终点：每步走到距离恰好小 1 的邻格。
// 距离场正确时，得到的路径与全量重算下的最短路径逐格等价。
// 起点越界/不可走/不可达分别返回带坐标的错误。
func (f *Field) Path(from Point) ([]Point, error) {
	ok, err := f.g.Walkable(from)
	if err != nil {
		return nil, fmt.Errorf("起点 %w: 坐标 (%d,%d) 不在 %dx%d 网格内", ErrOOB, from.X, from.Y, f.g.W, f.g.H)
	}
	if !ok {
		return nil, fmt.Errorf("%w: 起点坐标 (%d,%d) 不可走", ErrNoPath, from.X, from.Y)
	}
	w := f.g.W
	if f.dist[from.Y*w+from.X] == Inf {
		return nil, fmt.Errorf("%w: 起点坐标 (%d,%d) 无法到达终点 (%d,%d)", ErrNoPath, from.X, from.Y, f.goal.X, f.goal.Y)
	}
	path := []Point{from}
	at := from
	for at != f.goal {
		cur := f.dist[at.Y*w+at.X]
		next := Point{-1, -1}
		for _, dir := range dirs {
			np := Point{at.X + dir.X, at.Y + dir.Y}
			if np.X < 0 || np.Y < 0 || np.X >= f.g.W || np.Y >= f.g.H {
				continue
			}
			if f.dist[np.Y*w+np.X] == cur-1 {
				next = np
				break
			}
		}
		if next.X < 0 {
			return nil, fmt.Errorf("%w: 距离场在 (%d,%d) 处断裂", ErrNoPath, at.X, at.Y)
		}
		path = append(path, next)
		at = next
	}
	return path, nil
}
