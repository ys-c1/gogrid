package grid

import (
	"errors"
	"fmt"
	"strings"
)

// 区域生长相关错误。
var (
	// ErrNoSeed 种子数量为零。
	ErrNoSeed = errors.New("种子数量为零")
	// ErrSeedOOB 种子坐标越界，携带具体坐标。
	ErrSeedOOB = errors.New("种子坐标越界")
	// ErrSeedWall 种子落在墙内，携带具体坐标。
	ErrSeedWall = errors.New("种子落在墙内")
	// ErrSeedDup 多个种子落在同一格，携带具体坐标。
	ErrSeedDup = errors.New("种子坐标重复")
	// ErrTooFew 可走格数量不足以每个种子分到一格，携带坐标与数量。
	ErrTooFew = errors.New("可走格不足")
	// ErrBadRatio 极差比例参数非法。
	ErrBadRatio = errors.New("极差比例非法")
	// ErrUnreachable 存在与任何种子都不连通的可走格，携带该格坐标。
	ErrUnreachable = errors.New("存在不可达的可走格")
)

// Region 单个辖区：种子坐标与所辖全部格。
type Region struct {
	Seed  Point
	Cells []Point
}

// Area 返回辖区面积（格数）。
func (r Region) Area() int { return len(r.Cells) }

// Partition 一次区域生长的结果。
type Partition struct {
	g       *Grid
	Regions []Region
}

// Area 返回第 i 个辖区的面积。
func (p *Partition) Area(i int) int { return p.Regions[i].Area() }

// RangeStat 返回各辖区面积的最小值、最大值与极差（max-min）。
func (p *Partition) RangeStat() (min, max, spread int) {
	min, max = -1, -1
	for _, r := range p.Regions {
		a := r.Area()
		if min < 0 || a < min {
			min = a
		}
		if a > max {
			max = a
		}
	}
	return min, max, max - min
}

// GrowRegions 从若干种子出发做确定性等面积区域生长。
//
// 每个区域只在面积未达目标时接受新格；边界格按与本区共享边的长度
// （相邻格数）择优，共享边长度相同则按行优先坐标（y 再 x）取最靠前者，
// 全程不使用随机数。目标面积为总可走格数按种子数均分后的向上取整，
// 余数个区域（按种子输入顺序）各多承担一格。
func (g *Grid) GrowRegions(seeds []Point) (*Partition, error) {
	if len(seeds) == 0 {
		return nil, ErrNoSeed
	}

	// 先查越界（越界格无从判断墙），再查总量，最后查墙与重复；
	// 这样全墙地图报“可走格不足”，而单个落墙种子报“种子落在墙内”。
	for i, s := range seeds {
		if s.X < 0 || s.Y < 0 || s.X >= g.W || s.Y >= g.H {
			return nil, fmt.Errorf("%w: 第%d个种子 (%d,%d)", ErrSeedOOB, i, s.X, s.Y)
		}
	}

	n := g.WalkableCount()
	if n < len(seeds) {
		s := seeds[0]
		return nil, fmt.Errorf("%w: 可走格 %d，种子 %d 个，首个种子 (%d,%d)",
			ErrTooFew, n, len(seeds), s.X, s.Y)
	}

	seen := make(map[Point]int, len(seeds))
	for i, s := range seeds {
		if !g.cells[s.Y][s.X] {
			return nil, fmt.Errorf("%w: 第%d个种子 (%d,%d)", ErrSeedWall, i, s.X, s.Y)
		}
		if j, dup := seen[s]; dup {
			return nil, fmt.Errorf("%w: 第%d个种子与第%d个种子同为 (%d,%d)",
				ErrSeedDup, i, j, s.X, s.Y)
		}
		seen[s] = i
	}

	k := len(seeds)
	base, rem := n/k, n%k
	target := make([]int, k)
	for i := range target {
		target[i] = base
		if i < rem {
			target[i]++
		}
	}

	owner := make([][]int, g.H)
	for y := range owner {
		owner[y] = make([]int, g.W)
		for x := range owner[y] {
			owner[y][x] = -1
		}
	}

	regions := make([]Region, k)
	frontiers := make([][]Point, k)
	for i, s := range seeds {
		owner[s.Y][s.X] = i
		regions[i] = Region{Seed: s, Cells: []Point{s}}
		frontiers[i] = g.frontierNeighbors(s, owner, i)
	}

	// 阶段一：按种子顺序轮流扩张，仅当面积未达本区域目标时才收格。
	for {
		progress := false
		for i := 0; i < k; i++ {
			if len(regions[i].Cells) >= target[i] {
				continue
			}
			best, ok := g.pickFrontier(frontiers[i], owner, i)
			if !ok {
				continue // 本区域暂时被卡死，留给阶段二处理
			}
			g.assign(best, i, owner, regions, frontiers)
			progress = true
		}
		if !progress {
			break
		}
	}

	// 阶段二：剩余可走格（被瓶颈地形卡住的）分给当前最小、仍有前沿的区域，
	// 最小面积相同取索引最小，保证所有可走格都被分配。
	for {
		bestRegion, bestCell, ok := -1, Point{}, false
		for i := 0; i < k; i++ {
			c, has := g.pickFrontier(frontiers[i], owner, i)
			if !has {
				continue
			}
			if !ok || len(regions[i].Cells) < len(regions[bestRegion].Cells) {
				bestRegion, bestCell, ok = i, c, true
			}
		}
		if !ok {
			break
		}
		g.assign(bestCell, bestRegion, owner, regions, frontiers)
	}

	// 仍有可走格未被分配：它与所有种子都不连通（独立封闭区域）。
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if g.cells[y][x] && owner[y][x] < 0 {
				return nil, fmt.Errorf("%w: 可走格 (%d,%d) 与任何种子都不连通",
					ErrUnreachable, x, y)
			}
		}
	}

	return &Partition{g: g, Regions: regions}, nil
}

// frontierNeighbors 返回 p 的四邻中可走且未分配的格。
func (g *Grid) frontierNeighbors(p Point, owner [][]int, regionID int) []Point {
	var out []Point
	for _, d := range [4]Point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}} {
		np := Point{p.X + d.X, p.Y + d.Y}
		if np.X < 0 || np.Y < 0 || np.X >= g.W || np.Y >= g.H {
			continue
		}
		if g.cells[np.Y][np.X] && owner[np.Y][np.X] < 0 {
			out = append(out, np)
		}
	}
	return out
}

// pickFrontier 从候选前沿中按“共享边长度优先、行优先坐标次之”择优。
// 共享边长度即候选格四邻中已属于 regionID 的格数（每边长度均为 1）。
func (g *Grid) pickFrontier(cands []Point, owner [][]int, regionID int) (Point, bool) {
	best, bestScore, found := Point{}, -1, false
	for _, c := range cands {
		if owner[c.Y][c.X] >= 0 {
			continue // 已在其他区域收格时被占用，惰性跳过
		}
		score := 0
		for _, d := range [4]Point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}} {
			np := Point{c.X + d.X, c.Y + d.Y}
			if np.X >= 0 && np.Y >= 0 && np.X < g.W && np.Y < g.H && owner[np.Y][np.X] == regionID {
				score++
			}
		}
		// 共享边长度严格优先；长度相同按行优先坐标（y 再 x）取最靠前者，
		// 与候选进入前沿的顺序无关，保证择优完全确定、可复现。
		if !found || score > bestScore || (score == bestScore && rowMajorBefore(c, best)) {
			best, bestScore, found = c, score, true
		}
	}
	return best, found
}

// rowMajorLess 行优先次序：先比 y（行），同行再比 x（列）。
func rowMajorBefore(a, b Point) bool {
	return a.Y < b.Y || (a.Y == b.Y && a.X < b.X)
}

// assign 把格 c 划入 regionID，并更新所有区域的前沿。
func (g *Grid) assign(c Point, regionID int, owner [][]int, regions []Region, frontiers [][]Point) {
	owner[c.Y][c.X] = regionID
	regions[regionID].Cells = append(regions[regionID].Cells, c)
	// 追加新暴露的未分配邻格；重复项与失效项由 pickFrontier 按 owner 惰性剔除，
	// 不能整体替换，否则会丢掉此前已在排队的候选格。
	frontiers[regionID] = append(frontiers[regionID], g.frontierNeighbors(c, owner, regionID)...)
}

// CheckCoverage 独立校验一：所有可走格恰好被分配一次，且墙格不被分配、
// 任何格不越界。返回未满足时的错误（含坐标），满足时返回 nil。
func (g *Grid) CheckCoverage(p *Partition) error {
	if p == nil {
		return errors.New("分区为空")
	}
	taken := make(map[Point]int)
	for i, r := range p.Regions {
		for _, c := range r.Cells {
			if c.X < 0 || c.Y < 0 || c.X >= g.W || c.Y >= g.H {
				return fmt.Errorf("辖区%d 含越界格 (%d,%d)", i, c.X, c.Y)
			}
			if !g.cells[c.Y][c.X] {
				return fmt.Errorf("辖区%d 含墙格 (%d,%d)", i, c.X, c.Y)
			}
			if j, dup := taken[c]; dup {
				return fmt.Errorf("格 (%d,%d) 同时被辖区%d与辖区%d分配", c.X, c.Y, j, i)
			}
			taken[c] = i
		}
	}
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if g.cells[y][x] {
				if _, ok := taken[Point{x, y}]; !ok {
					return fmt.Errorf("可走格 (%d,%d) 未被任何辖区分配", x, y)
				}
			}
		}
	}
	return nil
}

// CheckBalance 独立校验二：各辖区面积极差必须落在给定比例内。
// ratio 为允许的极差相对平均面积的比例（0.2 即极差不超过平均面积的 20%），
// 必须 >= 0。该校验与覆盖校验互不依赖，可单独调用。
func (g *Grid) CheckBalance(p *Partition, ratio float64) error {
	if p == nil {
		return errors.New("分区为空")
	}
	if ratio < 0 {
		return fmt.Errorf("%w: ratio=%g", ErrBadRatio, ratio)
	}
	if len(p.Regions) == 0 {
		return errors.New("分区无辖区")
	}
	min, max, spread := p.RangeStat()
	total := 0
	for _, r := range p.Regions {
		total += r.Area()
	}
	if total == 0 {
		return errors.New("辖区总面积为零")
	}
	mean := float64(total) / float64(len(p.Regions))
	got := float64(spread) / mean
	if got > ratio {
		return fmt.Errorf("面积极差 %d（最小 %d / 最大 %d），相对均值比例 %.4f 超过上限 %.4f",
			spread, min, max, got, ratio)
	}
	return nil
}

// Report 收尾报告：逐个辖区列出种子与面积，并给出面积极差与相对均值比例。
func (p *Partition) Report() string {
	var b strings.Builder
	min, max, spread := p.RangeStat()
	total := 0
	for _, r := range p.Regions {
		total += r.Area()
	}
	fmt.Fprintf(&b, "辖区数 %d，可走格 %d\n", len(p.Regions), total)
	for i, r := range p.Regions {
		fmt.Fprintf(&b, "辖区%d 种子(%d,%d) 面积 %d\n",
			i, r.Seed.X, r.Seed.Y, r.Area())
	}
	mean := float64(total) / float64(len(p.Regions))
	fmt.Fprintf(&b, "面积最小 %d，最大 %d，极差 %d，极差/均值 %.4f\n",
		min, max, spread, float64(spread)/mean)
	return b.String()
}
