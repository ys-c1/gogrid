package grid

import (
	"errors"
	"fmt"
	"math"
)

// ErrNoRegion 表示网格中不存在任何可走连通域（例如全墙），
// 或查询坐标落在墙体上、不属于任何区域。
var ErrNoRegion = errors.New("不存在可走区域")

// Region 描述一个四连通可走区域。
type Region struct {
	ID       int     // 区域编号，从 1 开始，按最左上格的扫描顺序分配
	Area     int     // 区域内格子数 N
	Cells    []Point // 区域内全部格子
	Mean     [2]float64
	Centroid Point // 标注用质心：Mean 落在区域内即该格，否则为区域内距 Mean 最近的可走格
}

// MeanX / MeanY 返回区域内格子坐标的算术平均值（精确质心）。
func (r Region) MeanX() float64 { return r.Mean[0] }
func (r Region) MeanY() float64 { return r.Mean[1] }

// CentroidTolerance 由区域面积 N 反解允许的质心偏差。
//
// 反解依据：面积 N 的区域，其空间尺度（直径量级）为 √N。
// 当精确质心落在区域外（典型为环形区域），区域内可走格到质心的
// 最坏距离与区域直径同阶，故取 ε(N)=√N（格距单位，欧氏距离）。
// 同时平均坐标的单轴量化分辨率为 1/√N。
func CentroidTolerance(area int) float64 {
	if area <= 0 {
		return 0
	}
	return math.Sqrt(float64(area))
}

// Deviation 返回标注质心与精确质心之间的欧氏距离。
func (r Region) Deviation() float64 {
	dx := float64(r.Centroid.X) - r.Mean[0]
	dy := float64(r.Centroid.Y) - r.Mean[1]
	return math.Hypot(dx, dy)
}

// WithinTolerance 判断实际偏差是否不超过由面积反解的精度值。
func (r Region) WithinTolerance() bool {
	return r.Deviation() <= CentroidTolerance(r.Area)+1e-9
}

// LabelComponents 对网格做四连通连通域标记，返回按编号排序的区域列表。
// 连通口径与 Path 的四方向移动一致（上下左右，不含对角）。
// 全墙网格返回 ErrNoRegion；空网格（Parse 阶段已拦截）同样返回 ErrEmpty。
func (g *Grid) LabelComponents() ([]Region, error) {
	if g == nil || g.W == 0 || g.H == 0 {
		return nil, ErrEmpty
	}
	seen := make([][]bool, g.H)
	for y := range seen {
		seen[y] = make([]bool, g.W)
	}
	dirs := [4]Point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}
	var regions []Region
	id := 0
	for sy := 0; sy < g.H; sy++ {
		for sx := 0; sx < g.W; sx++ {
			if !g.cells[sy][sx] || seen[sy][sx] {
				continue
			}
			id++
			region := Region{ID: id}
			queue := []Point{{sx, sy}}
			seen[sy][sx] = true
			sumX, sumY := 0, 0
			for len(queue) > 0 {
				cur := queue[0]
				queue = queue[1:]
				region.Cells = append(region.Cells, cur)
				sumX += cur.X
				sumY += cur.Y
				for _, d := range dirs {
					np := Point{cur.X + d.X, cur.Y + d.Y}
					if np.X < 0 || np.Y < 0 || np.X >= g.W || np.Y >= g.H {
						continue
					}
					if seen[np.Y][np.X] || !g.cells[np.Y][np.X] {
						continue
					}
					seen[np.Y][np.X] = true
					queue = append(queue, np)
				}
			}
			region.Area = len(region.Cells)
			region.Mean = [2]float64{
				float64(sumX) / float64(region.Area),
				float64(sumY) / float64(region.Area),
			}
			region.Centroid = nearestCell(region.Cells, region.Mean)
			regions = append(regions, region)
		}
	}
	if len(regions) == 0 {
		return nil, ErrNoRegion
	}
	return regions, nil
}

// nearestCell 返回 cells 中距 mean 欧氏距离最近的格子。
// 平局规则（保证结果确定）：距离相同取 X 较小者，再相同取 Y 较小者。
func nearestCell(cells []Point, mean [2]float64) Point {
	best := cells[0]
	bestD := math.Hypot(float64(best.X)-mean[0], float64(best.Y)-mean[1])
	for _, c := range cells[1:] {
		d := math.Hypot(float64(c.X)-mean[0], float64(c.Y)-mean[1])
		if d < bestD || (d == bestD && (c.X < best.X || (c.X == best.X && c.Y < best.Y))) {
			best, bestD = c, d
		}
	}
	return best
}

// CentroidOf 返回指定编号区域的标注质心。
// 编号越界时返回携带编号与有效范围的 ErrOOB；全墙/空网格返回对应错误。
func (g *Grid) CentroidOf(regionID int) (Point, error) {
	regions, err := g.LabelComponents()
	if err != nil {
		return Point{}, err
	}
	if regionID < 1 || regionID > len(regions) {
		return Point{}, fmt.Errorf("%w: 区域编号 %d 超出有效范围 [1, %d]",
			ErrOOB, regionID, len(regions))
	}
	return regions[regionID-1].Centroid, nil
}

// CentroidAt 返回包含坐标 p 的区域的标注质心。
// p 越界返回携带坐标的 ErrOOB；p 为墙体（不属于任何区域）返回携带坐标的 ErrNoRegion。
func (g *Grid) CentroidAt(p Point) (Point, error) {
	if p.X < 0 || p.Y < 0 || p.X >= g.W || p.Y >= g.H {
		return Point{}, fmt.Errorf("%w: 坐标 (%d,%d) 不在 %dx%d 网格内",
			ErrOOB, p.X, p.Y, g.W, g.H)
	}
	if !g.cells[p.Y][p.X] {
		return Point{}, fmt.Errorf("%w: 坐标 (%d,%d) 是墙体，不属于任何区域",
			ErrNoRegion, p.X, p.Y)
	}
	regions, err := g.LabelComponents()
	if err != nil {
		return Point{}, err
	}
	for _, r := range regions {
		for _, c := range r.Cells {
			if c == p {
				return r.Centroid, nil
			}
		}
	}
	// 理论不可达：p 可走则必然属于某个连通域。
	return Point{}, fmt.Errorf("%w: 坐标 (%d,%d) 未找到所属区域", ErrNoRegion, p.X, p.Y)
}
