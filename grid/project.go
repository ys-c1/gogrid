package grid

import (
	"errors"
	"fmt"
	"math"
	"sort"
)

// 视线投射（line-of-sight projection）相关错误。错误信息以参数名开头，
// 便于调用方直接判断是哪个量不合法；原有 Visible 接口的错误语义不变。
var (
	ErrAngle       = errors.New("angle")
	ErrRadius      = errors.New("radius")
	ErrGrid        = errors.New("grid")
	ErrOrigin      = errors.New("origin")
	ErrLOSMismatch = errors.New("逐格射线对照不一致")
	ErrWallCross   = errors.New("射线跨过墙体")
	ErrRayBudget   = errors.New("射线数超出反解值")
)

const (
	// losEps 用于角度与扇区边界的浮点比较。
	losEps = 1e-9
	// rayEps 用于 DDA 中两条网格线交点的判定。
	rayEps = 1e-12
)

// CastConfig 描述一次扇形视线投射。
//
// 坐标系与 Grid 一致：x 向右、y 向下，角度 0° 指向 +x，90° 指向 +y。
//   - Origin：观察点，必须是可行走格；
//   - CenterDeg：扇形中线角度（度），必须有限；张角 360° 时不影响结果；
//   - AngleDeg：张角（度），取值 (0, 360]；
//   - Radius：欧氏半径（格），取值 [1, MaxRadius()]。
type CastConfig struct {
	Origin    Point
	CenterDeg float64
	AngleDeg  float64
	Radius    int
}

// MaxRadius 返回合法半径上界：网格外接矩形的对角线向上取整。
func (g *Grid) MaxRadius() int {
	if g == nil || g.W <= 0 || g.H <= 0 {
		return 0
	}
	return int(math.Ceil(math.Hypot(float64(g.W), float64(g.H))))
}

// RayBudget 由张角与半径反解扫掠所需的射线数。
//
// 反解依据：最外圈半径 R+0.5 处相邻射线之间的弧长不得超过 1 格，
// 于是角步长上限为 1/(R+0.5) 弧度，所需条数为
//
//	ceil(张角弧度 × (R + 0.5))
//
// Project 恰好投射这么多条射线，因此实际条数恒不超过反解值。
func RayBudget(angleDeg float64, radius int) int {
	if math.IsNaN(angleDeg) || angleDeg <= 0 || radius < 1 {
		return 0
	}
	a := angleDeg
	if math.IsInf(a, 0) || a > 360 {
		a = 360
	}
	n := int(math.Ceil(a * math.Pi / 180 * (float64(radius) + 0.5)))
	if n < 1 {
		n = 1
	}
	return n
}

// Ray 是扫掠中的一条射线。Cells 从观察点开始按经过顺序排列；
// 若撞墙，墙格是最后一个元素且 HitWall 为 true。射线在墙处停止，
// 绝不会把墙另一侧的格加入 Cells。
type Ray struct {
	AngleRad float64
	Cells    []Point
	HitWall  bool
	Wall     Point
}

// AngleDeg 返回射线角度的度数形式。
func (r Ray) AngleDeg() float64 { return r.AngleRad * 180 / math.Pi }

// Occluded 记录一个被遮挡的可走格，以及其逐格射线上第一个挡住它的墙格。
type Occluded struct {
	Cell Point
	Wall Point
}

// RangeField 是可见距离场：每个可见格记录其格心到观察点格心的欧氏距离。
// 不在扇区内或被遮挡的格没有条目。Size() 即距离场规模（条目数）。
type RangeField struct {
	w, h int
	dist []float64 // 下标 y*w+x；-1 表示无条目
	size int
}

// Size 返回距离场规模（可见格数）。
func (f *RangeField) Size() int {
	if f == nil {
		return 0
	}
	return f.size
}

// Dist 返回坐标 p 到观察点的距离，p 不在场中时第二返回值为 false。
func (f *RangeField) Dist(p Point) (float64, bool) {
	if f == nil || p.X < 0 || p.Y < 0 || p.X >= f.w || p.Y >= f.h {
		return 0, false
	}
	d := f.dist[p.Y*f.w+p.X]
	return d, d >= 0
}

func newRangeField(g *Grid, origin Point, cells []Point) *RangeField {
	f := &RangeField{w: g.W, h: g.H, dist: make([]float64, g.W*g.H)}
	for i := range f.dist {
		f.dist[i] = -1
	}
	for _, c := range cells {
		f.dist[c.Y*g.W+c.X] = math.Hypot(float64(c.X-origin.X), float64(c.Y-origin.Y))
		f.size++
	}
	return f
}

// Projection 是一次视线投射的完整结果。
//
// 计算分三步：
//  1. 按 RayBudget 反解出的条数等角步长转动，扫出可见扇区（DDA 射线，
//     撞墙即停）；
//  2. 用逐格 Bresenham 射线（与 Visible 同一判定）得到权威可见集；
//  3. 两者逐格对照：不一致的格由逐格射线裁定并计入回退次数，
//     再由最终可见集反推被遮挡格及其挡墙。
//
// 因此最终可见集与逐格射线逐格全等（CheckConsistency 独立复核），
// 而原始扫掠的一致率单独由 Stats 报告；两者分开核验。
type Projection struct {
	g      *Grid
	cfg    CastConfig
	budget int

	rays      []Ray
	raw       []Point
	rawSet    map[Point]bool
	visible   []Point
	occluded  []Occluded
	fallbacks []Point
	field     *RangeField
}

// CastStats 汇总一次投射的射线数、一致率与回退情况，供收尾报告。
type CastStats struct {
	W, H     int
	Origin   Point
	Center   float64
	Angle    float64
	Radius   int
	Budget   int // 由张角与半径反解的射线数
	Rays     int // 实际投射的扫掠射线数
	RawCells int // 原始扫掠可见格数
	Cells    int // 最终（逐格射线裁定后）可见格数
	Matched  int // 原始扫掠与逐格射线一致的格数
	Extra    int // 原始扫掠多出的格数
	Missing  int // 原始扫掠漏掉的格数
	Fallback int // 回退次数 = Extra + Missing
	Occluded int // 被遮挡的可走格数
	Range    int // 可见距离场规模
	// RawAgreement 为原始扫掠相对逐格射线的一致率（交并比）：
	// Matched / (RawCells + Cells - Matched)。最终一致率恒为 1。
	RawAgreement float64
}

// Project 执行一次扇形视线投射。张角为零（含负数、NaN、Inf）、
// 半径越界、网格为空三类输入分别返回带参数名的错误。
// 原有 Visible(a,b) 逐格视线接口保持不变。
func (g *Grid) Project(cfg CastConfig) (*Projection, error) {
	if err := g.ValidateCast(cfg); err != nil {
		return nil, err
	}

	p := &Projection{
		g:      g,
		cfg:    cfg,
		budget: RayBudget(cfg.AngleDeg, cfg.Radius),
		rawSet: make(map[Point]bool),
	}

	// 步骤一：等角步长扫掠。射线条数严格等于反解值。
	aRad := cfg.AngleDeg * math.Pi / 180
	start := (cfg.CenterDeg - cfg.AngleDeg/2) * math.Pi / 180
	step := aRad / float64(p.budget)
	p.rays = make([]Ray, 0, p.budget)
	for i := 0; i < p.budget; i++ {
		theta := start + (float64(i)+0.5)*step
		ray := g.traceRay(cfg.Origin, theta, cfg.Radius)
		p.rays = append(p.rays, ray)
		for _, c := range ray.Cells {
			if !g.cells[c.Y][c.X] {
				continue // 墙格不算可见格
			}
			if c != cfg.Origin && !g.inSector(c, cfg) {
				continue
			}
			p.rawSet[c] = true
		}
	}
	for c := range p.rawSet {
		p.raw = append(p.raw, c)
	}
	sortPoints(p.raw)

	// 步骤二：逐格 Bresenham 射线作为权威可见集，并由它反推遮挡。
	visible, occluded, err := g.perCellDetail(cfg)
	if err != nil {
		return nil, err
	}
	p.visible = visible
	p.occluded = occluded

	// 步骤三：逐格对照，不一致的格由逐格射线裁定，计入回退。
	refSet := make(map[Point]bool, len(visible))
	for _, c := range visible {
		refSet[c] = true
	}
	for _, c := range p.raw {
		if !refSet[c] {
			p.fallbacks = append(p.fallbacks, c)
		}
	}
	for _, c := range visible {
		if !p.rawSet[c] {
			p.fallbacks = append(p.fallbacks, c)
		}
	}
	sortPoints(p.fallbacks)

	p.field = newRangeField(g, cfg.Origin, p.visible)
	return p, nil
}

// PerCellVisible 只用逐格 Bresenham 射线求扇形内的可见格，不做扫掠。
// 它是 Project 的对照基准：扇形几何掩码内的每个可走格都用 Visible 判定，
// 结果按行优先排列。
func (g *Grid) PerCellVisible(cfg CastConfig) ([]Point, error) {
	if err := g.ValidateCast(cfg); err != nil {
		return nil, err
	}
	visible, _, err := g.perCellDetail(cfg)
	return visible, err
}

func (g *Grid) perCellDetail(cfg CastConfig) ([]Point, []Occluded, error) {
	var visible []Point
	var occluded []Occluded
	for _, c := range g.sectorCells(cfg) {
		ok, err := g.Visible(cfg.Origin, c)
		if err != nil {
			return nil, nil, err
		}
		if ok {
			visible = append(visible, c)
			continue
		}
		wall, found := g.firstWallOnLine(cfg.Origin, c)
		if !found {
			// 理论上不可达：逐格射线没有墙却判为不可见。
			return nil, nil, fmt.Errorf("%w: (%d,%d) 不可见但找不到挡墙",
				ErrLOSMismatch, c.X, c.Y)
		}
		occluded = append(occluded, Occluded{Cell: c, Wall: wall})
	}
	return visible, occluded, nil
}

// sectorCells 返回扇形几何掩码内的全部可走格（半径与张角，不判遮挡），
// 按行优先排列。
func (g *Grid) sectorCells(cfg CastConfig) []Point {
	r2 := cfg.Radius * cfg.Radius
	out := make([]Point, 0)
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if !g.cells[y][x] {
				continue
			}
			dx, dy := x-cfg.Origin.X, y-cfg.Origin.Y
			if dx*dx+dy*dy > r2 {
				continue
			}
			if !g.inSector(Point{X: x, Y: y}, cfg) {
				continue
			}
			out = append(out, Point{X: x, Y: y})
		}
	}
	return out
}

func (g *Grid) inSector(p Point, cfg CastConfig) bool {
	if p == cfg.Origin || cfg.AngleDeg >= 360 {
		return true
	}
	deg := math.Atan2(float64(p.Y-cfg.Origin.Y), float64(p.X-cfg.Origin.X)) * 180 / math.Pi
	diff := math.Mod(deg-cfg.CenterDeg+180, 360)
	if diff < 0 {
		diff += 360
	}
	diff -= 180
	return math.Abs(diff) <= cfg.AngleDeg/2+losEps
}

func (g *Grid) ValidateCast(cfg CastConfig) error {
	if g == nil || g.W <= 0 || g.H <= 0 || len(g.cells) != g.H {
		w, h := 0, 0
		if g != nil {
			w, h = g.W, g.H
		}
		return fmt.Errorf("%w: 网格为空（W=%d, H=%d）", ErrGrid, w, h)
	}
	for y := 0; y < g.H; y++ {
		if len(g.cells[y]) != g.W {
			return fmt.Errorf("%w: 网格第 %d 行宽 %d，应为 %d", ErrGrid, y, len(g.cells[y]), g.W)
		}
	}
	if math.IsNaN(cfg.AngleDeg) || math.IsInf(cfg.AngleDeg, 0) ||
		cfg.AngleDeg <= 0 || cfg.AngleDeg > 360 {
		return fmt.Errorf("%w: 张角必须在 (0, 360] 度内（收到 %v）", ErrAngle, cfg.AngleDeg)
	}
	if math.IsNaN(cfg.CenterDeg) || math.IsInf(cfg.CenterDeg, 0) {
		return fmt.Errorf("%w: 中线角度必须有限（收到 %v）", ErrAngle, cfg.CenterDeg)
	}
	if cfg.Radius < 1 || cfg.Radius > g.MaxRadius() {
		return fmt.Errorf("%w: 半径必须在 [1, %d] 内（收到 %d）",
			ErrRadius, g.MaxRadius(), cfg.Radius)
	}
	if ok, err := g.Walkable(cfg.Origin); err != nil {
		return fmt.Errorf("%w: 观察点 (%d,%d) 越界（网格 %dx%d）",
			ErrOrigin, cfg.Origin.X, cfg.Origin.Y, g.W, g.H)
	} else if !ok {
		return fmt.Errorf("%w: 观察点 (%d,%d) 是墙", ErrOrigin, cfg.Origin.X, cfg.Origin.Y)
	}
	return nil
}

func (g *Grid) inGrid(p Point) bool {
	return p.X >= 0 && p.Y >= 0 && p.X < g.W && p.Y < g.H
}

// traceRay 从观察点格心按角度 theta 做一次 DDA 射线遍历。
// 进入墙格立即停止（墙格作为最后一个元素）；离开网格或超出半径也停止。
// 射线恰好穿过网格角点时，若两侧任一正交格是墙则视为撞墙，不允许斜穿墙缝。
func (g *Grid) traceRay(o Point, theta float64, radius int) Ray {
	r := Ray{AngleRad: theta}
	x, y := o.X, o.Y
	dirX, dirY := math.Cos(theta), math.Sin(theta)
	r2 := float64(radius * radius)

	stepX, stepY := 0, 0
	tMaxX, tMaxY := math.Inf(1), math.Inf(1)
	tDeltaX, tDeltaY := math.Inf(1), math.Inf(1)
	if math.Abs(dirX) > rayEps {
		if dirX > 0 {
			stepX = 1
		} else {
			stepX = -1
		}
		tMaxX = 0.5 / math.Abs(dirX)
		tDeltaX = 1 / math.Abs(dirX)
	}
	if math.Abs(dirY) > rayEps {
		if dirY > 0 {
			stepY = 1
		} else {
			stepY = -1
		}
		tMaxY = 0.5 / math.Abs(dirY)
		tDeltaY = 1 / math.Abs(dirY)
	}

	for {
		cur := Point{X: x, Y: y}
		if !g.inGrid(cur) {
			return r
		}
		dx, dy := float64(x-o.X), float64(y-o.Y)
		if dx*dx+dy*dy > r2 {
			return r
		}
		r.Cells = append(r.Cells, cur)
		if !g.cells[y][x] {
			r.HitWall = true
			r.Wall = cur
			return r
		}
		switch {
		case stepX == 0:
			y += stepY
			tMaxY += tDeltaY
		case stepY == 0:
			x += stepX
			tMaxX += tDeltaX
		case math.Abs(tMaxX-tMaxY) <= rayEps:
			// 穿过网格角点。按几何顺序考虑对角格与两侧格：
			// 对角格是墙时射线下一步就进入它的内部，以它为挡墙；
			// 对角格可走但任一邻边是墙时，视为卡在零宽墙缝上，就地停下。
			diag := Point{X: x + stepX, Y: y + stepY}
			sideA := Point{X: x + stepX, Y: y}
			sideB := Point{X: x, Y: y + stepY}
			if !g.inGrid(diag) || !g.inGrid(sideA) || !g.inGrid(sideB) {
				return r
			}
			if !g.cells[diag.Y][diag.X] {
				if ddx, ddy := float64(diag.X-o.X), float64(diag.Y-o.Y); ddx*ddx+ddy*ddy > r2 {
					return r
				}
				r.Cells = append(r.Cells, diag)
				r.HitWall = true
				r.Wall = diag
				return r
			}
			if !g.cells[sideA.Y][sideA.X] || !g.cells[sideB.Y][sideB.X] {
				hit := sideA
				if !g.cells[sideB.Y][sideB.X] && g.cells[sideA.Y][sideA.X] {
					hit = sideB
				}
				if ddx, ddy := float64(hit.X-o.X), float64(hit.Y-o.Y); ddx*ddx+ddy*ddy > r2 {
					return r
				}
				r.Cells = append(r.Cells, hit)
				r.HitWall = true
				r.Wall = hit
				return r
			}
			x, y = diag.X, diag.Y
			tMaxX += tDeltaX
			tMaxY += tDeltaY
		case tMaxX < tMaxY:
			x += stepX
			tMaxX += tDeltaX
		default:
			y += stepY
			tMaxY += tDeltaY
		}
	}
}

// firstWallOnLine 沿与 Visible 相同的 Bresenham 路径返回第一个墙格。
// 两端都在网格内时，路径不会离开网格。
func (g *Grid) firstWallOnLine(a, b Point) (Point, bool) {
	x0, y0 := a.X, a.Y
	x1, y1 := b.X, b.Y
	dx, dy := abs(x1-x0), abs(y1-y0)
	sx, sy := 1, 1
	if x0 > x1 {
		sx = -1
	}
	if y0 > y1 {
		sy = -1
	}
	err := dx - dy
	for {
		if !g.cells[y0][x0] {
			return Point{X: x0, Y: y0}, true
		}
		if x0 == x1 && y0 == y1 {
			return Point{}, false
		}
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			x0 += sx
		}
		if e2 < dx {
			err += dx
			y0 += sy
		}
	}
}

// Config 返回本次投射的配置。
func (p *Projection) Config() CastConfig { return p.cfg }

// Rays 返回全部扫掠射线的副本。
func (p *Projection) Rays() []Ray {
	out := make([]Ray, len(p.rays))
	for i, r := range p.rays {
		out[i] = Ray{AngleRad: r.AngleRad, Cells: append([]Point(nil), r.Cells...),
			HitWall: r.HitWall, Wall: r.Wall}
	}
	return out
}

// RaysUsed 返回实际投射的扫掠射线数。
func (p *Projection) RaysUsed() int { return len(p.rays) }

// Budget 返回由张角与半径反解的射线数。
func (p *Projection) Budget() int { return p.budget }

// Visible 返回最终可见格（逐格射线裁定后），按行优先排列。
func (p *Projection) Visible() []Point { return append([]Point(nil), p.visible...) }

// RawVisible 返回原始扫掠的可见格，按行优先排列。
func (p *Projection) RawVisible() []Point { return append([]Point(nil), p.raw...) }

// Occluded 返回被遮挡的可走格及其挡墙，按行优先排列。
func (p *Projection) Occluded() []Occluded {
	return append([]Occluded(nil), p.occluded...)
}

// Fallbacks 返回回退次数：原始扫掠与逐格射线不一致、最终由逐格射线裁定的格数。
func (p *Projection) Fallbacks() int { return len(p.fallbacks) }

// FallbackCells 返回需要回退裁定的格，按行优先排列。
func (p *Projection) FallbackCells() []Point {
	return append([]Point(nil), p.fallbacks...)
}

// RangeField 返回可见距离场。
func (p *Projection) RangeField() *RangeField { return p.field }

// Stats 汇总射线数、一致率与回退情况。
func (p *Projection) Stats() CastStats {
	refSet := make(map[Point]bool, len(p.visible))
	for _, c := range p.visible {
		refSet[c] = true
	}
	matched := 0
	for _, c := range p.raw {
		if refSet[c] {
			matched++
		}
	}
	union := len(p.raw) + len(p.visible) - matched
	agreement := 1.0
	if union > 0 {
		agreement = float64(matched) / float64(union)
	}
	return CastStats{
		W: p.g.W, H: p.g.H,
		Origin: p.cfg.Origin, Center: p.cfg.CenterDeg, Angle: p.cfg.AngleDeg,
		Radius: p.cfg.Radius,
		Budget: p.budget, Rays: len(p.rays),
		RawCells: len(p.raw), Cells: len(p.visible),
		Matched:      matched,
		Extra:        len(p.raw) - matched,
		Missing:      len(p.visible) - matched,
		Fallback:     len(p.fallbacks),
		Occluded:     len(p.occluded),
		Range:        p.field.Size(),
		RawAgreement: agreement,
	}
}

// CheckConsistency 独立重算逐格 Bresenham 可见集，与最终可见集逐格全等核验。
func (p *Projection) CheckConsistency() error {
	ref, err := p.g.PerCellVisible(p.cfg)
	if err != nil {
		return err
	}
	onlyProj, onlyRef := diffPoints(p.visible, ref)
	if len(onlyProj) > 0 || len(onlyRef) > 0 {
		return fmt.Errorf("%w: 投射多出 %v，漏掉 %v", ErrLOSMismatch, onlyProj, onlyRef)
	}
	return nil
}

// CheckNoWallCrossing 独立复核扫掠射线没有跨过墙体。这一项与可见集核验分开：
//   - 每条射线的墙格若存在，必须是最后一个元素，且撞墙标记与末端格一致；
//   - 几何复核：沿真实射线方向细步采样，在射线停下之前不得进入任何墙格方块
//     的内部；撞墙射线还必须在停下处正好进入报告的那面墙；
//   - 射线在半径与网格边界内，且可见集里没有墙格。
func (p *Projection) CheckNoWallCrossing() error {
	for i, ray := range p.rays {
		if len(ray.Cells) == 0 {
			return fmt.Errorf("%w: 第 %d 条射线没有经过任何格", ErrWallCross, i)
		}
		for j, c := range ray.Cells {
			if !p.g.inGrid(c) {
				return fmt.Errorf("%w: 第 %d 条射线经过越界格 (%d,%d)",
					ErrWallCross, i, c.X, c.Y)
			}
			if !p.g.cells[c.Y][c.X] && j != len(ray.Cells)-1 {
				return fmt.Errorf("%w: 第 %d 条射线在 (%d,%d) 撞墙后仍进入 (%d,%d)",
					ErrWallCross, i, c.X, c.Y, ray.Cells[j+1].X, ray.Cells[j+1].Y)
			}
			d2 := float64((c.X-p.cfg.Origin.X)*(c.X-p.cfg.Origin.X) + (c.Y-p.cfg.Origin.Y)*(c.Y-p.cfg.Origin.Y))
			if d2 > float64(p.cfg.Radius*p.cfg.Radius) {
				return fmt.Errorf("%w: 第 %d 条射线越过半径，进入 (%d,%d)",
					ErrWallCross, i, c.X, c.Y)
			}
		}
		last := ray.Cells[len(ray.Cells)-1]
		lastIsWall := !p.g.cells[last.Y][last.X]
		if ray.HitWall != lastIsWall {
			return fmt.Errorf("%w: 第 %d 条射线的撞墙标记 %v 与末端格状态不符",
				ErrWallCross, i, ray.HitWall)
		}
		if ray.HitWall && ray.Wall != last {
			return fmt.Errorf("%w: 第 %d 条射线的挡墙 (%d,%d) 不是末端格 (%d,%d)",
				ErrWallCross, i, ray.Wall.X, ray.Wall.Y, last.X, last.Y)
		}
		if err := p.g.checkRayGeometry(p.cfg.Origin, ray, p.cfg.Radius); err != nil {
			return fmt.Errorf("%w: 第 %d 条射线（%.4f°）：%v",
				ErrWallCross, i, ray.AngleDeg(), err)
		}
	}
	for _, c := range p.raw {
		if !p.g.cells[c.Y][c.X] {
			return fmt.Errorf("%w: 原始扫掠把墙格 (%d,%d) 当成可见格", ErrWallCross, c.X, c.Y)
		}
	}
	for _, c := range p.visible {
		if !p.g.cells[c.Y][c.X] {
			return fmt.Errorf("%w: 最终可见集把墙格 (%d,%d) 当成可见格", ErrWallCross, c.X, c.Y)
		}
	}
	return nil
}

// checkRayGeometry 用几何区间独立复核射线的路径，不重放 DDA：
// 先求射线与最近墙格的接触点作为独立算出的停止参数，再枚举该点之前
// 射线真正“穿过内部”（区间非退化）的格。判据：
//   - 停止点之前不得有墙格内部被穿过——擦过角点或边线不算跨墙；
//   - 撞墙射线允许且只允许报告的那面墙被穿入内部（正常步进撞墙），
//     若挡墙只是擦角停下（区间退化）则不要求穿入内部；
//   - 停止点之后的几何路径不参与判定，与 DDA 撞墙即停的语义一致。
//
// 半径口径与 traceRay 相同：格心超出半径的格不计。
func (g *Grid) checkRayGeometry(origin Point, ray Ray, radius int) error {
	ox, oy := float64(origin.X), float64(origin.Y)
	dirX, dirY := math.Cos(ray.AngleRad), math.Sin(ray.AngleRad)
	r2 := float64(radius * radius)

	tStop := math.Inf(1)
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if g.cells[y][x] {
				continue
			}
			dx, dy := float64(x-origin.X), float64(y-origin.Y)
			if dx*dx+dy*dy > r2 {
				continue
			}
			tEnter, tExit, ok := raySquareIntervalClosed(ox, oy, dirX, dirY, Point{X: x, Y: y})
			if !ok || tExit <= 0 {
				continue
			}
			if tEnter < 0 {
				tEnter = 0
			}
			if tEnter < tStop {
				tStop = tEnter
			}
		}
	}

	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			c := Point{X: x, Y: y}
			if c == origin {
				continue
			}
			dx, dy := float64(x-origin.X), float64(y-origin.Y)
			if dx*dx+dy*dy > r2 {
				continue
			}
			if g.cells[y][x] {
				continue // 可走格
			}
			tEnter, tExit, ok := raySquareInterval(ox, oy, dirX, dirY, c)
			if !ok || tExit <= 0 {
				continue // 只擦角或擦边，不算穿入内部
			}
			if tEnter < 0 {
				tEnter = 0
			}
			if tEnter >= tStop+1e-6 || tExit <= tStop+1e-6 {
				continue // 停止点之后（含仅擦过停止点）
			}
			// 允许穿过内部的唯一墙是报告的挡墙。
			if !ray.HitWall || c != ray.Wall {
				return fmt.Errorf("穿过墙体 (%d,%d) 的内部", c.X, c.Y)
			}
		}
	}
	return nil
}

// raySquareIntervalClosed 与 raySquareInterval 相同，但方块按闭区间处理，
// 擦角与擦边也返回 ok=true（区间可退化）。
func raySquareIntervalClosed(ox, oy, dx, dy float64, c Point) (tEnter, tExit float64, ok bool) {
	loX, hiX := float64(c.X)-0.5, float64(c.X)+0.5
	loY, hiY := float64(c.Y)-0.5, float64(c.Y)+0.5
	tEnter, tExit = math.Inf(-1), math.Inf(1)
	if math.Abs(dx) < rayEps {
		if ox < loX || ox > hiX {
			return 0, 0, false
		}
	} else {
		t1 := (loX - ox) / dx
		t2 := (hiX - ox) / dx
		if t1 > t2 {
			t1, t2 = t2, t1
		}
		if t1 > tEnter {
			tEnter = t1
		}
		if t2 < tExit {
			tExit = t2
		}
	}
	if math.Abs(dy) < rayEps {
		if oy < loY || oy > hiY {
			return 0, 0, false
		}
	} else {
		t1 := (loY - oy) / dy
		t2 := (hiY - oy) / dy
		if t1 > t2 {
			t1, t2 = t2, t1
		}
		if t1 > tEnter {
			tEnter = t1
		}
		if t2 < tExit {
			tExit = t2
		}
	}
	if tExit < tEnter {
		return 0, 0, false
	}
	return tEnter, tExit, true
}

// raySquareInterval 返回从 (ox,oy) 沿 (dx,dy) 出发的射线进入/离开以格心
// (cx,cy) 为中心、边长为 1 的方块开内部所对应的参数区间。
// 射线只擦过角点时区间退化，返回 ok=false。
func raySquareInterval(ox, oy, dx, dy float64, c Point) (tEnter, tExit float64, ok bool) {
	loX, hiX := float64(c.X)-0.5, float64(c.X)+0.5
	loY, hiY := float64(c.Y)-0.5, float64(c.Y)+0.5
	tEnter, tExit = math.Inf(-1), math.Inf(1)
	if math.Abs(dx) < rayEps {
		if ox <= loX || ox >= hiX {
			return 0, 0, false
		}
	} else {
		t1 := (loX - ox) / dx
		t2 := (hiX - ox) / dx
		if t1 > t2 {
			t1, t2 = t2, t1
		}
		if t1 > tEnter {
			tEnter = t1
		}
		if t2 < tExit {
			tExit = t2
		}
	}
	if math.Abs(dy) < rayEps {
		if oy <= loY || oy >= hiY {
			return 0, 0, false
		}
	} else {
		t1 := (loY - oy) / dy
		t2 := (hiY - oy) / dy
		if t1 > t2 {
			t1, t2 = t2, t1
		}
		if t1 > tEnter {
			tEnter = t1
		}
		if t2 < tExit {
			tExit = t2
		}
	}
	if !(tExit > tEnter) {
		return 0, 0, false
	}
	return tEnter, tExit, true
}

// CheckBudget 核验实际射线数不超过由张角反解的值。
func (p *Projection) CheckBudget() error {
	derived := RayBudget(p.cfg.AngleDeg, p.cfg.Radius)
	if len(p.rays) > derived {
		return fmt.Errorf("%w: 实际 %d 条 > 反解 %d 条", ErrRayBudget, len(p.rays), derived)
	}
	return nil
}

func diffPoints(a, b []Point) (onlyA, onlyB []Point) {
	inA := make(map[Point]bool, len(a))
	inB := make(map[Point]bool, len(b))
	for _, p := range a {
		inA[p] = true
	}
	for _, p := range b {
		inB[p] = true
	}
	for _, p := range a {
		if !inB[p] {
			onlyA = append(onlyA, p)
		}
	}
	for _, p := range b {
		if !inA[p] {
			onlyB = append(onlyB, p)
		}
	}
	return onlyA, onlyB
}

func sortPoints(pts []Point) {
	sort.Slice(pts, func(i, j int) bool {
		if pts[i].Y != pts[j].Y {
			return pts[i].Y < pts[j].Y
		}
		return pts[i].X < pts[j].X
	})
}
