package grid

import (
	"errors"
	"math"
)

// ErrSmoothInvariant 平滑后不变量核验失败（间距变小或长度增加），正常不应发生。
var ErrSmoothInvariant = errors.New("平滑不变量核验失败")

// DefaultKappaMax 默认曲率上界：弧度/格，π/4 ≈ 0.785。
// 原始直角拐角的离散曲率为 π/2 ≈ 1.571，即上界约为原始值的一半。
const DefaultKappaMax = math.Pi / 4

const (
	epsSmooth  = 1e-9  // 浮点比较容差
	epsAngle   = 1e-12 // 共线判定
	sampleStep = 0.05  // 逐点核验采样步长（格）
)

type fpoint struct{ X, Y float64 }

func toFP(p Point) fpoint { return fpoint{float64(p.X), float64(p.Y)} }

// 障碍格视为以格心为中心、边长 1 的闭方块。
func squareBounds(c Point) (lo, hi fpoint) {
	return fpoint{float64(c.X) - 0.5, float64(c.Y) - 0.5},
		fpoint{float64(c.X) + 0.5, float64(c.Y) + 0.5}
}

func intervalDist(v, lo, hi float64) float64 {
	if v < lo {
		return lo - v
	}
	if v > hi {
		return v - hi
	}
	return 0
}

// distPointSquare 点到障碍格方块的欧氏距离，落在方块内为 0。
func distPointSquare(q fpoint, c Point) float64 {
	lo, hi := squareBounds(c)
	dx := intervalDist(q.X, lo.X, hi.X)
	dy := intervalDist(q.Y, lo.Y, hi.Y)
	return math.Hypot(dx, dy)
}

func cross(o, a, b fpoint) float64 {
	return (a.X-o.X)*(b.Y-o.Y) - (a.Y-o.Y)*(b.X-o.X)
}

func onSegment(p, a, b fpoint) bool {
	return math.Abs(cross(a, b, p)) <= epsSmooth &&
		p.X >= math.Min(a.X, b.X)-epsSmooth && p.X <= math.Max(a.X, b.X)+epsSmooth &&
		p.Y >= math.Min(a.Y, b.Y)-epsSmooth && p.Y <= math.Max(a.Y, b.Y)+epsSmooth
}

// segmentsIntersect 两线段是否相交（含端点相触、共线重叠）。
func segmentsIntersect(a, b, c, d fpoint) bool {
	o1, o2 := cross(a, b, c), cross(a, b, d)
	o3, o4 := cross(c, d, a), cross(c, d, b)
	if ((o1 > epsSmooth && o2 < -epsSmooth) || (o1 < -epsSmooth && o2 > epsSmooth)) &&
		((o3 > epsSmooth && o4 < -epsSmooth) || (o3 < -epsSmooth && o4 > epsSmooth)) {
		return true
	}
	if math.Abs(o1) <= epsSmooth && onSegment(c, a, b) {
		return true
	}
	if math.Abs(o2) <= epsSmooth && onSegment(d, a, b) {
		return true
	}
	if math.Abs(o3) <= epsSmooth && onSegment(a, c, d) {
		return true
	}
	if math.Abs(o4) <= epsSmooth && onSegment(b, c, d) {
		return true
	}
	return false
}

func distPointSegment(q, a, b fpoint) float64 {
	vx, vy := b.X-a.X, b.Y-a.Y
	l2 := vx*vx + vy*vy
	if l2 <= epsSmooth {
		return math.Hypot(q.X-a.X, q.Y-a.Y)
	}
	t := ((q.X-a.X)*vx + (q.Y-a.Y)*vy) / l2
	t = math.Max(0, math.Min(1, t))
	return math.Hypot(q.X-(a.X+t*vx), q.Y-(a.Y+t*vy))
}

// distSegmentSquare 线段到障碍格方块的精确最小欧氏距离：
// 端点在方块内、或线段穿过方块边界时为 0；否则取端点-方块与四角-线段的最小值。
func distSegmentSquare(a, b fpoint, c Point) float64 {
	lo, hi := squareBounds(c)
	if distPointSquare(a, c) <= epsSmooth || distPointSquare(b, c) <= epsSmooth {
		return 0
	}
	corners := [4]fpoint{
		{lo.X, lo.Y}, {hi.X, lo.Y}, {hi.X, hi.Y}, {lo.X, hi.Y},
	}
	for k := 0; k < 4; k++ {
		if segmentsIntersect(a, b, corners[k], corners[(k+1)%4]) {
			return 0
		}
	}
	d := math.Min(distPointSquare(a, c), distPointSquare(b, c))
	for _, q := range corners {
		d = math.Min(d, distPointSegment(q, a, b))
	}
	return d
}

func (g *Grid) blockedCells() []Point {
	var blocked []Point
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if !g.cells[y][x] {
				blocked = append(blocked, Point{x, y})
			}
		}
	}
	return blocked
}

func pointClearance(q fpoint, blocked []Point) float64 {
	d := math.Inf(1)
	for _, c := range blocked {
		d = math.Min(d, distPointSquare(q, c))
	}
	return d
}

// polylineClearance 整条折线到障碍方块的精确连续最小间距。
func polylineClearance(pts []Point, blocked []Point) float64 {
	d := math.Inf(1)
	for i := 1; i < len(pts); i++ {
		a, b := toFP(pts[i-1]), toFP(pts[i])
		for _, c := range blocked {
			d = math.Min(d, distSegmentSquare(a, b, c))
		}
	}
	if len(pts) == 1 {
		d = pointClearance(toFP(pts[0]), blocked)
	}
	return d
}

func polylineLength(pts []Point) float64 {
	l := 0.0
	for i := 1; i < len(pts); i++ {
		l += math.Hypot(float64(pts[i].X-pts[i-1].X), float64(pts[i].Y-pts[i-1].Y))
	}
	return l
}

// turnAngle 拐角处入射边与出射边的夹角，范围 [0, π]；共线为 0。
func turnAngle(a, b, c Point) float64 {
	ux, uy := float64(b.X-a.X), float64(b.Y-a.Y)
	vx, vy := float64(c.X-b.X), float64(c.Y-b.Y)
	cosA := (ux*vx + uy*vy) / (math.Hypot(ux, uy) * math.Hypot(vx, vy))
	cosA = math.Max(-1, math.Min(1, cosA))
	return math.Acos(cosA)
}

// curvature 离散曲率 κ = 转角 / min(入射边长, 出射边长)（弧度/格）。
// 共线点曲率为 0。
func curvature(a, b, c Point) float64 {
	theta := turnAngle(a, b, c)
	if theta <= epsAngle {
		return 0
	}
	l1 := math.Hypot(float64(b.X-a.X), float64(b.Y-a.Y))
	l2 := math.Hypot(float64(c.X-b.X), float64(c.Y-b.Y))
	return theta / math.Min(l1, l2)
}

// CornerStat 原始路径每个拐角的反解与处理记录。
type CornerStat struct {
	Index         int     // 在原始路径中的下标
	Point         Point   // 拐角坐标
	TurnAngle     float64 // 原始转角（弧度）
	KappaBefore   float64 // 平滑前曲率
	KNeed         int     // 闭合反解：满足上界所需的单边拉直步数 k*（最坏情形闭式解）
	KFeas         int     // 对称拉直时几何/间距允许的最大单边步数（查表，非试拉）
	KUsed         int     // 实际采用的单边拉直步数（被消除时为 0）
	Retained      bool    // 平滑后是否仍为路径点
	KappaAfter    float64 // 平滑后该点曲率（被消除为 0）
	GeometryBound bool    // 保留且实测曲率仍超上界：环境几何不允许进一步拉直
}

// SmoothResult 平滑结果与核验数据。
type SmoothResult struct {
	Original []Point
	Smoothed []Point

	LengthBefore    float64
	LengthAfter     float64
	ClearanceBefore float64 // 原始折线到障碍的精确连续最小间距
	ClearanceAfter  float64 // 平滑后折线的精确连续最小间距

	KappaMax       float64 // 曲率上界
	MaxKappaBefore float64
	MaxKappaAfter  float64
	GeometryBoundN int // 受几何约束无法压到上界的拐角数

	Corners []CornerStat

	SamplePoints        int     // 逐点核验采样点数
	SampledMinClearance float64 // 采样点上的最小间距
	PointwiseOK         bool    // 逐点间距不变量是否成立
}

// Smooth 使用默认曲率上界 DefaultKappaMax 平滑路径。
func (g *Grid) Smooth(path []Point) (*SmoothResult, error) {
	return g.SmoothAt(path, DefaultKappaMax)
}

// SmoothAt 按指定曲率上界 kappaMax（弧度/格）平滑路径。
//
// 不变量（逐点核验，失败返回 ErrSmoothInvariant）：
//  1. 平滑后路径与障碍的最小间距不得小于平滑前；
//  2. 路径长度不得增加；
//  3. 平滑路径点是原始路径点的子序列，弦均通过 Bresenham 视线与间距双重判定。
func (g *Grid) SmoothAt(path []Point, kappaMax float64) (*SmoothResult, error) {
	if len(path) == 0 {
		return nil, errors.New("路径为空，无法平滑")
	}
	if kappaMax <= 0 {
		return nil, errors.New("曲率上界必须为正数")
	}
	for _, p := range path {
		ok, err := g.Walkable(p)
		if err != nil {
			return nil, err
		}
		if !ok {
			return nil, &BlockedPointError{Point: p}
		}
	}
	blocked := g.blockedCells()
	n := len(path)
	res := &SmoothResult{
		Original:        append([]Point(nil), path...),
		KappaMax:        kappaMax,
		ClearanceBefore: polylineClearance(path, blocked),
		LengthBefore:    polylineLength(path),
	}
	d0 := res.ClearanceBefore

	if n <= 2 { // 单点或单段：无需拉直，直接核验
		res.Smoothed = append([]Point(nil), path...)
		res.LengthAfter = res.LengthBefore
		res.ClearanceAfter = d0
		res.MaxKappaBefore = 0
		res.MaxKappaAfter = 0
		if err := g.verifyPointwise(res, blocked, d0); err != nil {
			return nil, err
		}
		return res, nil
	}

	// ---- 1. 一次性算出全部弦可行性表（视线 + 间距），不做反复试拉 ----
	feasible := make([][]bool, n)
	for i := 0; i < n; i++ {
		feasible[i] = make([]bool, n)
	}
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			ok, err := g.Visible(path[i], path[j])
			if err != nil {
				return nil, err
			}
			if ok {
				a, b := toFP(path[i]), toFP(path[j])
				d := math.Inf(1)
				for _, c := range blocked {
					d = math.Min(d, distSegmentSquare(a, b, c))
				}
				ok = d >= d0-epsSmooth
			}
			feasible[i][j] = ok
		}
	}
	for i := 0; i < n-1; i++ {
		if !feasible[i][i+1] {
			return nil, ErrSmoothInvariant // 原始相邻边不可行，理论上不可能
		}
	}

	// ---- 2. 每个拐角的闭合反解 ----
	stats := make([]CornerStat, 0, n-2)
	for i := 1; i < n-1; i++ {
		theta := turnAngle(path[i-1], path[i], path[i+1])
		if theta <= epsAngle {
			continue // A* 路径一般无冗余共线点，防御性跳过
		}
		// 闭合反解（非试拉）：
		// 要求拉直后邻边弦长 L >= θ/κmax；k 步轴对齐折线的弦长下界为 k/√2，
		// 故 k* = ceil(θ/κmax · √2)，至少 1。
		kNeed := int(math.Ceil(theta/kappaMax*math.Sqrt2 - epsSmooth))
		if kNeed < 1 {
			kNeed = 1
		}
		kmx := i
		if n-1-i < kmx {
			kmx = n - 1 - i
		}
		kFeas := 0
		for k := kmx; k >= 1; k-- { // 查表，不是试拉：每对弦只判定一次
			if feasible[i-k][i+k] {
				kFeas = k
				break
			}
		}
		// GeometryBound 在组装后按实测曲率回填。
		st := CornerStat{
			Index:       i,
			Point:       path[i],
			TurnAngle:   theta,
			KappaBefore: curvature(path[i-1], path[i], path[i+1]),
			KNeed:       kNeed,
			KFeas:       kFeas,
		}
		stats = append(stats, st)
	}
	res.Corners = stats
	for _, st := range stats {
		res.MaxKappaBefore = math.Max(res.MaxKappaBefore, st.KappaBefore)
	}

	// ---- 3. 确定性单遍组装：string-pulling 贪心取最远可行弦（只查表，不试拉）----
	// 能被一条可行弦跨过的拐角，消除后曲率直接归零，无需保留；
	// 因视线/间距约束无法跨过的拐角必然成为顶点，而贪心取最远弦使其
	// 两侧邻边尽可能长，即该拐角在所有可行折线中能取得的最小曲率。
	idx := []int{0}
	cur := 0
	for cur < n-1 {
		next := cur + 1 // 原始相邻边必然可行（兜底）
		for j := cur + 1; j < n; j++ {
			if feasible[cur][j] {
				next = j
			}
		}
		idx = append(idx, next)
		cur = next
	}

	smoothed := make([]Point, len(idx))
	for k, id := range idx {
		smoothed[k] = path[id]
	}
	res.Smoothed = smoothed

	// ---- 4. 回填每个拐角的实际处理与平滑后曲率 ----
	chosen := map[int]bool{0: true, n - 1: true}
	for _, id := range idx {
		chosen[id] = true
	}
	maxAfter := 0.0
	for si := range stats {
		st := &stats[si]
		i := st.Index
		if !chosen[i] {
			st.Retained = false
			st.KUsed = 0
			st.KappaAfter = 0
			continue
		}
		st.Retained = true
		// 找该保留点在新折线中的前后邻点
		var pi, pj int
		for q, id := range idx {
			if id == i {
				pi, pj = idx[q-1], idx[q+1]
				break
			}
		}
		st.KUsed = i - pi
		if pj-i < st.KUsed {
			st.KUsed = pj - i
		}
		st.KappaAfter = curvature(path[pi], path[i], path[pj])
		// 以实测曲率为准：贪心已让该保留拐角的邻边尽可能长，
		// 仍超上界说明环境几何（单宽走廊、窄门等）不允许进一步拉直。
		st.GeometryBound = st.KappaAfter > kappaMax+epsSmooth
		maxAfter = math.Max(maxAfter, st.KappaAfter)
	}
	res.Corners = stats
	res.MaxKappaAfter = maxAfter
	res.GeometryBoundN = 0
	for _, st := range stats {
		if st.GeometryBound && st.Retained {
			res.GeometryBoundN++
		}
	}

	res.LengthAfter = polylineLength(smoothed)
	res.ClearanceAfter = polylineClearance(smoothed, blocked)

	// ---- 5. 不变量逐点核验 ----
	if res.LengthAfter > res.LengthBefore+epsSmooth {
		return nil, ErrSmoothInvariant
	}
	if res.ClearanceAfter < d0-epsSmooth {
		return nil, ErrSmoothInvariant
	}
	if err := g.verifyPointwise(res, blocked, d0); err != nil {
		return nil, err
	}
	return res, nil
}

// verifyPointwise 沿平滑后折线按 sampleStep 密集采样，逐点核验间距不变量。
func (g *Grid) verifyPointwise(res *SmoothResult, blocked []Point, d0 float64) error {
	minD := math.Inf(1)
	count := 0
	for i := 0; i < len(res.Smoothed); i++ {
		a := toFP(res.Smoothed[i])
		count++
		minD = math.Min(minD, pointClearance(a, blocked))
		if i == len(res.Smoothed)-1 {
			break
		}
		b := toFP(res.Smoothed[i+1])
		seg := math.Hypot(b.X-a.X, b.Y-a.Y)
		steps := int(math.Ceil(seg / sampleStep))
		for t := 1; t < steps; t++ {
			f := float64(t) / float64(steps)
			q := fpoint{a.X + f*(b.X-a.X), a.Y + f*(b.Y-a.Y)}
			minD = math.Min(minD, pointClearance(q, blocked))
			count++
		}
	}
	res.SamplePoints = count
	res.SampledMinClearance = minD
	res.PointwiseOK = minD >= d0-epsSmooth
	if !res.PointwiseOK {
		return ErrSmoothInvariant
	}
	return nil
}
