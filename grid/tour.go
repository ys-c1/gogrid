package grid

import (
	"errors"
	"fmt"
	"math"
	"strings"
)

var (
	ErrNoStops        = errors.New("点集为空")
	ErrDuplicateStop  = errors.New("点集含重复目标")
	ErrUnwalkableStop = errors.New("目标不可走")
	ErrTourInvariant  = errors.New("路径不变量被破坏")
	ErrBadOptions     = errors.New("路径参数非法")
	ErrTooManyStops   = errors.New("点集过大，无法暴力枚举")
)

const (
	// DefaultTourPasses 是未显式给出改进轮数时使用的扫描轮数上限。
	DefaultTourPasses = 100
	// BruteForceLimit 是暴力枚举全排列允许的最大点数（n! 增长很快）。
	BruteForceLimit = 9
)

// AcceptRule 是二次交换的接受准则。
type AcceptRule int

const (
	// AcceptStrict 只接受严格变短的交换。
	AcceptStrict AcceptRule = iota
	// AcceptNonWorsening 也接受等长交换；总长度不增，但交换次数会更多。
	AcceptNonWorsening
)

func (a AcceptRule) String() string {
	switch a {
	case AcceptStrict:
		return "strict"
	case AcceptNonWorsening:
		return "non_worsening"
	default:
		return fmt.Sprintf("accept_rule(%d)", int(a))
	}
}

// MarshalText 让归档里的接受准则保持可读的字符串形式。
func (a AcceptRule) MarshalText() ([]byte, error) {
	return []byte(a.String()), nil
}

// UnmarshalText 与 MarshalText 配对，使归档可原样读回。
func (a *AcceptRule) UnmarshalText(text []byte) error {
	switch string(text) {
	case AcceptStrict.String():
		*a = AcceptStrict
	case AcceptNonWorsening.String():
		*a = AcceptNonWorsening
	default:
		return fmt.Errorf("%w: 未知接受准则 %q", ErrBadOptions, text)
	}
	return nil
}

// TourOptions 配置二次交换改进阶段。
//
// MaxPasses 为扫描轮数上限：0 表示使用 DefaultTourPasses，负数非法。
// MaxSwaps 为接受的交换次数上限：0 表示只受 MaxPasses 约束，负数非法。
// Exact 为 true 时同时暴力枚举全排列，给出最优解与差距。
type TourOptions struct {
	MaxPasses int
	MaxSwaps  int
	Accept    AcceptRule
	Exact     bool
}

func (o TourOptions) validate() error {
	if o.MaxPasses < 0 {
		return fmt.Errorf("%w: MaxPasses=%d", ErrBadOptions, o.MaxPasses)
	}
	if o.MaxSwaps < 0 {
		return fmt.Errorf("%w: MaxSwaps=%d", ErrBadOptions, o.MaxSwaps)
	}
	if o.Accept != AcceptStrict && o.Accept != AcceptNonWorsening {
		return fmt.Errorf("%w: Accept=%s", ErrBadOptions, o.Accept)
	}
	return nil
}

// StopProblem 描述点集里的一个具体问题，并带上出错的坐标。
type StopProblem struct {
	Kind  error
	At    Point
	Cause error // 可选，例如 ErrOOB
}

func (p StopProblem) Error() string {
	if p.Kind == ErrNoStops {
		return p.Kind.Error()
	}
	msg := fmt.Sprintf("%v: 坐标(%d,%d)", p.Kind, p.At.X, p.At.Y)
	if p.Cause != nil {
		msg += "（" + p.Cause.Error() + "）"
	}
	return msg
}

// StopSetError 汇总点集的全部问题；可用 errors.As 取出逐条坐标。
type StopSetError struct {
	Problems []StopProblem
}

func (e *StopSetError) Error() string {
	parts := make([]string, len(e.Problems))
	for i, p := range e.Problems {
		parts[i] = p.Error()
	}
	return strings.Join(parts, "; ")
}

func (e *StopSetError) Unwrap() []error {
	errs := make([]error, 0, len(e.Problems)*2)
	for _, p := range e.Problems {
		errs = append(errs, p.Kind)
		if p.Cause != nil {
			errs = append(errs, p.Cause)
		}
	}
	return errs
}

// SegmentError 报出两个目标之间不可达，并带上两端坐标。
type SegmentError struct {
	From, To Point
	Cause    error
}

func (e *SegmentError) Error() string {
	return fmt.Sprintf("%v: (%d,%d) -> (%d,%d)",
		e.Cause, e.From.X, e.From.Y, e.To.X, e.To.Y)
}

func (e *SegmentError) Unwrap() error { return e.Cause }

// TourResult 保存初解、改进解，以及可选的暴力枚举最优解。
// 各字段带 json 标签，可直接归档；用 Grid.VerifyTour 可按归档顺序复算长度。
type TourResult struct {
	Stops           []Point    `json:"stops"`
	Accept          AcceptRule `json:"accept_rule"`
	InitialOrder    []Point    `json:"initial_order"`
	InitialLength   int        `json:"initial_length"`
	ImprovedOrder   []Point    `json:"improved_order"`
	ImprovedLength  int        `json:"improved_length"`
	Passes          int        `json:"passes"`
	Swaps           int        `json:"swaps"`
	Exact           bool       `json:"exact"`
	OptimalOrder    []Point    `json:"optimal_order,omitempty"`
	OptimalLength   int        `json:"optimal_length"` // -1 表示未计算
	Gap             int        `json:"gap"`            // -1 表示未计算
	GapRatio        float64    `json:"gap_ratio"`      // -1 表示未计算
	LowerBound      int        `json:"lower_bound"`    // 最优解的严格下界（最小生成树）
	GapUpperBound   int        `json:"gap_upper_bound"`
	RatioUpperBound float64    `json:"ratio_upper_bound"`
}

func (g *Grid) validateStops(stops []Point) error {
	if len(stops) == 0 {
		return &StopSetError{Problems: []StopProblem{{Kind: ErrNoStops}}}
	}
	var problems []StopProblem
	seen := make(map[Point]bool, len(stops))
	for _, p := range stops {
		ok, err := g.Walkable(p)
		if err != nil {
			problems = append(problems, StopProblem{Kind: ErrUnwalkableStop, At: p, Cause: err})
			continue
		}
		if !ok {
			problems = append(problems, StopProblem{Kind: ErrUnwalkableStop, At: p})
		}
		if seen[p] {
			problems = append(problems, StopProblem{Kind: ErrDuplicateStop, At: p})
			continue
		}
		seen[p] = true
	}
	if len(problems) > 0 {
		return &StopSetError{Problems: problems}
	}
	return nil
}

// tourDistances 预计算点集两两之间的最短路长度。
func (g *Grid) tourDistances(stops []Point) ([][]int, error) {
	n := len(stops)
	d := make([][]int, n)
	for i := range d {
		d[i] = make([]int, n)
	}
	for i := 0; i < n; i++ {
		for j := i + 1; j < n; j++ {
			path, _, err := g.Path(stops[i], stops[j])
			if err != nil {
				return nil, &SegmentError{From: stops[i], To: stops[j], Cause: err}
			}
			d[i][j] = len(path) - 1
			d[j][i] = d[i][j]
		}
	}
	return d, nil
}

// Tour 按“最近邻初解 + 二次交换改进”规划多目标访问顺序。
// 单目标 Path 的行为与签名保持不变。
func (g *Grid) Tour(stops []Point, opts TourOptions) (*TourResult, error) {
	if err := opts.validate(); err != nil {
		return nil, err
	}
	if err := g.validateStops(stops); err != nil {
		return nil, err
	}
	if opts.Exact && len(stops) > BruteForceLimit {
		return nil, fmt.Errorf("%w: %d 个点超过上限 %d", ErrTooManyStops, len(stops), BruteForceLimit)
	}
	d, err := g.tourDistances(stops)
	if err != nil {
		return nil, err
	}

	start := nearestNeighborOrder(d)
	initial := pointsAt(stops, start)
	initialLen, err := verifyOrderWithDist(stops, initial, d)
	if err != nil {
		return nil, err
	}
	if initialLen != orderLength(start, d) {
		return nil, fmt.Errorf("%w: 初解长度复算不一致", ErrTourInvariant)
	}

	work := append([]int(nil), start...)
	work, passes, swaps := improveOrder(d, work, opts)
	improved := pointsAt(stops, work)
	improvedLen, err := verifyOrderWithDist(stops, improved, d)
	if err != nil {
		return nil, err
	}
	if improvedLen != orderLength(work, d) {
		return nil, fmt.Errorf("%w: 改进解长度复算不一致", ErrTourInvariant)
	}
	if improvedLen > initialLen {
		return nil, fmt.Errorf("%w: 改进后长度 %d 超过初解 %d", ErrTourInvariant, improvedLen, initialLen)
	}

	res := &TourResult{
		Stops:          append([]Point(nil), stops...),
		Accept:         opts.Accept,
		InitialOrder:   initial,
		InitialLength:  initialLen,
		ImprovedOrder:  improved,
		ImprovedLength: improvedLen,
		Passes:         passes,
		Swaps:          swaps,
		Exact:          opts.Exact,
		OptimalLength:  -1,
		Gap:            -1,
		GapRatio:       -1,
	}

	// 最小生成树是任何通路的严格下界，故“改进解 - MST”必然覆盖真实差距。
	res.LowerBound = LowerBound(d)
	res.GapUpperBound = improvedLen - res.LowerBound
	if res.LowerBound > 0 {
		res.RatioUpperBound = float64(improvedLen) / float64(res.LowerBound)
	} else {
		res.RatioUpperBound = 1
	}
	if res.GapUpperBound < 0 {
		return nil, fmt.Errorf("%w: 下界 %d 超过改进解 %d", ErrTourInvariant, res.LowerBound, improvedLen)
	}

	if opts.Exact {
		optimal, optLen, err := optimalWithDist(d)
		if err != nil {
			return nil, err
		}
		if improvedLen < optLen {
			return nil, fmt.Errorf("%w: 改进解 %d 短于暴力最优 %d", ErrTourInvariant, improvedLen, optLen)
		}
		if res.LowerBound > optLen {
			return nil, fmt.Errorf("%w: 下界 %d 超过暴力最优 %d", ErrTourInvariant, res.LowerBound, optLen)
		}
		res.OptimalOrder = pointsAt(stops, optimal)
		res.OptimalLength = optLen
		res.Gap = improvedLen - optLen
		if res.Gap > res.GapUpperBound {
			return nil, fmt.Errorf("%w: 真实差距 %d 超过上界 %d", ErrTourInvariant, res.Gap, res.GapUpperBound)
		}
		if optLen == 0 {
			res.GapRatio = 1
		} else {
			res.GapRatio = float64(improvedLen) / float64(optLen)
		}
	}
	return res, nil
}

// VerifyTour 校验 order 是 stops 的一个排列，并返回按该顺序访问的总长度。
// 归档只需保存网格、点集与顺序，即可用本方法复算。
func (g *Grid) VerifyTour(stops, order []Point) (int, error) {
	if err := g.validateStops(stops); err != nil {
		return 0, err
	}
	d, err := g.tourDistances(stops)
	if err != nil {
		return 0, err
	}
	return verifyOrderWithDist(stops, order, d)
}

// OptimalTour 暴力枚举全排列，返回最优访问顺序与长度。
func (g *Grid) OptimalTour(stops []Point) ([]Point, int, error) {
	if err := g.validateStops(stops); err != nil {
		return nil, 0, err
	}
	if len(stops) > BruteForceLimit {
		return nil, 0, fmt.Errorf("%w: %d 个点超过上限 %d", ErrTooManyStops, len(stops), BruteForceLimit)
	}
	d, err := g.tourDistances(stops)
	if err != nil {
		return nil, 0, err
	}
	order, length, err := optimalWithDist(d)
	if err != nil {
		return nil, 0, err
	}
	return pointsAt(stops, order), length, nil
}

// LowerBound 返回访问顺序总长度的严格下界，即点集上的最小生成树长度。
//
// 任何一条经过全部目标的通路都是一个生成树（n 个点、n-1 条边、连通无环），
// 因此其长度不小于最小生成树长度；距离取自最短路，满足三角不等式，
// 故该下界对最优解同样成立。代价 O(n²)。
func LowerBound(d [][]int) int {
	n := len(d)
	if n <= 1 {
		return 0
	}
	inTree := make([]bool, n)
	best := make([]int, n)
	for i := range best {
		best[i] = math.MaxInt
	}
	best[0] = 0
	total := 0
	for {
		u, uBest := -1, math.MaxInt
		for v := 0; v < n; v++ {
			if !inTree[v] && best[v] < uBest {
				u, uBest = v, best[v]
			}
		}
		if u < 0 {
			break
		}
		inTree[u] = true
		total += best[u]
		for v := 0; v < n; v++ {
			if !inTree[v] && d[u][v] < best[v] {
				best[v] = d[u][v]
			}
		}
	}
	return total
}

// --- helpers ---
