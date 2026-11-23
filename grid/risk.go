package grid

import (
	"errors"
	"fmt"
	"math"
	"sort"
	"strings"
)

// NoCell 是全局性输入错误（不对应单个格子）使用的坐标哨兵。
var NoCell = Point{-1, -1}

var (
	ErrRiskEmptyGrid   = errors.New("风险场网格为空")
	ErrRiskNegative    = errors.New("风险值为负")
	ErrRiskNonFinite   = errors.New("风险值非有限")
	ErrRiskZeroLevels  = errors.New("目标层数为零")
	ErrRiskLayerBounds = errors.New("层容量区间不可行")
)

// InputError 携带越界输入所在的坐标；At == NoCell 表示该错误是全局性的。
type InputError struct {
	Err    error
	At     Point
	Detail string
}

func (e *InputError) Error() string {
	var b strings.Builder
	b.WriteString(e.Err.Error())
	if e.Detail != "" {
		b.WriteString("（")
		b.WriteString(e.Detail)
		b.WriteString("）")
	}
	if e.At != NoCell {
		fmt.Fprintf(&b, "（坐标 %d,%d）", e.At.X, e.At.Y)
	} else {
		b.WriteString("（全局输入，无对应格子）")
	}
	return b.String()
}

func (e *InputError) Unwrap() error { return e.Err }

// RiskField 是 W×H 的非负风险场，行优先存储。
type RiskField struct {
	W, H int
	risk []float64
}

// NewRiskField 由二维风险值构造风险场。
// 空网格、负风险、非有限风险逐项拒绝；负风险与非有限风险带回具体坐标。
func NewRiskField(rows [][]float64) (*RiskField, error) {
	if len(rows) == 0 || len(rows[0]) == 0 {
		return nil, &InputError{Err: ErrRiskEmptyGrid, At: NoCell}
	}
	w, h := len(rows[0]), len(rows)
	risk := make([]float64, 0, w*h)
	for y, row := range rows {
		if len(row) != w {
			return nil, &InputError{
				Err:    ErrShape,
				At:     Point{0, y},
				Detail: fmt.Sprintf("第 %d 行宽度 %d，应为 %d", y, len(row), w),
			}
		}
		for x, v := range row {
			if math.IsNaN(v) || math.IsInf(v, 0) {
				return nil, &InputError{Err: ErrRiskNonFinite, At: Point{x, y}}
			}
			if v < 0 {
				return nil, &InputError{Err: ErrRiskNegative, At: Point{x, y}}
			}
			risk = append(risk, v)
		}
	}
	return &RiskField{W: w, H: h, risk: risk}, nil
}

// At 返回坐标处风险值。
func (f *RiskField) At(p Point) (float64, error) {
	if p.X < 0 || p.Y < 0 || p.X >= f.W || p.Y >= f.H {
		return 0, ErrOOB
	}
	return f.risk[p.Y*f.W+p.X], nil
}

// Cells 返回总格子数。
func (f *RiskField) Cells() int { return f.W * f.H }

// LayerOptions 描述分层要求。
type LayerOptions struct {
	Levels      int // 目标层数，必须 > 0
	MinPerLayer int // 每层最少格子数（可为 0）
	MaxPerLayer int // 每层最多格子数；0 表示不设上限（即格子总数）
}

func (o LayerOptions) resolve(n int) (min, max int, err error) {
	if o.Levels <= 0 {
		return 0, 0, &InputError{Err: ErrRiskZeroLevels, At: NoCell}
	}
	if o.MinPerLayer < 0 || o.MaxPerLayer < 0 ||
		(o.MaxPerLayer > 0 && o.MaxPerLayer < o.MinPerLayer) {
		return 0, 0, &InputError{
			Err:    ErrRiskLayerBounds,
			At:     NoCell,
			Detail: fmt.Sprintf("层数 %d，区间 [%d,%d]", o.Levels, o.MinPerLayer, o.MaxPerLayer),
		}
	}
	min, max = o.MinPerLayer, o.MaxPerLayer
	if max == 0 {
		max = n
	}
	if o.Levels*min > n || o.Levels*max < n {
		return 0, 0, &InputError{
			Err: ErrRiskLayerBounds,
			At:  NoCell,
			Detail: fmt.Sprintf("层数 %d，区间 [%d,%d]，格子数 %d 无法均分",
				o.Levels, min, max, n),
		}
	}
	return min, max, nil
}

// RiskLayer 是一个风险层。Lo/Hi 为该层实际覆盖的风险闭区间（空层取边界风险值）。
type RiskLayer struct {
	Index int
	Lo    float64
	Hi    float64
	Count int
}

// QuantileCut 是第 Boundary 层与下一层之间的切点。
// Target 是由目标层数反解出的分位 Boundary/Levels；Actual 是实际分位（累计格数/总格数）。
type QuantileCut struct {
	Boundary int
	Target   float64
	Actual   float64
	Delta    float64
	Bound    float64 // 由目标层数反解的 |Actual-Target| 上界 1/n
	Tie      bool    // 该切点两侧风险值相等（切开了同值块）
}

// LayerSet 是一次分层的结果。
type LayerSet struct {
	field      *RiskField
	opts       LayerOptions
	Levels     int
	Layers     []RiskLayer
	Thresholds []float64 // 长度 Levels-1，Thresholds[i] 是第 i+1 层的上沿
	Cuts       []QuantileCut
	MaxDelta   float64
	DeltaBound float64
	assign     []int // 行优先，1 起始层号
}

// Layering 按各格风险值的经验分位切层。
//
// 排序规则为 (风险, Y, X)：风险严格升序分配，平局按格子位置决定归属。
// 因此同值格子可能被相邻层切开，但层号随风险单调不降。
// 每层格数由容量区间 [MinPerLayer, MaxPerLayer] 约束，分配采用最均衡方案。
func (f *RiskField) Layering(opts LayerOptions) (*LayerSet, error) {
	n := f.Cells()
	min, max, err := opts.resolve(n)
	if err != nil {
		return nil, err
	}
	L := opts.Levels

	idx := make([]int, n)
	for i := range idx {
		idx[i] = i
	}
	sort.Slice(idx, func(a, b int) bool {
		ia, ib := idx[a], idx[b]
		if f.risk[ia] != f.risk[ib] {
			return f.risk[ia] < f.risk[ib]
		}
		if ia/f.W != ib/f.W {
			return ia/f.W < ib/f.W
		}
		return ia%f.W < ib%f.W
	})

	// 逐层分配计数：每层取累计位 ⌈i·n/L⌉ 与已分配数之差（向上取整），
	// 再夹在剩余格子可实现的区间内，保证每层落在容量区间、且末尾恰好分完。
	counts := make([]int, L)
	cum, remaining := 0, n
	for i := 1; i <= L; i++ {
		left := L - i
		lo := min
		if v := remaining - left*max; v > lo {
			lo = v
		}
		hi := max
		if v := remaining - left*min; v < hi {
			hi = v
		}
		nIdeal := ceilDiv(i*n, L) - cum
		if nIdeal < lo {
			nIdeal = lo
		}
		if nIdeal > hi {
			nIdeal = hi
		}
		counts[i-1] = nIdeal
		cum += nIdeal
		remaining -= nIdeal
	}

	set := &LayerSet{
		field:  f,
		opts:   opts,
		Levels: L,
		Layers: make([]RiskLayer, 0, L),
		assign: make([]int, n),
	}
	pos := 0
	for i, c := range counts {
		layer := RiskLayer{Index: i + 1, Count: c}
		if c > 0 {
			layer.Lo = f.risk[idx[pos]]
			layer.Hi = f.risk[idx[pos+c-1]]
			for k := pos; k < pos+c; k++ {
				set.assign[idx[k]] = i + 1
			}
		} else {
			// 空层没有格子，用切点处风险值占位，保持 Lo/Hi 单调。
			at := pos
			if at >= n {
				at = n - 1
			}
			layer.Lo = f.risk[idx[at]]
			layer.Hi = f.risk[idx[at]]
		}
		set.Layers = append(set.Layers, layer)
		pos += c
	}

	for i := 1; i < L; i++ {
		cut := QuantileCut{
			Boundary: i,
			Target:   float64(i) / float64(L),
			Actual:   float64(cumOf(counts, i)) / float64(n),
		}
		cut.Delta = math.Abs(cut.Actual - cut.Target)
		// 由目标层数 L 反解：第 i 个切点的累计格数取 ⌈i·n/L⌉（受容量夹取后不变，
		// 见上方分配），与理想位之差 < 1，故 |实际分位 - 目标分位| < 1/n。
		cut.Bound = 1.0 / float64(n)
		cut.Tie = set.Layers[i-1].Hi == set.Layers[i].Lo
		set.Cuts = append(set.Cuts, cut)
		set.Thresholds = append(set.Thresholds, set.Layers[i-1].Hi)
		if cut.Delta > set.MaxDelta {
			set.MaxDelta = cut.Delta
		}
		if cut.Bound > set.DeltaBound {
			set.DeltaBound = cut.Bound
		}
	}
	return set, nil
}

func cumOf(counts []int, upto int) int {
	s := 0
	for i := 0; i < upto; i++ {
		s += counts[i]
	}
	return s
}

func ceilDiv(a, b int) int { return (a + b - 1) / b }

// LayerAt 返回坐标所在层号（1 起始）。
func (s *LayerSet) LayerAt(p Point) (int, error) {
	if p.X < 0 || p.Y < 0 || p.X >= s.field.W || p.Y >= s.field.H {
		return 0, ErrOOB
	}
	return s.assign[p.Y*s.field.W+p.X], nil
}

// Counts 返回各层格子数（按层号顺序）。
func (s *LayerSet) Counts() []int {
	out := make([]int, len(s.Layers))
	for i, l := range s.Layers {
		out[i] = l.Count
	}
	return out
}

// CheckCounts 核验：每层格数落在给定区间，且各层之和等于总格数。
func (s *LayerSet) CheckCounts() error {
	n := s.field.Cells()
	min, max := s.opts.MinPerLayer, s.opts.MaxPerLayer
	if max == 0 {
		max = n
	}
	var errs []error
	if len(s.Layers) != s.Levels {
		errs = append(errs, fmt.Errorf("层数 %d，实际 %d", s.Levels, len(s.Layers)))
	}
	sum := 0
	for _, l := range s.Layers {
		sum += l.Count
		if l.Count < min || l.Count > max {
			errs = append(errs, fmt.Errorf("第 %d 层格子数 %d 不在 [%d,%d]", l.Index, l.Count, min, max))
		}
	}
	if sum != n {
		errs = append(errs, fmt.Errorf("各层计数之和 %d，应为 %d", sum, n))
	}
	return errors.Join(errs...)
}

// CheckMonotone 逐对核验：风险严格更小的格子，层号不得更大。
// 复杂度 O(N²)，用于校验而非生产路径。
func (s *LayerSet) CheckMonotone() error {
	f := s.field
	n := f.Cells()
	const maxReport = 5
	var errs []error
	for a := 0; a < n; a++ {
		for b := a + 1; b < n; b++ {
			ra, rb := f.risk[a], f.risk[b]
			low, high := a, b
			if rb < ra {
				low, high = b, a
			} else if ra == rb {
				continue
			}
			if s.assign[low] <= s.assign[high] {
				continue
			}
			pl := Point{low % f.W, low / f.W}
			ph := Point{high % f.W, high / f.W}
			errs = append(errs, fmt.Errorf(
				"层号与风险不单调：风险 %g@(%d,%d) 在层 %d，风险 %g@(%d,%d) 在层 %d",
				f.risk[low], pl.X, pl.Y, s.assign[low],
				f.risk[high], ph.X, ph.Y, s.assign[high]))
			if len(errs) >= maxReport {
				return errors.Join(errs...)
			}
		}
	}
	return errors.Join(errs...)
}

// CheckQuantiles 核验：每个切点的实际分位偏差不超过给出的上界。
func (s *LayerSet) CheckQuantiles() error {
	const eps = 1e-12
	var errs []error
	for _, c := range s.Cuts {
		if c.Delta > c.Bound+eps {
			errs = append(errs, fmt.Errorf(
				"第 %d 个切点分位偏差 %.12g 超过上界 %.12g", c.Boundary, c.Delta, c.Bound))
		}
	}
	if s.MaxDelta > s.DeltaBound+eps {
		errs = append(errs, fmt.Errorf(
			"最大分位偏差 %.12g 超过上界 %.12g", s.MaxDelta, s.DeltaBound))
	}
	return errors.Join(errs...)
}

// Verify 依次执行计数、单调、分位三项核验。
func (s *LayerSet) Verify() error {
	return errors.Join(s.CheckCounts(), s.CheckMonotone(), s.CheckQuantiles())
}

// CutsTable 返回切点的分位表。
func (s *LayerSet) CutsTable() string {
	var b strings.Builder
	fmt.Fprintf(&b, "%-4s %-10s %-10s %-10s %-10s %-10s %-4s\n",
		"切点", "阈值", "目标分位", "实际分位", "偏差", "偏差上界", "平局")
	for _, c := range s.Cuts {
		tie := ""
		if c.Tie {
			tie = "是"
		}
		fmt.Fprintf(&b, "%-4d %-10.6g %-10.4f %-10.4f %-10.4g %-10.4g %-4s\n",
			c.Boundary, s.Thresholds[c.Boundary-1], c.Target, c.Actual,
			c.Delta, c.Bound, tie)
	}
	return b.String()
}

// Table 返回逐层阈值与计数表。
func (s *LayerSet) Table() string {
	var b strings.Builder
	fmt.Fprintf(&b, "层数=%d 格子数=%d 最大分位偏差=%.6g 偏差上界=%.6g\n",
		s.Levels, s.field.Cells(), s.MaxDelta, s.DeltaBound)
	fmt.Fprintf(&b, "%-3s %-10s %-10s %-10s %-5s %-10s %-10s\n",
		"层", "阈值", "下界", "上界", "计数", "实际分位", "目标分位")
	for _, l := range s.Layers {
		thr, actual, target := "—", "—", "—"
		if l.Index < s.Levels {
			c := s.Cuts[l.Index-1]
			thr = fmt.Sprintf("%.6g", s.Thresholds[l.Index-1])
			actual = fmt.Sprintf("%.4f", c.Actual)
			target = fmt.Sprintf("%.4f", c.Target)
		}
		fmt.Fprintf(&b, "%-3d %-10s %-10s %-10s %-5d %-10s %-10s\n",
			l.Index, thr, fmt.Sprintf("%.6g", l.Lo), fmt.Sprintf("%.6g", l.Hi),
			l.Count, actual, target)
	}
	return b.String()
}
