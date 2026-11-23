package grid

import (
	"errors"
	"math"
	"testing"
)

func mustField(t *testing.T, rows [][]float64) *RiskField {
	t.Helper()
	f, err := NewRiskField(rows)
	if err != nil {
		t.Fatalf("构造风险场失败: %v", err)
	}
	return f
}

// 逐格核验层号确实等于 Layering 的结果，并跑完三项核验。
func verifyAll(t *testing.T, f *RiskField, s *LayerSet) {
	t.Helper()
	if err := s.Verify(); err != nil {
		t.Fatalf("核验失败: %v\n%s", err, s.Table())
	}
	if got, want := len(s.Thresholds), s.Levels-1; got != want {
		t.Fatalf("阈值个数 %d，应为 %d", got, want)
	}
	// 层号随风险单调：按风险升序扫描，层号不得回退。
	type cell struct {
		risk float64
		lay  int
	}
	var cells []cell
	for y := 0; y < f.H; y++ {
		for x := 0; x < f.W; x++ {
			lay, err := s.LayerAt(Point{x, y})
			if err != nil {
				t.Fatalf("LayerAt(%d,%d): %v", x, y, err)
			}
			if lay < 1 || lay > s.Levels {
				t.Fatalf("(%d,%d) 层号 %d 越界", x, y, lay)
			}
			v, _ := f.At(Point{x, y})
			cells = append(cells, cell{v, lay})
		}
	}
	for i := 0; i < len(cells); i++ {
		for j := 0; j < len(cells); j++ {
			if cells[i].risk < cells[j].risk && cells[i].lay > cells[j].lay {
				t.Fatalf("单调性破坏: 风险 %g 在层 %d，风险 %g 在层 %d",
					cells[i].risk, cells[i].lay, cells[j].risk, cells[j].lay)
			}
		}
	}
	// 阈值就是相邻层的分界值。
	for i, th := range s.Thresholds {
		if th != s.Layers[i].Hi {
			t.Fatalf("阈值 %d = %g，应为 %g", i, th, s.Layers[i].Hi)
		}
	}
}

func TestRiskRejectsBadInput(t *testing.T) {
	_, err := NewRiskField(nil)
	var ie *InputError
	if !errors.As(err, &ie) || !errors.Is(err, ErrRiskEmptyGrid) {
		t.Fatalf("空网格应拒绝，得到 %v", err)
	}
	if ie.At != NoCell {
		t.Fatalf("空网格坐标应为哨兵，得到 %v", ie.At)
	}

	if _, err := NewRiskField([][]float64{{}}); !errors.Is(err, ErrRiskEmptyGrid) {
		t.Fatalf("零宽网格应拒绝，得到 %v", err)
	}

	f := mustField(t, [][]float64{{0, 1, 2}, {3, 4, 5}})
	_, err = NewRiskField([][]float64{{1, 2}, {3, -0.5}})
	if !errors.Is(err, ErrRiskNegative) || !errors.As(err, &ie) || ie.At != (Point{1, 1}) {
		t.Fatalf("负风险应带坐标拒绝，得到 %v", err)
	}
	if _, err := NewRiskField([][]float64{{1, math.NaN()}}); !errors.Is(err, ErrRiskNonFinite) {
		t.Fatalf("NaN 应拒绝，得到 %v", err)
	}
	if _, err := NewRiskField([][]float64{{1, math.Inf(1)}}); !errors.Is(err, ErrRiskNonFinite) {
		t.Fatalf("Inf 应拒绝，得到 %v", err)
	}

	_, err = f.Layering(LayerOptions{Levels: 0})
	if !errors.Is(err, ErrRiskZeroLevels) || !errors.As(err, &ie) || ie.At != NoCell {
		t.Fatalf("零层应拒绝，得到 %v", err)
	}
	_, err = f.Layering(LayerOptions{Levels: 3, MinPerLayer: 3})
	if !errors.Is(err, ErrRiskLayerBounds) {
		t.Fatalf("不可行区间应拒绝，得到 %v", err)
	}
}

func TestRiskUniform(t *testing.T) {
	const w, h = 6, 5
	rows := make([][]float64, h)
	for y := range rows {
		rows[y] = make([]float64, w)
		for x := range rows[y] {
			rows[y][x] = float64(y*w + x)
		}
	}
	f := mustField(t, rows)
	s, err := f.Layering(LayerOptions{Levels: 4, MinPerLayer: 5, MaxPerLayer: 9})
	if err != nil {
		t.Fatal(err)
	}
	verifyAll(t, f, s)
	if got := s.Counts(); got[0] < 5 || got[1] < 5 || got[2] < 5 || got[3] < 5 {
		t.Fatalf("均匀分布的层计数异常: %v", got)
	}
	// 均匀线性场：分位与目标几乎重合。
	if s.MaxDelta > 0.05 {
		t.Fatalf("均匀场分位偏差过大: %.6g", s.MaxDelta)
	}
	t.Logf("\n%s\n%s", s.Table(), s.CutsTable())
}

func TestRiskBimodal(t *testing.T) {
	// 双峰：30% 低风险 + 70% 高风险，中间几乎无值。
	rows := [][]float64{
		{0.1, 0.2, 0.1, 0.3},
		{0.2, 0.1, 0.2, 0.1},
		{9.0, 9.1, 8.9, 9.2},
		{9.3, 8.8, 9.1, 9.0},
		{9.2, 9.0, 9.3, 9.1},
	}
	f := mustField(t, rows)
	s, err := f.Layering(LayerOptions{Levels: 3, MinPerLayer: 3})
	if err != nil {
		t.Fatal(err)
	}
	verifyAll(t, f, s)
	// 双峰之间没有数据，中间层必然很薄或为空；容量下限 3 约束仍须满足。
	for _, l := range s.Layers {
		if l.Count < 3 {
			t.Fatalf("双峰场景第 %d 层计数 %d < 3", l.Index, l.Count)
		}
	}
	t.Logf("\n%s\n%s", s.Table(), s.CutsTable())
}

func TestRiskSkewed(t *testing.T) {
	// 极端偏斜：99 个 0 风险格子 + 1 个极大风险格子。
	const n = 100
	rows := make([][]float64, 10)
	for y := range rows {
		rows[y] = make([]float64, 10)
	}
	rows[9][9] = 1e6
	f := mustField(t, rows)
	s, err := f.Layering(LayerOptions{Levels: 4, MinPerLayer: 1})
	if err != nil {
		t.Fatal(err)
	}
	verifyAll(t, f, s)
	if got := s.Counts(); got[0] != 25 || got[1] != 25 || got[2] != 25 || got[3] != 25 {
		t.Fatalf("偏斜场景按分位应均分为 25×4，得到 %v", got)
	}
	// 分位切不散零值块：三个阈值全部塌到 0。
	for i, th := range s.Thresholds {
		if th != 0 {
			t.Fatalf("阈值 %d 应为 0，得到 %g", i, th)
		}
	}
	last := s.Layers[len(s.Layers)-1]
	if last.Lo != 0 || last.Hi != 1e6 {
		t.Fatalf("最高层应覆盖 [0,1e6]（零值块被切开），得到 [%g,%g]", last.Lo, last.Hi)
	}
	for _, c := range s.Cuts {
		if !c.Tie {
			t.Fatalf("切点 %d 应报告平局（切开同值块）", c.Boundary)
		}
		if math.Abs(c.Actual-c.Target) > c.Bound+1e-12 {
			t.Fatalf("切点 %d 分位偏差 %.6g 超过上界 %.6g", c.Boundary,
				math.Abs(c.Actual-c.Target), c.Bound)
		}
	}
	t.Logf("\n%s\n%s", s.Table(), s.CutsTable())
}

func TestRiskSingleCell(t *testing.T) {
	f := mustField(t, [][]float64{{7}})
	for _, levels := range []int{1, 4} {
		s, err := f.Layering(LayerOptions{Levels: levels, MinPerLayer: 0})
		if err != nil {
			t.Fatalf("单格 %d 层: %v", levels, err)
		}
		verifyAll(t, f, s)
		if lay, _ := s.LayerAt(Point{0, 0}); lay != 1 {
			t.Fatalf("单格应落在第 1 层，得到 %d", lay)
		}
		if sum := len(s.Counts()); sum != levels {
			t.Fatalf("层数 %d，得到 %d", levels, sum)
		}
		t.Logf("层数=%d\n%s", levels, s.Table())
	}
	// 容量下限 1、层数 2 对单格不可行。
	if _, err := f.Layering(LayerOptions{Levels: 2, MinPerLayer: 1}); !errors.Is(err, ErrRiskLayerBounds) {
		t.Fatalf("单格两层应拒绝，得到 %v", err)
	}
}

func TestRiskAllZero(t *testing.T) {
	rows := [][]float64{
		{0, 0, 0},
		{0, 0, 0},
	}
	f := mustField(t, rows)
	s, err := f.Layering(LayerOptions{Levels: 3, MinPerLayer: 1})
	if err != nil {
		t.Fatal(err)
	}
	verifyAll(t, f, s)
	for _, l := range s.Layers {
		if l.Lo != 0 || l.Hi != 0 {
			t.Fatalf("全零场第 %d 层区间应为 [0,0]，得到 [%g,%g]", l.Index, l.Lo, l.Hi)
		}
	}
	for _, c := range s.Cuts {
		if c.Tie != true {
			t.Fatalf("全零场每个切点都是平局")
		}
		if c.Bound <= 0 {
			t.Fatalf("全零场应给出正的分位上界")
		}
	}
	t.Logf("\n%s\n%s", s.Table(), s.CutsTable())
}

func TestRiskThresholdsMonotone(t *testing.T) {
	f := mustField(t, [][]float64{{0, 1, 2, 3, 4, 5, 6, 7}})
	s, err := f.Layering(LayerOptions{Levels: 4, MinPerLayer: 1})
	if err != nil {
		t.Fatal(err)
	}
	verifyAll(t, f, s)
	for i := 1; i < len(s.Thresholds); i++ {
		if s.Thresholds[i] < s.Thresholds[i-1] {
			t.Fatalf("阈值非单调: %v", s.Thresholds)
		}
	}
	for i := 1; i < len(s.Layers); i++ {
		if s.Layers[i].Lo < s.Layers[i-1].Hi {
			t.Fatalf("层区间交错: %+v", s.Layers)
		}
	}
}
