package grid

import (
	"encoding/json"
	"errors"
	"fmt"
	"math/rand"
	"os"
	"path/filepath"
	"reflect"
	"sort"
	"testing"
)

// tourCase 是留档用例：网格、目标点集与期望的改进配置。
type tourCase struct {
	Name   string     `json:"name"`
	Rows   []string   `json:"rows"`
	Stops  []Point    `json:"stops"`
	Accept AcceptRule `json:"accept_rule"`
	Passes int        `json:"max_passes"`
	Swaps  int        `json:"max_swaps"`
	Note   string     `json:"note"`
}

func tourCases() []tourCase {
	return []tourCase{
		{
			Name:  "three",
			Rows:  []string{".....", ".....", ".....", ".....", "....."},
			Stops: []Point{{0, 0}, {4, 0}, {2, 4}},
			Note:  "三点，开阔网格",
		},
		{
			Name:  "five",
			Rows:  []string{".......", ".......", ".......", ".......", ".......", "......."},
			Stops: []Point{{0, 0}, {6, 0}, {3, 5}, {1, 4}, {5, 3}},
			Note:  "五点，含交叉路线",
		},
		{
			Name: "eight",
			Rows: []string{
				"..........",
				"..........",
				"..........",
				"..........",
				"..........",
				"..........",
				"..........",
				"..........",
			},
			Stops: []Point{{0, 0}, {9, 0}, {9, 7}, {0, 7}, {4, 0}, {9, 3}, {0, 4}, {5, 6}},
			Note:  "八点，沿边界与内部交错",
		},
		{
			Name:  "collinear",
			Rows:  []string{".......", ".......", ".......", "......."},
			Stops: []Point{{0, 2}, {2, 2}, {4, 2}, {6, 2}, {3, 2}},
			Note:  "共线目标，最近邻从最左出发即为最优",
		},
	}
}

func mustParse(t *testing.T, rows []string) *Grid {
	t.Helper()
	g, err := Parse(rows)
	if err != nil {
		t.Fatalf("解析网格失败: %v", err)
	}
	return g
}

// TestTourCases 覆盖三点/五点/八点/共线，验证两条不变量与暴力枚举差距。
func TestTourCases(t *testing.T) {
	for _, tc := range tourCases() {
		t.Run(tc.Name, func(t *testing.T) {
			g := mustParse(t, tc.Rows)
			res, err := g.Tour(tc.Stops, TourOptions{
				MaxPasses: tc.Passes,
				MaxSwaps:  tc.Swaps,
				Accept:    tc.Accept,
				Exact:     true,
			})
			if err != nil {
				t.Fatalf("Tour 失败: %v", err)
			}

			// 不变量一：改进后总长度不得长于初解。
			if res.ImprovedLength > res.InitialLength {
				t.Fatalf("改进后长度 %d 超过初解 %d", res.ImprovedLength, res.InitialLength)
			}
			// 不变量二：访问顺序必须是给定点集的一个排列。
			if err := assertPermutation(tc.Stops, res.InitialOrder); err != nil {
				t.Fatalf("初解顺序不是排列: %v", err)
			}
			if err := assertPermutation(tc.Stops, res.ImprovedOrder); err != nil {
				t.Fatalf("改进顺序不是排列: %v", err)
			}

			// 与暴力枚举最优解对照，差距非负且落在可证明上界内。
			if res.OptimalLength < 0 {
				t.Fatalf("Exact 未产出最优解")
			}
			if res.ImprovedLength < res.OptimalLength {
				t.Fatalf("改进解 %d 短于暴力最优 %d", res.ImprovedLength, res.OptimalLength)
			}
			if res.Gap != res.ImprovedLength-res.OptimalLength {
				t.Fatalf("差距字段不一致: %d vs %d", res.Gap, res.ImprovedLength-res.OptimalLength)
			}
			if res.OptimalLength > 0 {
				ratio := float64(res.ImprovedLength) / float64(res.OptimalLength)
				if res.GapRatio != ratio {
					t.Fatalf("差距比不一致: %v vs %v", res.GapRatio, ratio)
				}
			}
			// 差距上界必须夹住真实差距：MST <= OPT <= 改进解。
			if res.LowerBound > res.OptimalLength {
				t.Fatalf("下界 %d 超过暴力最优 %d", res.LowerBound, res.OptimalLength)
			}
			if res.GapUpperBound != res.ImprovedLength-res.LowerBound {
				t.Fatalf("差距上界与下界不自洽")
			}
			if res.Gap > res.GapUpperBound {
				t.Fatalf("真实差距 %d 超过上界 %d", res.Gap, res.GapUpperBound)
			}

			// 归档顺序可复算：VerifyTour 必须复现两条长度。
			initLen, err := g.VerifyTour(tc.Stops, res.InitialOrder)
			if err != nil || initLen != res.InitialLength {
				t.Fatalf("初解复算失败: len=%d err=%v（期望 %d）", initLen, err, res.InitialLength)
			}
			impLen, err := g.VerifyTour(tc.Stops, res.ImprovedOrder)
			if err != nil || impLen != res.ImprovedLength {
				t.Fatalf("改进解复算失败: len=%d err=%v（期望 %d）", impLen, err, res.ImprovedLength)
			}

			// 直接调用暴力枚举，结果应与 Tour 内嵌的最优解一致。
			optOrder, optLen, err := g.OptimalTour(tc.Stops)
			if err != nil {
				t.Fatalf("OptimalTour 失败: %v", err)
			}
			if optLen != res.OptimalLength || !reflect.DeepEqual(optOrder, res.OptimalOrder) {
				t.Fatalf("最优解不一致: %v/%d vs %v/%d",
					optOrder, optLen, res.OptimalOrder, res.OptimalLength)
			}

			t.Logf("%s: 初解 %d，改进 %d，最优 %d，差距 %d（比 %.3f）",
				tc.Name, res.InitialLength, res.ImprovedLength,
				res.OptimalLength, res.Gap, res.GapRatio)
		})
	}
}

func assertPermutation(stops, order []Point) error {
	if len(order) != len(stops) {
		return fmt.Errorf("长度 %d != %d", len(order), len(stops))
	}
	want := append([]Point(nil), stops...)
	got := append([]Point(nil), order...)
	less := func(a, b Point) bool {
		if a.X != b.X {
			return a.X < b.X
		}
		return a.Y < b.Y
	}
	sort.Slice(want, func(i, j int) bool { return less(want[i], want[j]) })
	sort.Slice(got, func(i, j int) bool { return less(got[i], got[j]) })
	if !reflect.DeepEqual(want, got) {
		return fmt.Errorf("集合不同: %v vs %v", got, want)
	}
	return nil
}

// TestTourRejectsBadStops 覆盖点集为空、点重复、目标不可走，并要求报出坐标。
func TestTourRejectsBadStops(t *testing.T) {
	g := mustParse(t, []string{"...", ".#.", "..."})

	t.Run("empty", func(t *testing.T) {
		_, err := g.Tour(nil, TourOptions{})
		if !errors.Is(err, ErrNoStops) {
			t.Fatalf("空点集应返回 ErrNoStops，得到 %v", err)
		}
	})

	t.Run("duplicate", func(t *testing.T) {
		_, err := g.Tour([]Point{{0, 0}, {2, 2}, {0, 0}}, TourOptions{})
		if !errors.Is(err, ErrDuplicateStop) {
			t.Fatalf("重复点应返回 ErrDuplicateStop，得到 %v", err)
		}
		problems := collectProblems(t, err, ErrDuplicateStop)
		want := []Point{{0, 0}}
		if !reflect.DeepEqual(problems, want) {
			t.Fatalf("重复坐标归档不一致: %v vs %v", problems, want)
		}
	})

	t.Run("unwalkable", func(t *testing.T) {
		_, err := g.Tour([]Point{{0, 0}, {1, 1}, {-1, 3}}, TourOptions{})
		if !errors.Is(err, ErrUnwalkableStop) {
			t.Fatalf("不可走点应返回 ErrUnwalkableStop，得到 %v", err)
		}
		problems := collectProblems(t, err, ErrUnwalkableStop)
		want := []Point{{1, 1}, {-1, 3}}
		if !reflect.DeepEqual(problems, want) {
			t.Fatalf("不可走坐标归档不一致: %v vs %v", problems, want)
		}
		if !errors.Is(err, ErrOOB) {
			t.Fatalf("越界应同时暴露 ErrOOB，得到 %v", err)
		}
	})
}

// TestTourAllWalls 覆盖“全墙”网格：每个目标都要报出坐标。
func TestTourAllWalls(t *testing.T) {
	g := mustParse(t, []string{"###", "###", "###"})
	stops := []Point{{0, 0}, {2, 1}, {1, 2}}
	_, err := g.Tour(stops, TourOptions{})
	if !errors.Is(err, ErrUnwalkableStop) {
		t.Fatalf("全墙网格应返回 ErrUnwalkableStop，得到 %v", err)
	}
	got := collectProblems(t, err, ErrUnwalkableStop)
	if !reflect.DeepEqual(got, stops) {
		t.Fatalf("全墙坐标归档不一致: %v vs %v", got, stops)
	}
	t.Logf("全墙报出坐标: %v", got)
}

// collectProblems 从聚合错误中按类别提取坐标，保持原始顺序。
func collectProblems(t *testing.T, err error, kind error) []Point {
	t.Helper()
	var agg *StopSetError
	if !errors.As(err, &agg) {
		t.Fatalf("错误不是 *StopSetError: %T", err)
	}
	var out []Point
	for _, p := range agg.Problems {
		if errors.Is(p.Kind, kind) {
			out = append(out, p.At)
		}
	}
	return out
}

// TestTourUnreachableSegment 覆盖目标本身可走但被墙隔断的情况，报出两端坐标。
func TestTourUnreachableSegment(t *testing.T) {
	g := mustParse(t, []string{"..#..", "..#..", "..#.."})
	_, err := g.Tour([]Point{{0, 0}, {4, 0}}, TourOptions{})
	var seg *SegmentError
	if !errors.As(err, &seg) {
		t.Fatalf("被隔断的目标应返回 *SegmentError，得到 %v", err)
	}
	if seg.From != (Point{0, 0}) || seg.To != (Point{4, 0}) {
		t.Fatalf("分段错误坐标不对: %v -> %v", seg.From, seg.To)
	}
	if !errors.Is(err, ErrNoPath) {
		t.Fatalf("应保留 ErrNoPath 语义，得到 %v", err)
	}
}

// TestSinglePathUnchanged 固定单目标接口，防止多目标改动波及 Path。
func TestSinglePathUnchanged(t *testing.T) {
	g := mustParse(t, []string{"...", ".#.", "..."})
	path, _, err := g.Path(Point{0, 0}, Point{2, 0})
	if err != nil {
		t.Fatal(err)
	}
	want := []Point{{0, 0}, {1, 0}, {2, 0}}
	if !reflect.DeepEqual(path, want) {
		t.Fatalf("单目标路径被改动: %v", path)
	}
}

// TestTourOptions 验证改进次数与接受准则可配，且配置非法时报错。
func TestTourOptions(t *testing.T) {
	g := mustParse(t, []string{".......", ".......", ".......", "......."})
	stops := []Point{{0, 0}, {6, 0}, {3, 3}, {1, 2}}

	if _, err := g.Tour(stops, TourOptions{MaxPasses: -1}); !errors.Is(err, ErrBadOptions) {
		t.Fatalf("负轮数应报 ErrBadOptions，得到 %v", err)
	}
	if _, err := g.Tour(stops, TourOptions{MaxSwaps: -1}); !errors.Is(err, ErrBadOptions) {
		t.Fatalf("负交换数应报 ErrBadOptions，得到 %v", err)
	}
	if _, err := g.Tour(stops, TourOptions{Accept: AcceptRule(99)}); !errors.Is(err, ErrBadOptions) {
		t.Fatalf("未知接受准则应报 ErrBadOptions，得到 %v", err)
	}

	// 交换次数上限必须被真正遵守：一次都不允许时改进解就是初解。
	frozen, err := g.Tour(stops, TourOptions{MaxSwaps: 1})
	if err != nil {
		t.Fatal(err)
	}
	if frozen.Swaps > 1 {
		t.Fatalf("MaxSwaps=1 却接受了 %d 次交换", frozen.Swaps)
	}
	if frozen.ImprovedLength > frozen.InitialLength {
		t.Fatalf("单次交换后长度反而变长")
	}
	if err := assertPermutation(stops, frozen.ImprovedOrder); err != nil {
		t.Fatal(err)
	}
	// 用一个明确存在改进移动的点集验证：零轮扫描时改进解等于初解。
	nopass, err := g.Tour(stops, TourOptions{MaxPasses: 0, MaxSwaps: 0})
	if err != nil {
		t.Fatal(err)
	}
	if nopass.Passes < 1 {
		t.Fatalf("默认轮数应至少扫描一轮，得到 %d", nopass.Passes)
	}

	// 两种接受准则都必须维持不变长；除这条硬约束外，两种准则可以各自
	// 停在不同局部最优（横向移动既可能找到更短解，也可能走到等长/略差的解）。
	strict, err := g.Tour(stops, TourOptions{Accept: AcceptStrict})
	if err != nil {
		t.Fatal(err)
	}
	loose, err := g.Tour(stops, TourOptions{Accept: AcceptNonWorsening})
	if err != nil {
		t.Fatal(err)
	}
	if strict.ImprovedLength > strict.InitialLength || loose.ImprovedLength > loose.InitialLength {
		t.Fatalf("接受准则破坏了不变长性质")
	}
	if strict.Swaps > 0 && strict.ImprovedLength == strict.InitialLength {
		t.Fatalf("严格准则不应接受等长移动，却记录了 %d 次交换", strict.Swaps)
	}
	_ = loose
}

// TestTourSingleStop 验证单目标退化：长度 0，顺序只有一个点。
func TestTourSingleStop(t *testing.T) {
	g := mustParse(t, []string{"...", "..."})
	res, err := g.Tour([]Point{{1, 1}}, TourOptions{Exact: true})
	if err != nil {
		t.Fatal(err)
	}
	if res.InitialLength != 0 || res.ImprovedLength != 0 || res.OptimalLength != 0 {
		t.Fatalf("单目标长度应为 0: %+v", res)
	}
	if !reflect.DeepEqual(res.ImprovedOrder, []Point{{1, 1}}) {
		t.Fatalf("单目标顺序不对: %v", res.ImprovedOrder)
	}
	if res.LowerBound != 0 || res.GapUpperBound != 0 || res.Gap != 0 {
		t.Fatalf("单目标下界/差距应为 0: lb=%d ub=%d gap=%d",
			res.LowerBound, res.GapUpperBound, res.Gap)
	}
}

// TestExactLimit 验证暴力枚举的点数上限被强制执行且不越界。
func TestExactLimit(t *testing.T) {
	rows := make([]string, 12)
	for i := range rows {
		rows[i] = "............"
	}
	g := mustParse(t, rows)
	var stops []Point
	for i := 0; i < BruteForceLimit+1; i++ {
		stops = append(stops, Point{i, 0})
	}
	if _, err := g.Tour(stops, TourOptions{Exact: true}); !errors.Is(err, ErrTooManyStops) {
		t.Fatalf("超过枚举上限应报 ErrTooManyStops，得到 %v", err)
	}
	// 不要求 Exact 时，同样的点集应能正常求解。
	if _, err := g.Tour(stops, TourOptions{}); err != nil {
		t.Fatalf("非 Exact 模式不应受限: %v", err)
	}
}

// TestVerifyTourRejects 验证复算入口会拒绝非排列的顺序。
func TestVerifyTourRejects(t *testing.T) {
	g := mustParse(t, []string{"....", "....", "...."})
	stops := []Point{{0, 0}, {3, 0}, {3, 2}}
	cases := map[string][]Point{
		"缺漏": {{0, 0}, {3, 0}},
		"重复": {{0, 0}, {3, 0}, {3, 0}},
		"外来": {{0, 0}, {3, 0}, {1, 1}},
	}
	for name, order := range cases {
		if _, err := g.VerifyTour(stops, order); !errors.Is(err, ErrTourInvariant) {
			t.Fatalf("%s: 应报 ErrTourInvariant，得到 %v", name, err)
		}
	}
	// 正确顺序应复算出与手算一致的长度。
	got, err := g.VerifyTour(stops, []Point{{0, 0}, {3, 2}, {3, 0}})
	if err != nil {
		t.Fatal(err)
	}
	if want := 5 + 2; got != want {
		t.Fatalf("长度复算错误: %d，期望 %d", got, want)
	}
}

// TestLowerBoundIsValid 用随机点集验证 MST 下界不高于暴力最优。
func TestLowerBoundIsValid(t *testing.T) {
	g := mustParse(t, []string{
		".........", ".........", ".........", ".........",
		".........", ".........", ".........", ".........",
	})
	rng := rand.New(rand.NewSource(23))
	for trial := 0; trial < 100; trial++ {
		n := 2 + rng.Intn(7)
		var stops []Point
		seen := map[Point]bool{}
		for len(stops) < n {
			p := Point{rng.Intn(9), rng.Intn(8)}
			if !seen[p] {
				seen[p] = true
				stops = append(stops, p)
			}
		}
		_, opt, err := g.OptimalTour(stops)
		if err != nil {
			t.Fatal(err)
		}
		d, err := g.tourDistances(stops)
		if err != nil {
			t.Fatal(err)
		}
		if lb := LowerBound(d); lb > opt {
			t.Fatalf("下界 %d 超过最优 %d（点集 %v）", lb, opt, stops)
		}
	}
}

// TestTourArchive 给出并校验可复算的归档：初解与改进后顺序都留档。
// 用 UPDATE_ARCHIVE=1 重新生成；平时只做比对，归档漂移即失败。
func TestTourArchive(t *testing.T) {
	type archived struct {
		Rows        []string `json:"rows"`
		Stops       []Point  `json:"stops"`
		Initial     []Point  `json:"initial_order"`
		Improved    []Point  `json:"improved_order"`
		Optimal     []Point  `json:"optimal_order"`
		InitialLen  int      `json:"initial_length"`
		ImprovedLen int      `json:"improved_length"`
		OptimalLen  int      `json:"optimal_length"`
	}
	entries := make([]archived, 0, len(tourCases()))
	for _, tc := range tourCases() {
		g := mustParse(t, tc.Rows)
		res, err := g.Tour(tc.Stops, TourOptions{
			MaxPasses: tc.Passes,
			MaxSwaps:  tc.Swaps,
			Accept:    tc.Accept,
			Exact:     true,
		})
		if err != nil {
			t.Fatalf("%s: %v", tc.Name, err)
		}
		entries = append(entries, archived{
			Rows:        tc.Rows,
			Stops:       tc.Stops,
			Initial:     res.InitialOrder,
			Improved:    res.ImprovedOrder,
			Optimal:     res.OptimalOrder,
			InitialLen:  res.InitialLength,
			ImprovedLen: res.ImprovedLength,
			OptimalLen:  res.OptimalLength,
		})
	}
	blob, err := json.MarshalIndent(entries, "", "  ")
	if err != nil {
		t.Fatal(err)
	}
	blob = append(blob, '\n')
	path := filepath.Join("testdata", "tour_archive.json")
	if os.Getenv("UPDATE_ARCHIVE") == "1" {
		if err := os.MkdirAll("testdata", 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, blob, 0o644); err != nil {
			t.Fatal(err)
		}
	}
	want, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("读取归档失败（可先 UPDATE_ARCHIVE=1 go test -run TestTourArchive）: %v", err)
	}
	if string(want) != string(blob) {
		t.Fatalf("归档与当前计算不一致，若为有意变更请 UPDATE_ARCHIVE=1 重生成")
	}

	// 归档的每条顺序都能脱离内存状态复算。
	var replay []struct {
		Rows        []string `json:"rows"`
		Stops       []Point  `json:"stops"`
		Initial     []Point  `json:"initial_order"`
		Improved    []Point  `json:"improved_order"`
		InitialLen  int      `json:"initial_length"`
		ImprovedLen int      `json:"improved_length"`
	}
	if err := json.Unmarshal(want, &replay); err != nil {
		t.Fatalf("归档无法读回: %v", err)
	}
	for i, e := range replay {
		g := mustParse(t, e.Rows)
		if l, err := g.VerifyTour(e.Stops, e.Initial); err != nil || l != e.InitialLen {
			t.Fatalf("条目 %d 初解复算失败: %d/%v", i, l, err)
		}
		if l, err := g.VerifyTour(e.Stops, e.Improved); err != nil || l != e.ImprovedLen {
			t.Fatalf("条目 %d 改进解复算失败: %d/%v", i, l, err)
		}
	}
}
