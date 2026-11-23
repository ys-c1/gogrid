package grid

import (
	"errors"
	"strings"
	"testing"
)

// assertPartition 通用不变量：覆盖完整、无重复、墙不被分配，并返回各区面积。
func assertPartition(t *testing.T, g *Grid, p *Partition) map[Point]int {
	t.Helper()
	if err := g.CheckCoverage(p); err != nil {
		t.Fatalf("覆盖校验失败: %v", err)
	}
	owner := map[Point]int{}
	for i, r := range p.Regions {
		for _, c := range r.Cells {
			owner[c] = i
		}
	}
	if len(owner) != g.WalkableCount() {
		t.Fatalf("分配格数 %d 不等于可走格数 %d", len(owner), g.WalkableCount())
	}
	return owner
}

func TestGrowHomogeneous(t *testing.T) {
	// 4x4 全可走，四角种子，目标面积各 4。
	g, _ := Parse([]string{"....", "....", "....", "...."})
	seeds := []Point{{0, 0}, {3, 0}, {0, 3}, {3, 3}}
	p, err := g.GrowRegions(seeds)
	if err != nil {
		t.Fatal(err)
	}
	assertPartition(t, g, p)
	if len(p.Regions) != 4 {
		t.Fatalf("辖区数 %d", len(p.Regions))
	}
	min, max, spread := p.RangeStat()
	if min != 4 || max != 4 || spread != 0 {
		t.Fatalf("均质图应四等分，得到 min=%d max=%d spread=%d", min, max, spread)
	}
	if err := g.CheckBalance(p, 0.0); err != nil {
		t.Fatalf("极差为零时零容差也应通过: %v", err)
	}
	t.Log("\n" + p.Report())
}

func TestGrowLongCorridor(t *testing.T) {
	// 1x8 狭长走廊，两端种子，各 4 格。
	g, _ := Parse([]string{"........"})
	seeds := []Point{{0, 0}, {7, 0}}
	p, err := g.GrowRegions(seeds)
	if err != nil {
		t.Fatal(err)
	}
	assertPartition(t, g, p)
	min, max, spread := p.RangeStat()
	if min != 4 || max != 4 || spread != 0 {
		t.Fatalf("走廊应平分，得到 min=%d max=%d spread=%d", min, max, spread)
	}
	// 各辖区必须连通（走廊里等价于一段连续区间）。
	for i, r := range p.Regions {
		if !connected(g, r.Cells) {
			t.Fatalf("辖区%d 不连通", i)
		}
	}
}

func TestGrowWithObstacles(t *testing.T) {
	// 5x5 带十字形障碍，墙 6 格、可走 19 格，两个种子，目标面积 9/10。
	g, _ := Parse([]string{
		".....",
		".###.",
		".....",
		".###.",
		".....",
	})
	if g.WalkableCount() != 19 {
		t.Fatalf("可走格应为 19，实际 %d", g.WalkableCount())
	}
	seeds := []Point{{0, 0}, {4, 4}}
	p, err := g.GrowRegions(seeds)
	if err != nil {
		t.Fatal(err)
	}
	assertPartition(t, g, p)
	min, max, spread := p.RangeStat()
	// 19 格分两区：9 与 10。
	if min != 9 || max != 10 || spread != 1 {
		t.Fatalf("障碍图面积应为 9/10，得到 min=%d max=%d spread=%d", min, max, spread)
	}
	if err := g.CheckBalance(p, 0.2); err != nil {
		t.Fatalf("极差比例 0.2 内应通过: %v", err)
	}
	for i, r := range p.Regions {
		if !connected(g, r.Cells) {
			t.Fatalf("辖区%d 不连通", i)
		}
	}
}

func TestGrowSingleSeed(t *testing.T) {
	// 3x3 单种子，全部 9 格归一个辖区。
	g, _ := Parse([]string{"...", "...", "..."})
	p, err := g.GrowRegions([]Point{{1, 1}})
	if err != nil {
		t.Fatal(err)
	}
	assertPartition(t, g, p)
	if len(p.Regions) != 1 || p.Area(0) != 9 {
		t.Fatalf("单种子应独吞 9 格，得到 %d 区 %d 格", len(p.Regions), p.Area(0))
	}
	if err := g.CheckBalance(p, 0.0); err != nil {
		t.Fatal(err)
	}

	// 共享边长度择优：第一步只能选与种子共享 1 条边的四邻；
	// 并列时行优先，(1,0) 在 (0,1)/(2,1)/(1,2) 之前。
	got := p.Regions[0].Cells
	wantOrder := []Point{{1, 1}, {1, 0}}
	for i, w := range wantOrder {
		if got[i] != w {
			t.Fatalf("生长次序第 %d 格应为 %v，实际 %v（未按共享边/行优先择优）", i, w, got[i])
		}
	}
}

func TestGrowDeterministic(t *testing.T) {
	// 同一种子顺序跑两次，分配结果必须逐格一致（全程无随机）。
	g, _ := Parse([]string{
		".....",
		".##..",
		"..#..",
		"...#.",
		".....",
	})
	seeds := []Point{{0, 0}, {4, 0}, {0, 4}, {4, 4}}
	p1, err := g.GrowRegions(seeds)
	if err != nil {
		t.Fatal(err)
	}
	p2, err := g.GrowRegions(seeds)
	if err != nil {
		t.Fatal(err)
	}
	if len(p1.Regions) != len(p2.Regions) {
		t.Fatal("两次运行辖区数不一致")
	}
	for i := range p1.Regions {
		if len(p1.Regions[i].Cells) != len(p2.Regions[i].Cells) {
			t.Fatalf("辖区%d 两次面积不一致", i)
		}
		for j, c := range p1.Regions[i].Cells {
			if c != p2.Regions[i].Cells[j] {
				t.Fatalf("辖区%d 第%d格两次不一致: %v vs %v", i, j, c, p2.Regions[i].Cells[j])
			}
		}
	}
	// 调换种子顺序后，面积分布仍应一致（对称输入下），且依旧全覆盖。
	assertPartition(t, g, p1)
}

func TestGrowBottleneckOverflow(t *testing.T) {
	// 瓶颈地形：左侧宽厅通过单格瓶颈接右侧窄道，
	// 阶段一可能有区域被卡住，阶段二负责补齐覆盖并维持面积均衡。
	g, _ := Parse([]string{
		"...#.",
		"...#.",
		".....",
		"...#.",
		"...#.",
	})
	// 可走格：左厅 3x5=15，右道四格 (4,0)(4,1)(4,3)(4,4)，中间行全通 5 格，共 21。
	if g.WalkableCount() != 21 {
		t.Fatalf("可走格应为 21，实际 %d", g.WalkableCount())
	}
	seeds := []Point{{0, 0}, {4, 2}}
	p, err := g.GrowRegions(seeds)
	if err != nil {
		t.Fatalf("瓶颈地形阶段二应补齐覆盖: %v", err)
	}
	assertPartition(t, g, p)
	if err := g.CheckBalance(p, 0.2); err != nil {
		// 21 格两区为 10/11，极差/均值 ≈ 0.095，0.2 必须通过。
		t.Fatalf("瓶颈分区平衡校验失败: %v", err)
	}
	for i, r := range p.Regions {
		if !connected(g, r.Cells) {
			t.Fatalf("辖区%d 不连通", i)
		}
	}
	min, max, spread := p.RangeStat()
	if spread > 1 {
		t.Fatalf("瓶颈分区极差应 ≤1，得到 %d (min=%d max=%d)", spread, min, max)
	}
}

func TestErrNoSeed(t *testing.T) {
	g, _ := Parse([]string{"..."})
	if _, err := g.GrowRegions(nil); !errors.Is(err, ErrNoSeed) {
		t.Fatalf("零种子应报 ErrNoSeed，得到 %v", err)
	}
	if _, err := g.GrowRegions([]Point{}); !errors.Is(err, ErrNoSeed) {
		t.Fatalf("空种子切片应报 ErrNoSeed，得到 %v", err)
	}
}

func TestErrSeedInWall(t *testing.T) {
	g, _ := Parse([]string{".#.", "..."})
	_, err := g.GrowRegions([]Point{{1, 0}})
	if !errors.Is(err, ErrSeedWall) {
		t.Fatalf("墙内种子应报 ErrSeedWall，得到 %v", err)
	}
	if !strings.Contains(err.Error(), "(1,0)") {
		t.Fatalf("错误信息应携带坐标，得到 %q", err.Error())
	}
	// 越界与落墙分别报不同错误。
	_, err = g.GrowRegions([]Point{{5, 0}})
	if !errors.Is(err, ErrSeedOOB) || !strings.Contains(err.Error(), "(5,0)") {
		t.Fatalf("越界种子应报带坐标的 ErrSeedOOB，得到 %v", err)
	}
}

func TestErrTooFewAndAllWall(t *testing.T) {
	// 全墙：可走格 0 < 种子数，报可走格不足并带坐标。
	g, _ := Parse([]string{"###", "###"})
	_, err := g.GrowRegions([]Point{{0, 0}, {1, 1}})
	if !errors.Is(err, ErrTooFew) {
		t.Fatalf("全墙应报 ErrTooFew，得到 %v", err)
	}
	if !strings.Contains(err.Error(), "(0,0)") {
		t.Fatalf("错误信息应携带首个种子坐标，得到 %q", err.Error())
	}

	// 单格可走但两个种子：同样不足。
	g2, _ := Parse([]string{".#", "##"})
	_, err = g2.GrowRegions([]Point{{0, 0}, {0, 0}})
	// 先经过数量检查（1 < 2），因此报 ErrTooFew 而不是重复。
	if !errors.Is(err, ErrTooFew) {
		t.Fatalf("可走格不足应报 ErrTooFew，得到 %v", err)
	}
}

func TestErrSeedDupAndUnreachable(t *testing.T) {
	// 重复种子：可走格充足时才暴露重复问题，带坐标。
	g, _ := Parse([]string{"...", "..."})
	_, err := g.GrowRegions([]Point{{0, 0}, {0, 0}})
	if !errors.Is(err, ErrSeedDup) || !strings.Contains(err.Error(), "(0,0)") {
		t.Fatalf("重复种子应报带坐标的 ErrSeedDup，得到 %v", err)
	}

	// 两个互不连通的封闭可走区，种子只覆盖其一：另一区不可达，报坐标。
	// 可走格为 (0,0)(2,0)(0,2)(2,2)，行优先扫描下首个不可达格是 (2,0)。
	g2, _ := Parse([]string{
		".#.",
		"###",
		".#.",
	})
	_, err = g2.GrowRegions([]Point{{0, 0}})
	if !errors.Is(err, ErrUnreachable) {
		t.Fatalf("封闭区应报 ErrUnreachable，得到 %v", err)
	}
	if !strings.Contains(err.Error(), "(2,0)") {
		t.Fatalf("错误信息应携带不可达格坐标，得到 %q", err.Error())
	}
}

func TestChecksAreIndependent(t *testing.T) {
	// 两条校验分开验：手工构造覆盖完整但极差超标的分区，
	// CheckCoverage 通过而 CheckBalance 失败。
	g, _ := Parse([]string{"....", "...."})
	p := &Partition{
		g: g,
		Regions: []Region{
			{Seed: Point{0, 0}, Cells: []Point{{0, 0}}},
			{Seed: Point{3, 1}, Cells: []Point{
				{1, 0}, {2, 0}, {3, 0},
				{0, 1}, {1, 1}, {2, 1}, {3, 1},
			}},
		},
	}
	if err := g.CheckCoverage(p); err != nil {
		t.Fatalf("手工分区覆盖完整，CheckCoverage 应通过: %v", err)
	}
	if err := g.CheckBalance(p, 0.2); err == nil {
		t.Fatal("面积 1/7 极差 6，比例 0.2 下 CheckBalance 必须失败")
	}
	if err := g.CheckBalance(p, 2.0); err != nil {
		t.Fatalf("放宽到 2.0 后应通过: %v", err)
	}
	if err := g.CheckBalance(p, -0.1); !errors.Is(err, ErrBadRatio) {
		t.Fatalf("负比例应报 ErrBadRatio，得到 %v", err)
	}

	// 反向：覆盖缺失时 CheckCoverage 单独失败。
	p2 := &Partition{
		g:       g,
		Regions: []Region{{Seed: Point{0, 0}, Cells: []Point{{0, 0}}}},
	}
	if err := g.CheckCoverage(p2); err == nil ||
		!strings.Contains(err.Error(), "未被任何辖区分配") {
		t.Fatalf("漏分格应被 CheckCoverage 捕获，得到 %v", err)
	}
}

// connected 用 BFS 校验一组格在网格中四连通。
func connected(g *Grid, cells []Point) bool {
	set := make(map[Point]bool, len(cells))
	for _, c := range cells {
		set[c] = true
	}
	if len(set) == 0 {
		return false
	}
	start := cells[0]
	queue := []Point{start}
	seen := map[Point]bool{start: true}
	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		for _, d := range [4]Point{{1, 0}, {-1, 0}, {0, 1}, {0, -1}} {
			np := Point{cur.X + d.X, cur.Y + d.Y}
			if set[np] && !seen[np] {
				seen[np] = true
				queue = append(queue, np)
			}
		}
	}
	return len(seen) == len(set)
}
