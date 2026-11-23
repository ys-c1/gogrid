package grid

import (
	"errors"
	"fmt"
	"testing"
)

// 用例一：同向——T 形汇合，两条等长支路在同一格交汇。两队到交汇格距离相同，
// 同时放行必争格，低优先级队在己方支路让行一窗。
func TestFleetSameDirection(t *testing.T) {
	g, _ := Parse([]string{
		"..#..",
		".....",
		".....",
	})
	teams := []FleetTeam{
		{ID: 1, Priority: 2, Start: Point{0, 0}, Goal: Point{4, 2}},
		{ID: 2, Priority: 1, Start: Point{0, 2}, Goal: Point{4, 0}},
	}
	plan, err := g.FleetSchedule(teams, 15)
	if err != nil {
		t.Fatal(err)
	}
	if c := plan.Verify(); !c.OK() {
		t.Fatalf("排程核验失败: %+v", c)
	}
	if plan.ConflictN != 1 || plan.Makespan != 7 {
		t.Fatalf("同向用例期望 1 次冲突/完成时间 7，实际 %d/%d", plan.ConflictN, plan.Makespan)
	}
	if plan.FinishAt[2] != 6 || plan.FinishAt[1] != 7 {
		t.Fatalf("各队完成时间异常: %v", plan.FinishAt)
	}
	assertWindowBound(t, plan)
}

// 用例二：相向——墙体把网格分成东西两区，仅中心格 (2,2) 相通。两队自西北、东南
// 对向行驶，最短路径在中心格附近的共享支路上连续争用，低优先级队在己方支路让行。
func TestFleetHeadOn(t *testing.T) {
	g, _ := Parse([]string{
		"..#..",
		"..#..",
		".....",
		"..#..",
		"..#..",
	})
	teams := []FleetTeam{
		{ID: 1, Priority: 2, Start: Point{0, 0}, Goal: Point{4, 4}},
		{ID: 2, Priority: 1, Start: Point{4, 4}, Goal: Point{0, 0}},
	}
	plan, err := g.FleetSchedule(teams, 20)
	if err != nil {
		t.Fatal(err)
	}
	if c := plan.Verify(); !c.OK() {
		t.Fatalf("排程核验失败: %+v", c)
	}
	if plan.ConflictN != 3 || plan.Makespan != 11 {
		t.Fatalf("相向用例期望 3 次冲突/完成时间 11，实际 %d/%d", plan.ConflictN, plan.Makespan)
	}
	if plan.FinishAt[2] != 8 || plan.FinishAt[1] != 11 {
		t.Fatalf("各队完成时间异常: %v", plan.FinishAt)
	}
	assertWindowBound(t, plan)
}

// 用例二附：开阔地直向对穿——让行点必在对方路径上，纯等待无解，报编号 4
// （这正是多队互相堵死的情形；单格宽走廊的对向堵死见 TestFleetNarrowCorridor）。
func TestFleetHeadOnStraightNoSolution(t *testing.T) {
	g, _ := Parse([]string{
		"...",
		"...",
		"...",
	})
	teams := []FleetTeam{
		{ID: 1, Priority: 2, Start: Point{0, 1}, Goal: Point{2, 1}},
		{ID: 2, Priority: 1, Start: Point{2, 1}, Goal: Point{0, 1}},
	}
	if _, err := g.FleetSchedule(teams, 15); !codeIs(err, 4) {
		t.Fatalf("直向对穿应报编号 4，实际: %v", err)
	}
}

// 用例三：十字交叉——两队路径在中心格十字相交，低优先级队在交叉点前让行一窗。
func TestFleetCross(t *testing.T) {
	g, _ := Parse([]string{
		"...",
		"...",
		"...",
	})
	teams := []FleetTeam{
		{ID: 1, Priority: 2, Start: Point{0, 1}, Goal: Point{2, 1}},
		{ID: 2, Priority: 1, Start: Point{1, 0}, Goal: Point{1, 2}},
	}
	plan, err := g.FleetSchedule(teams, 10)
	if err != nil {
		t.Fatal(err)
	}
	if c := plan.Verify(); !c.OK() {
		t.Fatalf("排程核验失败: %+v", c)
	}
	if plan.ConflictN != 1 || plan.Makespan != 3 {
		t.Fatalf("十字交叉用例期望 1 次冲突/完成时间 3，实际 %d/%d", plan.ConflictN, plan.Makespan)
	}
	assertWindowBound(t, plan)
}

// 用例四：狭窄走廊——单格宽走廊内对向行驶，互相堵死且无处避让，报编号 4。
func TestFleetNarrowCorridor(t *testing.T) {
	g, _ := Parse([]string{
		".......",
	})
	teams := []FleetTeam{
		{ID: 1, Priority: 1, Start: Point{0, 0}, Goal: Point{6, 0}},
		{ID: 2, Priority: 2, Start: Point{6, 0}, Goal: Point{0, 0}},
	}
	_, err := g.FleetSchedule(teams, 20)
	if !codeIs(err, 4) {
		t.Fatalf("狭窄走廊对向堵死应报编号 4，实际: %v", err)
	}
}

// 用例五：单队——结果与单队 Path 接口一致，零冲突。
func TestFleetSingle(t *testing.T) {
	g, _ := Parse([]string{
		"...",
		".#.",
		"...",
	})
	teams := []FleetTeam{{ID: 7, Priority: 1, Start: Point{0, 0}, Goal: Point{2, 2}}}
	plan, err := g.FleetSchedule(teams, 10)
	if err != nil {
		t.Fatal(err)
	}
	if c := plan.Verify(); !c.OK() {
		t.Fatalf("排程核验失败: %+v", c)
	}
	path, _, perr := g.Path(Point{0, 0}, Point{2, 2})
	if perr != nil {
		t.Fatal(perr)
	}
	if plan.ConflictN != 0 || plan.Makespan != len(path)-1 {
		t.Fatalf("单队用例期望 0 冲突/完成时间 %d，实际 %d/%d",
			len(path)-1, plan.ConflictN, plan.Makespan)
	}
	if len(plan.Routes[7]) != len(path) {
		t.Fatalf("单队排程路径应与 Path 一致: %v vs %v", plan.Routes[7], path)
	}
	assertWindowBound(t, plan)
}

// 三类非法输入各报编号。
func TestFleetIllegalInputs(t *testing.T) {
	g, _ := Parse([]string{"..", ".."})

	if _, err := g.FleetSchedule(nil, 10); !codeIs(err, 1) {
		t.Fatalf("队伍数为零应报编号 1，实际: %v", err)
	}

	empty, _ := Parse([]string{"##", "##"})
	if _, err := empty.FleetSchedule([]FleetTeam{{ID: 1, Start: Point{0, 0}, Goal: Point{1, 1}}}, 10); !codeIs(err, 2) {
		t.Fatalf("网格为空应报编号 2，实际: %v", err)
	}

	dup := []FleetTeam{
		{ID: 1, Start: Point{0, 0}, Goal: Point{1, 0}},
		{ID: 2, Start: Point{0, 0}, Goal: Point{1, 1}},
	}
	if _, err := g.FleetSchedule(dup, 10); !codeIs(err, 3) {
		t.Fatalf("起点重复应报编号 3，实际: %v", err)
	}
}

// 上界过紧但几何可行时必须报编号 5（区别于堵死的编号 4）。
func TestFleetOverBound(t *testing.T) {
	g, _ := Parse([]string{"...."})
	teams := []FleetTeam{{ID: 1, Start: Point{0, 0}, Goal: Point{3, 0}}}
	if _, err := g.FleetSchedule(teams, 2); !codeIs(err, 5) {
		t.Fatalf("上界过紧应报编号 5，实际: %v", err)
	}
	// 上界恰好等于路径长度时应通过。
	if _, err := g.FleetSchedule(teams, 3); err != nil {
		t.Fatalf("上界等于完成时间应通过，实际: %v", err)
	}
}

// 负上界直接拒绝；不可达终点包装为编号 4。
func TestFleetBoundAndUnreachable(t *testing.T) {
	g, _ := Parse([]string{
		".#.",
		"...",
	})
	teams := []FleetTeam{{ID: 1, Start: Point{0, 0}, Goal: Point{2, 0}}}
	if _, err := g.FleetSchedule(teams, -1); err == nil {
		t.Fatal("负上界应报错")
	}
	// 起点可走但被整行墙孤立，终点不可达，单队 Path 失败应包装成编号 4。
	g2, _ := Parse([]string{
		".##",
		"###",
		"...",
	})
	if _, err := g2.FleetSchedule(
		[]FleetTeam{{ID: 1, Start: Point{0, 0}, Goal: Point{2, 2}}}, 10); !codeIs(err, 4) {
		t.Fatalf("不可达终点应报编号 4，实际: %v", err)
	}
}

// 篡改排程制造同格冲突，Verify 必须抓到；再验证窗数反解的守门作用。
func TestFleetVerifyDetectsTampering(t *testing.T) {
	g, _ := Parse([]string{
		"...",
		"...",
	})
	teams := []FleetTeam{
		{ID: 1, Priority: 1, Start: Point{0, 0}, Goal: Point{2, 0}},
		{ID: 2, Priority: 2, Start: Point{0, 1}, Goal: Point{2, 1}},
	}
	plan, err := g.FleetSchedule(teams, 10)
	if err != nil {
		t.Fatal(err)
	}
	if c := plan.Verify(); !c.OK() {
		t.Fatalf("原始排程应通过核验: %+v", c)
	}

	// 人为把 t=1 两队都搬到同一格。
	plan.Windows[1].Seat[1] = Point{1, 0}
	plan.Windows[1].Seat[2] = Point{1, 0}
	c := plan.Verify()
	if len(c.SeatViolations) == 0 {
		t.Fatal("核验未能抓出同格冲突")
	}
	if c.OK() {
		t.Fatal("存在冲突时 OK 必须为假")
	}

	// 人为虚增时间窗，应超过由冲突数反解的窗数。
	before := len(plan.Windows)
	extra := FleetWindow{T: before, Seat: map[int]Point{1: {2, 0}, 2: {2, 1}}}
	plan.Windows = append(plan.Windows, extra)
	c2 := plan.Verify()
	if !c2.WindowOverflow || c2.ActualWindows <= c2.NeedWindows {
		t.Fatalf("窗数反解守门失效: actual=%d need=%d", c2.ActualWindows, c2.NeedWindows)
	}
}

// 篡改相邻两窗制造对向换边，Verify 必须抓到换边冲突。
func TestFleetVerifyDetectsSwap(t *testing.T) {
	g, _ := Parse([]string{
		"....",
		"....",
	})
	teams := []FleetTeam{
		{ID: 1, Priority: 1, Start: Point{0, 0}, Goal: Point{3, 0}},
		{ID: 2, Priority: 2, Start: Point{0, 1}, Goal: Point{3, 1}},
	}
	plan, err := g.FleetSchedule(teams, 10)
	if err != nil {
		t.Fatal(err)
	}
	// t=0: 两队各在 (0,0)/(0,1)；把 t=1 改成互相换边。
	plan.Windows[1].Seat[1] = Point{0, 1}
	plan.Windows[1].Seat[2] = Point{0, 0}
	c := plan.Verify()
	if len(c.SwapViolations) == 0 {
		t.Fatal("核验未能抓出对向换边冲突")
	}
	if c.OK() {
		t.Fatal("存在换边冲突时 OK 必须为假")
	}
}

// 原有单队接口保持不变：签名与错误行为回归。
func TestSingleTeamAPIUnchanged(t *testing.T) {
	g, _ := Parse([]string{"..#", "...", "#.."})
	if _, _, err := g.Path(Point{-1, 0}, Point{0, 0}); !errors.Is(err, ErrOOB) {
		t.Fatalf("Path 越界行为变化: %v", err)
	}
	if _, err := g.Visible(Point{1, 0}, Point{1, 2}); err != nil {
		t.Fatal(err)
	}
}

func assertWindowBound(t *testing.T, p *FleetPlan) {
	t.Helper()
	if len(p.Windows) > p.RequiredWindows() {
		t.Fatalf("实际窗数 %d 超过反解值 %d（冲突 %d）", len(p.Windows), p.RequiredWindows(), p.ConflictN)
	}
}

func codeIs(err error, code int) bool {
	var fe fleetErr
	if errors.As(err, &fe) {
		return fe.Code() == code
	}
	return false
}

// 收尾表：完成时间与冲突数。
func TestFleetSummaryTable(t *testing.T) {
	type spec struct {
		name  string
		rows  []string
		teams []FleetTeam
		bound int
	}
	cases := []spec{
		{"同向", []string{"..#..", ".....", "....."}, []FleetTeam{
			{ID: 1, Priority: 2, Start: Point{0, 0}, Goal: Point{4, 2}},
			{ID: 2, Priority: 1, Start: Point{0, 2}, Goal: Point{4, 0}},
		}, 15},
		{"相向", []string{"..#..", "..#..", ".....", "..#..", "..#.."}, []FleetTeam{
			{ID: 1, Priority: 2, Start: Point{0, 0}, Goal: Point{4, 4}},
			{ID: 2, Priority: 1, Start: Point{4, 4}, Goal: Point{0, 0}},
		}, 20},
		{"十字交叉", []string{"...", "...", "..."}, []FleetTeam{
			{ID: 1, Priority: 2, Start: Point{0, 1}, Goal: Point{2, 1}},
			{ID: 2, Priority: 1, Start: Point{1, 0}, Goal: Point{1, 2}},
		}, 10},
		{"狭窄走廊", []string{"......."}, []FleetTeam{
			{ID: 1, Priority: 1, Start: Point{0, 0}, Goal: Point{6, 0}},
			{ID: 2, Priority: 2, Start: Point{6, 0}, Goal: Point{0, 0}},
		}, 20},
		{"单队", []string{"...", ".#.", "..."}, []FleetTeam{
			{ID: 7, Priority: 1, Start: Point{0, 0}, Goal: Point{2, 2}},
		}, 10},
	}

	t.Log("\n完成时间与冲突数表：")
	t.Logf("%-8s %-8s %-10s %-10s %-12s %-10s %-8s",
		"用例", "队伍数", "完成时间", "冲突数", "实际窗数", "反解窗数", "核验")
	for _, c := range cases {
		g, _ := Parse(c.rows)
		plan, err := g.FleetSchedule(c.teams, c.bound)
		if err != nil {
			if codeIs(err, 4) {
				t.Logf("%-8s %-8d %-10s %-10s %-12s %-10s %-8s",
					c.name, len(c.teams), "无解(4)", "-", "-", "-", "预期无解")
				continue
			}
			t.Fatalf("%s 意外错误: %v", c.name, err)
		}
		check := plan.Verify()
		status := "通过"
		if !check.OK() {
			status = "失败"
		}
		t.Logf("%-8s %-8d %-10d %-10d %-12d %-10d %-8s",
			c.name, len(c.teams), plan.Makespan, plan.ConflictN,
			check.ActualWindows, check.NeedWindows, status)
		if !check.OK() {
			t.Fatalf("%s 核验失败: %+v", c.name, check)
		}
		fmt.Printf("[表] %s: 完成时间=%d 冲突数=%d 实际窗=%d 反解窗=%d\n",
			c.name, plan.Makespan, plan.ConflictN, check.ActualWindows, check.NeedWindows)
	}
}
