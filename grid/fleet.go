package grid

import "fmt"

// 车队协同排程的错误编号（单队接口原有的 ErrShape/ErrEmpty/ErrOOB/ErrNoPath 保持不变）。
//
//	1: 队伍数为零
//	2: 网格为空
//	3: 起点重复
//	4: 排程无解（含对向堵死等几何不可行情形）
//	5: 总完成时间超过给定上界
var (
	ErrNoTeams    = fleetErr{code: 1, msg: "队伍数为零"}
	ErrEmptyGrid  = fleetErr{code: 2, msg: "网格为空"}
	ErrDupStart   = fleetErr{code: 3, msg: "起点重复"}
	ErrNoSchedule = fleetErr{code: 4, msg: "排程无解"}
	ErrOverBound  = fleetErr{code: 5, msg: "总完成时间超过上界"}
)

type fleetErr struct {
	code int
	msg  string
}

func (e fleetErr) Error() string { return fmt.Sprintf("[%d] %s", e.code, e.msg) }
func (e fleetErr) Code() int     { return e.code }

// FleetTeam 描述一支队伍：编号、优先级（数值越小越优先）、起点、终点。
type FleetTeam struct {
	ID       int
	Priority int
	Start    Point
	Goal     Point
}

// FleetWindow 表示第 t 个时间窗（t 为该窗起始时刻）各队所在格。
type FleetWindow struct {
	T    int
	Seat map[int]Point
}

// FleetPlan 是车队协同排程结果。
type FleetPlan struct {
	Teams     []FleetTeam
	Windows   []FleetWindow // 含 t=0 的初始窗
	Routes    map[int][]Point
	FinishAt  map[int]int // 各队首次抵达终点的时刻
	Conflicts map[int]int // 各队因避让产生的推迟（等待）次数
	ConflictN int         // 冲突总数
	Makespan  int         // 总完成时间（实际时间窗数 - 1）
	Bound     int         // 给定的完成时间上界
	baseSpan  int         // 无冲突时的基准窗跨度（最长单队路径）
}

// RequiredWindows 由冲突数反解需要的时间窗数（含初始窗）：
//
//	需要窗数 = 基准跨度 + 冲突总数 + 1
//
// 每发生一次冲突，低优先级队伍推迟一个时间窗，故实际窗数不会超过该反解值。
func (p *FleetPlan) RequiredWindows() int {
	return p.baseSpan + p.ConflictN + 1
}

type reservation struct {
	seat map[int]Point // 时刻 -> 该队占用格
}

// FleetSchedule 按时间窗逐轮规划多支队伍。
//
// 规划流程：
//  1. 校验输入（三类非法输入分别给出编号 1/2/3）；
//  2. 复用单队 Path 接口求各队静态最短路径；
//  3. 按优先级逐队、逐时间窗规划：只允许沿静态路径前进或原地等待，
//     与已确定的高优先级队伍在同一时刻争用同一格（含对向换边）即判为冲突，
//     按优先级让低优先级队伍推迟一个时间窗；
//  4. 排不出则报编号 4，完成时间超上界则报编号 5。
//
// 原有单队接口（Parse/Path/Visible）签名与行为均不变。
func (g *Grid) FleetSchedule(teams []FleetTeam, bound int) (*FleetPlan, error) {
	if len(teams) == 0 {
		return nil, ErrNoTeams
	}
	if g == nil || g.W == 0 || g.H == 0 || g.WalkableCount() == 0 {
		return nil, ErrEmptyGrid
	}
	seen := map[Point]bool{}
	for _, tm := range teams {
		if seen[tm.Start] {
			return nil, ErrDupStart
		}
		seen[tm.Start] = true
	}
	if bound < 0 {
		return nil, fmt.Errorf("上界非法: %d", bound)
	}

	ordered := make([]FleetTeam, len(teams))
	copy(ordered, teams)
	sortTeams(ordered)

	// 各队静态最短路径（复用单队接口）。
	static := map[int][]Point{}
	baseSpan := 0
	for _, tm := range ordered {
		path, _, err := g.Path(tm.Start, tm.Goal)
		if err != nil {
			return nil, fmt.Errorf("%w: 队伍 %d: %v", ErrNoSchedule, tm.ID, err)
		}
		static[tm.ID] = path
		if len(path)-1 > baseSpan {
			baseSpan = len(path) - 1
		}
	}

	// 高优先级队伍的时空预留表（每队一条，按规划顺序累积）。
	res := map[int]*reservation{}
	routes := map[int][]Point{}
	finishAt := map[int]int{}
	conflicts := map[int]int{}
	conflictN := 0
	makespan := 0

	// 安全时限独立于给定上界：仅用于判定几何可行性（堵死时报编号 4），
	// 排得出来但超出上界的情形在全部规划完成后报编号 5。
	horizon := bound + g.WalkableCount() + len(teams)

	for _, tm := range ordered {
		route, waits, finish, err := planOne(g, tm, static[tm.ID], res, horizon)
		if err != nil {
			return nil, err
		}
		routes[tm.ID] = route
		finishAt[tm.ID] = finish
		conflicts[tm.ID] = waits
		conflictN += waits
		if finish > makespan {
			makespan = finish
		}
		r := &reservation{seat: make(map[int]Point, len(route))}
		for t, p := range route {
			r.seat[t] = p
		}
		res[tm.ID] = r
	}

	if makespan > bound {
		return nil, fmt.Errorf("%w: 完成时间 %d > 上界 %d", ErrOverBound, makespan, bound)
	}

	windows := make([]FleetWindow, makespan+1)
	for t := 0; t <= makespan; t++ {
		w := FleetWindow{T: t, Seat: map[int]Point{}}
		for _, tm := range ordered {
			route := routes[tm.ID]
			if t < len(route) {
				w.Seat[tm.ID] = route[t]
			} else {
				w.Seat[tm.ID] = route[len(route)-1] // 已完成的队伍停在终点
			}
		}
		windows[t] = w
	}

	return &FleetPlan{
		Teams:     ordered,
		Windows:   windows,
		Routes:    routes,
		FinishAt:  finishAt,
		Conflicts: conflicts,
		ConflictN: conflictN,
		Makespan:  makespan,
		Bound:     bound,
		baseSpan:  baseSpan,
	}, nil
}

// planOne 为单支队伍做时间维 BFS：沿静态路径只前进或等待，避让全部高优先级预留。
// 返回逐时刻位置序列（含 t=0）、等待（冲突推迟）次数、首次抵达终点时刻。
// horizon 为搜索安全时限，仅用于几何可行性判定，与完成时间上界无关。
func planOne(g *Grid, tm FleetTeam, path []Point, res map[int]*reservation, horizon int) ([]Point, int, int, error) {
	type state struct {
		step, pos int // step: 静态路径上的步号，pos: 当前时刻
	}
	start := state{step: 0, pos: 0}
	visited := map[state]bool{start: true}
	prev := map[state]state{}
	queue := []state{start}
	var goal state
	found := false

	for len(queue) > 0 {
		cur := queue[0]
		queue = queue[1:]
		if cur.pos > horizon {
			continue
		}
		if cur.step == len(path)-1 {
			goal, found = cur, true
			break
		}
		t := cur.pos + 1
		// 动作一：原地等待（推迟一个时间窗）。
		wait := state{step: cur.step, pos: t}
		if !visited[wait] && legalSeat(res, path[cur.step], t) {
			visited[wait] = true
			prev[wait] = cur
			queue = append(queue, wait)
		}
		// 动作二：沿静态路径前进一步；同时检查对向换边冲突。
		next := state{step: cur.step + 1, pos: t}
		if !visited[next] && legalSeat(res, path[next.step], t) &&
			legalSwap(res, path[cur.step], path[next.step], t) {
			visited[next] = true
			prev[next] = cur
			queue = append(queue, next)
		}
	}
	if !found {
		return nil, 0, 0, fmt.Errorf("%w: 队伍 %d 在安全时限 %d 内无法避让冲突", ErrNoSchedule, tm.ID, horizon)
	}

	// 回溯状态链，展开成逐时刻位置序列。
	var chain []state
	for s := goal; s != start; s = prev[s] {
		chain = append([]state{s}, chain...)
	}
	route := []Point{path[0]}
	for _, s := range chain {
		route = append(route, path[s.step])
	}
	// 时刻数减去静态路径步数，即为等待（冲突推迟）次数。
	waits := len(route) - len(path)
	return route, waits, goal.pos, nil
}

// legalSeat 检查在时刻 t 占据 cell 是否与任何高优先级队伍同格冲突。
func legalSeat(res map[int]*reservation, cell Point, t int) bool {
	for _, r := range res {
		if p, ok := r.seat[t]; ok && p == cell {
			return false
		}
	}
	return true
}

// legalSwap 检查对向换边冲突：时刻 t-1 位于 next、时刻 t 退到 cur 的高优先级队伍，
// 会与本队 cur->next 的移动在同一时间窗内对穿。
func legalSwap(res map[int]*reservation, cur, next Point, t int) bool {
	for _, r := range res {
		if p0, ok := r.seat[t-1]; ok && p0 == next {
			if p1, ok := r.seat[t]; ok && p1 == cur {
				return false
			}
		}
	}
	return true
}

func sortTeams(teams []FleetTeam) {
	for i := 1; i < len(teams); i++ {
		for j := i; j > 0; j-- {
			a, b := teams[j-1], teams[j]
			if a.Priority > b.Priority ||
				(a.Priority == b.Priority && a.ID > b.ID) {
				teams[j-1], teams[j] = b, a
			} else {
				break
			}
		}
	}
}

// FleetCheck 对已生成的排程做两点逐一核验，并补充换边冲突核验。
type FleetCheck struct {
	SeatViolations []string // 同一时刻占据同一格的冲突描述
	SwapViolations []string // 同一时间窗对向换边冲突描述
	OverBound      bool     // 总完成时间是否超过上界
	WindowOverflow bool     // 实际窗数是否超过由冲突数反解的窗数
	ActualWindows  int      // 实际时间窗数（含初始窗）
	NeedWindows    int      // 反解需要的时间窗数
}

func (c *FleetCheck) OK() bool {
	return len(c.SeatViolations) == 0 && len(c.SwapViolations) == 0 &&
		!c.OverBound && !c.WindowOverflow
}

// Verify 逐时刻、逐队核验排程：
//  1. 同一时刻任意两队不得占据同一格（两点逐一比对）；
//  2. 同一时间窗内不得对向换边；
//  3. 总完成时间不得超过给定上界；
//  4. 实际时间窗数不得超过由冲突数反解的值。
func (p *FleetPlan) Verify() *FleetCheck {
	c := &FleetCheck{
		ActualWindows:  len(p.Windows),
		NeedWindows:    p.RequiredWindows(),
		OverBound:      p.Makespan > p.Bound,
		WindowOverflow: len(p.Windows) > p.RequiredWindows(),
	}
	ids := make([]int, len(p.Teams))
	for i, tm := range p.Teams {
		ids[i] = tm.ID
	}
	// 核验点一：同一时刻同格冲突，所有队伍两两逐一核验。
	for _, w := range p.Windows {
		for i := 0; i < len(ids); i++ {
			for j := i + 1; j < len(ids); j++ {
				if w.Seat[ids[i]] == w.Seat[ids[j]] {
					c.SeatViolations = append(c.SeatViolations,
						fmt.Sprintf("t=%d 队伍 %d 与 %d 同占 %v", w.T, ids[i], ids[j], w.Seat[ids[i]]))
				}
			}
		}
	}
	// 补充核验：同一时间窗对向换边。
	for t := 1; t < len(p.Windows); t++ {
		prev, cur := p.Windows[t-1].Seat, p.Windows[t].Seat
		for i := 0; i < len(ids); i++ {
			for j := i + 1; j < len(ids); j++ {
				a, b := ids[i], ids[j]
				if prev[a] == cur[b] && prev[b] == cur[a] && cur[a] != cur[b] {
					c.SwapViolations = append(c.SwapViolations,
						fmt.Sprintf("t=%d 队伍 %d 与 %d 对向换边 %v<->%v", t, a, b, prev[a], prev[b]))
				}
			}
		}
	}
	return c
}
