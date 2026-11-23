// 命令 replan 汇报动态重规划在两档网格、五种变化用例下的表现：
// 重算格数、重算比例（三成预算）、增量与全量重算的耗时对照。
package main

import (
	"fmt"
	"time"

	"gogrid/grid"
)

// level 描述一档网格：大主区 + 右下角带门的小房间。
// 门的开/关只影响小房间这一局部，主区距离场可整体复用。
type level struct {
	name         string
	w, h         int
	roomL, roomR int // 小房间墙的左右列（含墙）
	roomT, roomB int // 小房间墙的上下行（含墙）
	door         grid.Point
	goal         grid.Point
	start        grid.Point
}

// roomMap 生成"主区 + 带门小房间"地图，doorOpen 决定门初始开关。
// 小房间由 roomL/roomR/roomT/roomB 四面墙围合，门开在左墙中部。
func roomMap(lv level, doorOpen bool) []string {
	wall := func(x, y int) bool {
		if x != lv.roomL && x != lv.roomR && y != lv.roomT && y != lv.roomB {
			return false
		}
		// 落在房间矩形外的不算墙。
		if x < lv.roomL || x > lv.roomR || y < lv.roomT || y > lv.roomB {
			return false
		}
		return true
	}
	rows := make([]string, lv.h)
	for y := 0; y < lv.h; y++ {
		b := make([]byte, lv.w)
		for x := 0; x < lv.w; x++ {
			b[x] = '.'
			if wall(x, y) {
				if doorOpen && x == lv.door.X && y == lv.door.Y {
					b[x] = '.'
				} else {
					b[x] = '#'
				}
			}
		}
		rows[y] = string(b)
	}
	return rows
}

// result 汇总一个用例的一次重规划汇报。
type result struct {
	caseName   string
	touched    int
	full       int
	ratio      float64
	overBudget bool
	incrTime   time.Duration
	fullTime   time.Duration
	note       string
}

func main() {
	// 两档网格：小房间占比都明显小于三成，门变化应落在预算内。
	levels := []level{
		{
			name: "小网格 25x25", w: 25, h: 25,
			roomL: 18, roomR: 23, roomT: 18, roomB: 23,
			door:  grid.Point{X: 18, Y: 20},
			goal:  grid.Point{X: 2, Y: 2},
			start: grid.Point{X: 1, Y: 1},
		},
		{
			name: "大网格 61x61", w: 61, h: 61,
			roomL: 46, roomR: 58, roomT: 46, roomB: 58,
			door:  grid.Point{X: 46, Y: 52},
			goal:  grid.Point{X: 3, Y: 3},
			start: grid.Point{X: 1, Y: 1},
		},
	}

	all := map[string][]result{}
	for _, lv := range levels {
		all[lv.name] = runLevel(lv)
	}

	for _, lv := range levels {
		fmt.Printf("================ %s ================\n", lv.name)
		fmt.Printf("%-14s %8s %8s %9s %12s %12s  %s\n",
			"用例", "重算格", "全量格", "重算比例", "增量耗时", "全量耗时", "预算/备注")
		for _, r := range all[lv.name] {
			flag := "达标(≤30%)"
			if r.overBudget {
				flag = "超三成: " + r.note
			} else if r.note != "" {
				flag = r.note
			}
			fmt.Printf("%-14s %8d %8d %8.1f%% %12s %12s  %s\n",
				r.caseName, r.touched, r.full, 100*r.ratio,
				r.incrTime.Round(time.Microsecond), r.fullTime.Round(time.Microsecond), flag)
		}
		fmt.Println()
	}

	// 收尾：重算比例汇总表。
	fmt.Println("================ 重算比例汇总表 ================")
	header := fmt.Sprintf("%-14s", "用例")
	for _, lv := range levels {
		header += fmt.Sprintf(" | %-18s", lv.name)
	}
	fmt.Println(header)
	cases := []string{"开门", "关门", "连续两次变化", "起点被堵", "终点被堵"}
	for _, cn := range cases {
		line := fmt.Sprintf("%-14s", cn)
		for _, lv := range levels {
			for _, r := range all[lv.name] {
				if r.caseName == cn {
					mark := ""
					if r.overBudget {
						mark = " *超预算"
					}
					line += fmt.Sprintf(" | %9.1f%%%9s", 100*r.ratio, mark)
				}
			}
		}
		fmt.Println(line)
	}
	fmt.Println("\n* 标记表示重算比例超过三成预算，对应行已注明原因并给出全量重算对照时间。")

	// 单独再列一张超预算项的原因与全量对照表，便于收尾核对。
	anyOver := false
	for _, lv := range levels {
		for _, r := range all[lv.name] {
			if r.overBudget {
				if !anyOver {
					fmt.Println("\n================ 超预算项原因与全量对照 ================")
					anyOver = true
				}
				fmt.Printf("[%s] %s: 比例 %.1f%%。原因: %s。全量重算对照耗时 %s（增量 %s）。\n",
					lv.name, r.caseName, 100*r.ratio, r.note,
					r.fullTime.Round(time.Microsecond), r.incrTime.Round(time.Microsecond))
			}
		}
	}
}

// bench 对同一初始状态分别计时增量更新与全量重算。
// 通过克隆保证两条路径从完全相同的网格/距离场出发；多轮取平均降低噪声。
func bench(base *grid.Field, changes []grid.Change) (grid.Stats, time.Duration, time.Duration) {
	const rounds = 100
	var incrTotal, fullTotal time.Duration
	var st grid.Stats
	for k := 0; k < rounds; k++ {
		f := base.Clone()
		t0 := time.Now()
		s, err := f.Update(copyChanges(changes))
		if err != nil {
			panic(err)
		}
		incrTotal += time.Since(t0)
		st = s
	}
	for k := 0; k < rounds; k++ {
		f := base.Clone()
		if _, err := f.Update(copyChanges(changes)); err != nil {
			panic(err)
		}
		t0 := time.Now()
		_, _ = f.FullRecompute()
		fullTotal += time.Since(t0)
	}
	return st, incrTotal / rounds, fullTotal / rounds
}

// copyChanges 防御性拷贝变化切片，避免别名修改。
func copyChanges(c []grid.Change) []grid.Change {
	out := make([]grid.Change, len(c))
	copy(out, c)
	return out
}

func runLevel(lv level) []result {
	door := lv.door
	var out []result

	// 用例 1：开门（门初始关闭，小房间与主区断开）。
	{
		g, _ := grid.Parse(roomMap(lv, false))
		f, _ := grid.NewField(g, lv.goal)
		st, inc, full := bench(f, []grid.Change{{P: door, Walkable: true}})
		out = append(out, result{"开门", st.Touched, st.FullCount, st.Ratio(), st.OverBudget(), inc, full, ""})
	}

	// 用例 2：关门（门初始打开，关门后仅小房间内部失效）。
	{
		g, _ := grid.Parse(roomMap(lv, true))
		f, _ := grid.NewField(g, lv.goal)
		st, inc, full := bench(f, []grid.Change{{P: door, Walkable: false}})
		out = append(out, result{"关门", st.Touched, st.FullCount, st.Ratio(), st.OverBudget(), inc, full, ""})
	}

	// 用例 3：连续两次变化（先关门，再开门并同时堵小房间内另一格）。
	{
		g, _ := grid.Parse(roomMap(lv, true))
		f, _ := grid.NewField(g, lv.goal)
		other := grid.Point{X: (lv.roomL + lv.roomR) / 2, Y: (lv.roomT + lv.roomB) / 2}
		pre := f.Clone()
		if _, err := pre.Update([]grid.Change{{P: door, Walkable: false}}); err != nil {
			panic(err)
		}
		st, inc, full := bench(pre, []grid.Change{
			{P: door, Walkable: true},
			{P: other, Walkable: false},
		})
		out = append(out, result{"连续两次变化", st.Touched, st.FullCount, st.Ratio(), st.OverBudget(), inc, full, ""})
	}

	// 用例 4：起点被堵（单格局部失效）。
	{
		g, _ := grid.Parse(roomMap(lv, true))
		f, _ := grid.NewField(g, lv.goal)
		st, inc, full := bench(f, []grid.Change{{P: lv.start, Walkable: false}})
		out = append(out, result{"起点被堵", st.Touched, st.FullCount, st.Ratio(), st.OverBudget(), inc, full,
			"单格局部失效"})
	}

	// 用例 5：终点被堵（整域失效，必然超三成，需说明原因并给全量对照时间）。
	{
		g, _ := grid.Parse(roomMap(lv, true))
		f, _ := grid.NewField(g, lv.goal)
		st, inc, full := bench(f, []grid.Change{{P: lv.goal, Walkable: false}})
		note := fmt.Sprintf("终点被堵使全图失去根，%d 个可行走格全部不可达，无局部可复用区域", st.FullCount)
		out = append(out, result{"终点被堵", st.Touched, st.FullCount, st.Ratio(), st.OverBudget(), inc, full, note})
	}

	return out
}
