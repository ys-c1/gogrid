package grid

import (
	"math/rand"
	"testing"
)

// scenarioMap 构造带门的房间地图：竖墙 x=wallX，门位于 (wallX, doorY)。
// doorOpen=true 时门是开的（墙在此处留缺口）。
func scenarioMap(w, h, wallX, doorY int, doorOpen bool) []string {
	rows := make([]string, h)
	for y := 0; y < h; y++ {
		b := make([]byte, w)
		for x := 0; x < w; x++ {
			b[x] = '.'
			if x == wallX && !(y == doorY && doorOpen) {
				b[x] = '#'
			}
		}
		rows[y] = string(b)
	}
	return rows
}

// assertEqualFull 逐格对照增量距离场与全量重算结果。
func assertEqualFull(t *testing.T, f *Field, tag string) {
	t.Helper()
	full, _ := f.FullRecompute()
	got := f.Distances()
	if len(got) != len(full) {
		t.Fatalf("[%s] 距离场长度不一致 %d != %d", tag, len(got), len(full))
	}
	for i, v := range got {
		if v != full[i] {
			p := Point{X: i % f.g.W, Y: i / f.g.W}
			t.Fatalf("[%s] 格子 (%d,%d) 增量结果 %d 与全量重算 %d 不一致", tag, p.X, p.Y, v, full[i])
		}
	}
}

// assertPathConsistent 距离场路径长度必须等于起点距离，且与静态 A* 等长。
func assertPathConsistent(t *testing.T, f *Field, from Point, tag string) {
	t.Helper()
	p, err := f.Path(from)
	if err != nil {
		t.Fatalf("[%s] 距离场寻路失败: %v", tag, err)
	}
	want, err := f.Dist(from)
	if err != nil {
		t.Fatalf("[%s] 距离查询失败: %v", tag, err)
	}
	if len(p)-1 != want {
		t.Fatalf("[%s] 路径步数 %d 与距离场 %d 不符", tag, len(p)-1, want)
	}
	ap, _, aerr := f.g.Path(from, f.goal)
	if aerr != nil {
		t.Fatalf("[%s] 静态 A* 失败: %v", tag, aerr)
	}
	if len(ap) != len(p) {
		t.Fatalf("[%s] 距离场路径 %d 步与静态 A* %d 步不等长", tag, len(p)-1, len(ap)-1)
	}
}

func TestInitialFieldIsFullBFS(t *testing.T) {
	g, _ := Parse(scenarioMap(9, 9, 4, 4, true))
	f, err := NewField(g, Point{8, 8})
	if err != nil {
		t.Fatal(err)
	}
	assertEqualFull(t, f, "初始距离场")
}

// TestFixedScenarios 覆盖五个指定用例：开门、关门、连续两次变化、起点被堵、终点被堵。
func TestFixedScenarios(t *testing.T) {
	const W, H, wallX, doorY = 9, 9, 4, 4
	door := Point{wallX, doorY}
	start := Point{1, 1}
	goal := Point{8, 8}

	t.Run("开门", func(t *testing.T) {
		// 门初始关闭，开门后左侧房间恢复到终点的路径。
		g, _ := Parse(scenarioMap(W, H, wallX, doorY, false))
		f, err := NewField(g, goal)
		if err != nil {
			t.Fatal(err)
		}
		st, err := f.Update([]Change{{P: door, Walkable: true}})
		if err != nil {
			t.Fatal(err)
		}
		assertEqualFull(t, f, "开门")
		assertPathConsistent(t, f, start, "开门")
		if st.Touched > st.FullCount {
			t.Fatalf("重算格数 %d 超过全量 %d", st.Touched, st.FullCount)
		}
		t.Logf("开门: 重算 %d / 全量 %d = %.1f%%", st.Touched, st.FullCount, 100*st.Ratio())
	})

	t.Run("关门", func(t *testing.T) {
		// 门初始打开，关门后 raise 波重算受影响区域。
		g, _ := Parse(scenarioMap(W, H, wallX, doorY, true))
		f, _ := NewField(g, goal)
		st, err := f.Update([]Change{{P: door, Walkable: false}})
		if err != nil {
			t.Fatal(err)
		}
		assertEqualFull(t, f, "关门")
		// 唯一通道被切断，起点应不可达。
		if _, err := f.Path(start); err == nil {
			t.Fatal("关门后起点应不可达")
		}
		if st.Touched > st.FullCount {
			t.Fatalf("重算格数 %d 超过全量 %d", st.Touched, st.FullCount)
		}
		t.Logf("关门: 重算 %d / 全量 %d = %.1f%%", st.Touched, st.FullCount, 100*st.Ratio())
	})

	t.Run("连续两次变化", func(t *testing.T) {
		g, _ := Parse(scenarioMap(W, H, wallX, doorY, true))
		f, _ := NewField(g, goal)
		st1, err := f.Update([]Change{{P: door, Walkable: false}})
		if err != nil {
			t.Fatal(err)
		}
		assertEqualFull(t, f, "第一次变化-关门")
		// 第二次变化：开门，且同时变动另一格，验证连续调用与批量混合。
		other := Point{wallX, doorY + 1}
		st2, err := f.Update([]Change{
			{P: door, Walkable: true},
			{P: other, Walkable: false},
		})
		if err != nil {
			t.Fatal(err)
		}
		assertEqualFull(t, f, "第二次变化-开门+堵格")
		assertPathConsistent(t, f, start, "连续两次变化")
		t.Logf("连续变化①: %d/%d=%.1f%%  ②: %d/%d=%.1f%%",
			st1.Touched, st1.FullCount, 100*st1.Ratio(),
			st2.Touched, st2.FullCount, 100*st2.Ratio())
	})

	t.Run("起点被堵", func(t *testing.T) {
		g, _ := Parse(scenarioMap(W, H, wallX, doorY, true))
		f, _ := NewField(g, goal)
		st, err := f.Update([]Change{{P: start, Walkable: false}})
		if err != nil {
			t.Fatal(err)
		}
		assertEqualFull(t, f, "起点被堵")
		if _, err := f.Path(start); err == nil {
			t.Fatal("起点被堵后应报错并带坐标")
		} else {
			t.Logf("起点被堵寻路报错（预期）: %v", err)
		}
		// 单格关闭只影响局部，应明显低于三成预算。
		if st.OverBudget() {
			t.Fatalf("起点被堵重算比例 %.1f%% 超过 30%%", 100*st.Ratio())
		}
		t.Logf("起点被堵: 重算 %d / 全量 %d = %.1f%%", st.Touched, st.FullCount, 100*st.Ratio())
	})

	t.Run("终点被堵", func(t *testing.T) {
		g, _ := Parse(scenarioMap(W, H, wallX, doorY, true))
		f, _ := NewField(g, goal)
		st, err := f.Update([]Change{{P: goal, Walkable: false}})
		if err != nil {
			t.Fatal(err)
		}
		assertEqualFull(t, f, "终点被堵")
		if _, err := f.Path(start); err == nil {
			t.Fatal("终点被堵后所有格子均不可达，应报错")
		} else {
			t.Logf("终点被堵寻路报错（预期）: %v", err)
		}
		t.Logf("终点被堵: 重算 %d / 全量 %d = %.1f%%（整域失效，允许超三成）",
			st.Touched, st.FullCount, 100*st.Ratio())

		// 终点重新打开：lower 波必须重建完整距离场。
		st2, err := f.Update([]Change{{P: goal, Walkable: true}})
		if err != nil {
			t.Fatal(err)
		}
		assertEqualFull(t, f, "终点恢复")
		assertPathConsistent(t, f, start, "终点恢复")
		t.Logf("终点恢复: 重算 %d / 全量 %d = %.1f%%", st2.Touched, st2.FullCount, 100*st2.Ratio())
	})
}

// TestErrorReportsCoordinates 越界、空网格、起点不可走分别验证错误中带坐标。
func TestErrorReportsCoordinates(t *testing.T) {
	g, _ := Parse(scenarioMap(9, 9, 4, 4, true))
	f, err := NewField(g, Point{8, 8})
	if err != nil {
		t.Fatal(err)
	}

	// 变化区域越界。
	if _, err := f.Update([]Change{{P: Point{9, 0}, Walkable: false}}); err == nil {
		t.Fatal("越界变化应报错")
	} else {
		t.Logf("越界变化报错（预期）: %v", err)
	}
	if _, err := f.Update([]Change{{P: Point{0, -1}, Walkable: true}}); err == nil {
		t.Fatal("负坐标应报错")
	}

	// 网格为空。
	if _, err := NewField(&Grid{W: 0, H: 0}, Point{0, 0}); err == nil {
		t.Fatal("空网格应报错")
	} else {
		t.Logf("空网格报错（预期）: %v", err)
	}

	// 起点不可走（先堵上再寻路）。
	bad := Point{2, 2}
	if _, err := f.Update([]Change{{P: bad, Walkable: false}}); err != nil {
		t.Fatal(err)
	}
	if _, err := f.Path(bad); err == nil {
		t.Fatal("起点不可走应报错并带坐标")
	} else {
		t.Logf("起点不可走报错（预期）: %v", err)
	}

	// 终点初始不可走。
	if _, err := NewField(g, Point{4, 0}); err == nil { // (4,0) 是墙
		t.Fatal("终点不可走应报错并带坐标")
	}
}

// TestRandomIncrementalCorrectness 随机压测：
// 每次随机变动后，增量距离场必须与全量重算逐格相同；
// 距离场路径必须与静态 A* 路径等长；重算格数不得超过全量规模。
func TestRandomIncrementalCorrectness(t *testing.T) {
	rng := rand.New(rand.NewSource(20261124))
	sizes := [][2]int{{6, 6}, {10, 8}, {15, 12}}
	totalRuns := 0
	for _, sz := range sizes {
		w, h := sz[0], sz[1]
		for iter := 0; iter < 60; iter++ {
			// 随机障碍密度 0%~30%，保证通常仍有连通区域。
			rows := make([]string, h)
			for y := 0; y < h; y++ {
				b := make([]byte, w)
				for x := 0; x < w; x++ {
					if rng.Float64() < 0.25 {
						b[x] = '#'
					} else {
						b[x] = '.'
					}
				}
				rows[y] = string(b)
			}
			g, err := Parse(rows)
			if err != nil {
				t.Fatal(err)
			}
			// 随机选一个可行走终点。
			var goal Point
			found := false
			for try := 0; try < 100; try++ {
				goal = Point{rng.Intn(w), rng.Intn(h)}
				if g.cells[goal.Y][goal.X] {
					found = true
					break
				}
			}
			if !found {
				continue
			}
			f, err := NewField(g, goal)
			if err != nil {
				t.Fatal(err)
			}
			// 连续 25 步随机变化（单格或批量）。
			for step := 0; step < 25; step++ {
				n := 1 + rng.Intn(3)
				changes := make([]Change, n)
				for i := range changes {
					p := Point{rng.Intn(w), rng.Intn(h)}
					changes[i] = Change{P: p, Walkable: !g.cells[p.Y][p.X]}
				}
				before := f.Distances() // 快照更新前距离场，用于紧致性断言
				st, err := f.Update(changes)
				if err != nil {
					t.Fatalf("随机更新失败: %v", err)
				}
				if st.Touched > st.FullCount && st.FullCount > 0 {
					t.Fatalf("重算 %d 超过全量 %d", st.Touched, st.FullCount)
				}
				// 逐格不变量 + 重算区紧致性。
				full, _ := f.FullRecompute()
				got := f.Distances()
				realChanged := 0
				for i, v := range got {
					if v != full[i] {
						p := Point{i % w, i / w}
						t.Fatalf("随机用例 %dx%d iter=%d step=%d 格子(%d,%d): 增量 %d != 全量 %d",
							w, h, iter, step, p.X, p.Y, v, full[i])
					}
					// 紧致性下界：任何距离真正变化的可行走格都必须被计入重算，
					// 否则意味着该格被错误地"原值复用"。
					if before[i] != full[i] && g.cells[i/w][i%w] {
						realChanged++
						if f.touchedMark[i] != f.tick {
							p := Point{i % w, i / w}
							t.Fatalf("重算遗漏: 格(%d,%d) 距离 %d→%d 却未计入 touched",
								p.X, p.Y, before[i], full[i])
						}
					}
				}
				if st.Touched < realChanged {
					t.Fatalf("重算格数 %d 小于实际变化格数 %d", st.Touched, realChanged)
				}
				// 路径等长不变量（对所有可行走起点抽样检查）。
				for sample := 0; sample < 5; sample++ {
					from := Point{rng.Intn(w), rng.Intn(h)}
					if !g.cells[from.Y][from.X] {
						continue
					}
					p, perr := f.Path(from)
					ap, _, aerr := g.Path(from, goal)
					if (perr == nil) != (aerr == nil) {
						t.Fatalf("可达性分歧: 距离场 err=%v A* err=%v from=(%d,%d)", perr, aerr, from.X, from.Y)
					}
					if perr == nil && len(p) != len(ap) {
						t.Fatalf("路径长度分歧: 距离场 %d A* %d from=(%d,%d)", len(p), len(ap), from.X, from.Y)
					}
				}
				totalRuns++
			}
		}
	}
	t.Logf("随机压测完成，累计更新 %d 次，全部逐格一致", totalRuns)
}
