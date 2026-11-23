package grid

import "testing"

func TestParseAndCount(t *testing.T) {
	g, err := Parse([]string{"..#", "...", "#.."})
	if err != nil {
		t.Fatal(err)
	}
	if g.W != 3 || g.H != 3 || g.WalkableCount() != 7 {
		t.Fatalf("解析错误 %d %d %d", g.W, g.H, g.WalkableCount())
	}
	if _, err := Parse([]string{"..", "..."}); err == nil {
		t.Fatal("行宽不一致应报错")
	}
	if _, err := Parse(nil); err == nil {
		t.Fatal("空网格应报错")
	}
}

func TestPathAndBlocked(t *testing.T) {
	g, _ := Parse([]string{"...", ".#.", "..."})
	path, visited, err := g.Path(Point{0, 0}, Point{2, 2})
	if err != nil {
		t.Fatal(err)
	}
	if len(path) != 5 || visited == 0 {
		t.Fatalf("路径异常 %d %d", len(path), visited)
	}
	if _, _, err := g.Path(Point{0, 0}, Point{1, 1}); err == nil {
		t.Fatal("目标不可走应报错")
	}
	if _, _, err := g.Path(Point{-1, 0}, Point{0, 0}); err == nil {
		t.Fatal("越界应报错")
	}
}

func TestVisibleThroughWall(t *testing.T) {
	g, _ := Parse([]string{"...", ".#.", "..."})
	ok, err := g.Visible(Point{0, 0}, Point{1, 0})
	if err != nil || !ok {
		t.Fatal("相邻可见")
	}
	blocked, err := g.Visible(Point{1, 0}, Point{1, 2})
	if err != nil {
		t.Fatal(err)
	}
	if blocked {
		t.Fatal("穿过墙体应不可见")
	}
}
