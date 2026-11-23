package grid

import "errors"

var (
	ErrShape  = errors.New("行宽不一致")
	ErrEmpty  = errors.New("网格为空")
	ErrOOB    = errors.New("坐标越界")
	ErrNoPath = errors.New("无可行路径")
)

type Point struct{ X, Y int }

type Grid struct {
	W, H  int
	cells [][]bool
}

func Parse(rows []string) (*Grid, error) {
	if len(rows) == 0 || len(rows[0]) == 0 {
		return nil, &CoordError{Err: ErrEmpty, P: Point{0, 0}}
	}
	w := len(rows[0])
	g := &Grid{W: w, H: len(rows), cells: make([][]bool, len(rows))}
	for y, row := range rows {
		if len(row) != w {
			// 报出该行第一个偏离首行宽度的列位置。
			return nil, &CoordError{Err: ErrShape, P: Point{w, y}}
		}
		g.cells[y] = make([]bool, w)
		for x := 0; x < w; x++ {
			g.cells[y][x] = row[x] != '#'
		}
	}
	return g, nil
}

func (g *Grid) Walkable(p Point) (bool, error) {
	if p.X < 0 || p.Y < 0 || p.X >= g.W || p.Y >= g.H {
		return false, &CoordError{Err: ErrOOB, P: p}
	}
	return g.cells[p.Y][p.X], nil
}

func (g *Grid) WalkableCount() int {
	n := 0
	for y := 0; y < g.H; y++ {
		for x := 0; x < g.W; x++ {
			if g.cells[y][x] {
				n++
			}
		}
	}
	return n
}
