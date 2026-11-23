package grid

import (
	"errors"
	"fmt"
)

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
		return nil, ErrEmpty
	}
	w := len(rows[0])
	g := &Grid{W: w, H: len(rows), cells: make([][]bool, len(rows))}
	for y, row := range rows {
		if len(row) != w {
			return nil, ErrShape
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
		return false, ErrOOB
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

// Set 直接修改某格的可行走状态（门的开/关）。越界返回带坐标的 ErrOOB。
func (g *Grid) Set(p Point, walkable bool) error {
	if p.X < 0 || p.Y < 0 || p.X >= g.W || p.Y >= g.H {
		return fmt.Errorf("%w: 坐标 (%d,%d) 不在 %dx%d 网格内", ErrOOB, p.X, p.Y, g.W, g.H)
	}
	g.cells[p.Y][p.X] = walkable
	return nil
}

// Clone 深拷贝网格，用于基准对照与计时。
func (g *Grid) Clone() *Grid {
	cells := make([][]bool, g.H)
	for y := range g.cells {
		cells[y] = append([]bool(nil), g.cells[y]...)
	}
	return &Grid{W: g.W, H: g.H, cells: cells}
}
