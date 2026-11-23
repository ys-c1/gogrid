package grid

import (
	"errors"
	"fmt"
)

var (
	ErrShape          = errors.New("行宽不一致")
	ErrEmpty          = errors.New("网格为空")
	ErrOOB            = errors.New("坐标越界")
	ErrNoPath         = errors.New("无可行路径")
	ErrNegativeWeight = errors.New("权重为负")
	ErrBlocked        = errors.New("格点不可走")
)

type Point struct{ X, Y int }

func (p Point) String() string {
	return fmt.Sprintf("(%d,%d)", p.X, p.Y)
}

type Grid struct {
	W, H    int
	cells   [][]bool
	weights [][]int
}

func Parse(rows []string) (*Grid, error) {
	if len(rows) == 0 || len(rows[0]) == 0 {
		return nil, ErrEmpty
	}
	w := len(rows[0])
	g := &Grid{W: w, H: len(rows), cells: make([][]bool, len(rows)), weights: make([][]int, len(rows))}
	for y, row := range rows {
		if len(row) != w {
			return nil, ErrShape
		}
		g.cells[y] = make([]bool, w)
		g.weights[y] = make([]int, w)
		for x := 0; x < w; x++ {
			// 可走格默认等权 1，墙体权重为 0（不参与寻路）
			g.cells[y][x] = row[x] != '#'
			if g.cells[y][x] {
				g.weights[y][x] = 1
			}
		}
	}
	return g, nil
}

func (g *Grid) Walkable(p Point) (bool, error) {
	if p.X < 0 || p.Y < 0 || p.X >= g.W || p.Y >= g.H {
		return false, fmt.Errorf("%w: 坐标%s", ErrOOB, p)
	}
	return g.cells[p.Y][p.X], nil
}

// Weight 返回格点地形权重（进入该格的代价）。
func (g *Grid) Weight(p Point) (int, error) {
	if p.X < 0 || p.Y < 0 || p.X >= g.W || p.Y >= g.H {
		return 0, fmt.Errorf("%w: 坐标%s", ErrOOB, p)
	}
	return g.weights[p.Y][p.X], nil
}

// SetWeight 设置地形权重，负权重、越界、墙体均返回带坐标的错误。
func (g *Grid) SetWeight(p Point, w int) error {
	if g == nil || g.H == 0 || g.W == 0 {
		return fmt.Errorf("%w: 坐标%s", ErrEmpty, p)
	}
	if p.X < 0 || p.Y < 0 || p.X >= g.W || p.Y >= g.H {
		return fmt.Errorf("%w: 坐标%s", ErrOOB, p)
	}
	if w < 0 {
		return fmt.Errorf("%w: 坐标%s 权重=%d", ErrNegativeWeight, p, w)
	}
	if !g.cells[p.Y][p.X] {
		return fmt.Errorf("%w: 坐标%s 为墙体", ErrBlocked, p)
	}
	g.weights[p.Y][p.X] = w
	return nil
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
