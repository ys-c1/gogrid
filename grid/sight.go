package grid

func (g *Grid) Visible(a, b Point) (bool, error) {
	if _, err := g.Walkable(a); err != nil {
		return false, err
	}
	if _, err := g.Walkable(b); err != nil {
		return false, err
	}
	x0, y0 := a.X, a.Y
	x1, y1 := b.X, b.Y
	dx := abs(x1 - x0)
	dy := abs(y1 - y0)
	sx, sy := 1, 1
	if x0 > x1 {
		sx = -1
	}
	if y0 > y1 {
		sy = -1
	}
	err := 0
	for {
		if !g.cells[y0][x0] {
			return false, nil
		}
		if x0 == x1 && y0 == y1 {
			return true, nil
		}
		e2 := 2 * err
		if e2 > -dy {
			err -= dy
			x0 += sx
		}
		if e2 < dx {
			err += dx
			y0 += sy
		}
	}
}
