package grid

import "fmt"

// SamePointError 起点与终点重合（或相邻两个必经点重合），携带坐标。
type SamePointError struct {
	Point Point
}

func (e *SamePointError) Error() string {
	return fmt.Sprintf("起终点相同: (%d,%d)", e.Point.X, e.Point.Y)
}

// BlockedPointError 必经点（含起点、终点）落在障碍格或越界，携带坐标。
type BlockedPointError struct {
	Point       Point
	OutOfBounds bool
}

func (e *BlockedPointError) Error() string {
	if e.OutOfBounds {
		return fmt.Sprintf("必经点越界: (%d,%d)", e.Point.X, e.Point.Y)
	}
	return fmt.Sprintf("必经点不可走: (%d,%d)", e.Point.X, e.Point.Y)
}

// Unwrap 兼容既有哨兵错误判断：errors.Is(err, ErrOOB/ErrNoPath) 仍成立。
func (e *BlockedPointError) Unwrap() error {
	if e.OutOfBounds {
		return ErrOOB
	}
	return ErrNoPath
}

// NoPathError 两个必经点之间无路可达，携带该段两端坐标。
type NoPathError struct {
	From Point
	To   Point
}

func (e *NoPathError) Error() string {
	return fmt.Sprintf("无路可达: (%d,%d) -> (%d,%d)", e.From.X, e.From.Y, e.To.X, e.To.Y)
}

func (e *NoPathError) Unwrap() error { return ErrNoPath }
