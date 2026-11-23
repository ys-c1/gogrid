package grid

import (
	"fmt"
	"testing"
)

// 基准：开阔地图上的全开扫描 vs 不同张角的扇形 FOV，以及两档半径的耗时对比。
// 运行：go test ./grid -bench 'BenchmarkFOV' -benchmem
func benchGrid(b *testing.B) *Grid {
	b.Helper()
	const n = 24
	rows := make([]string, n)
	line := make([]byte, n)
	for x := range line {
		line[x] = '.'
	}
	for y := range rows {
		rows[y] = string(line)
	}
	g, err := Parse(rows)
	if err != nil {
		b.Fatal(err)
	}
	return g
}

func BenchmarkFOVAngles(b *testing.B) {
	g := benchGrid(b)
	origin := Point{X: 12, Y: 12}
	for _, angle := range []float64{45, 90, 180, 360} {
		cfg := FOVConfig{Origin: origin, Facing: East, AngleDeg: angle, Radius: 10}
		b.Run(fmt.Sprintf("angle-%g", angle), func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				if _, err := g.FOV(cfg); err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}

func BenchmarkFOVRadii(b *testing.B) {
	g := benchGrid(b)
	origin := Point{X: 12, Y: 12}
	for _, radius := range []int{10, 5} {
		cfg := FOVConfig{Origin: origin, Facing: East, AngleDeg: 90, Radius: radius}
		b.Run(fmt.Sprintf("radius-%d", radius), func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				if _, err := g.FOV(cfg); err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}

func BenchmarkVisibleSet(b *testing.B) {
	g := benchGrid(b)
	origin := Point{X: 12, Y: 12}
	for _, radius := range []int{10, 5} {
		r := radius
		b.Run(fmt.Sprintf("radius-%d", r), func(b *testing.B) {
			b.ReportAllocs()
			for i := 0; i < b.N; i++ {
				if _, err := g.VisibleSet(origin, r); err != nil {
					b.Fatal(err)
				}
			}
		})
	}
}
