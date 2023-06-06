package astar_test

import (
	"astar/astar"
	"fmt"
	"testing"
)

// Test2FindPath 四边形二维寻路
func Test2FindPath(t *testing.T) {
	var list []*astar.Coordinate

	// 障碍物
	list = append(list,
		astar.NewCoordinate(1, 2),
		astar.NewCoordinate(2, 1),
		astar.NewCoordinate(3, 0),
	)

	r := astar.NewAstar(astar.Model3, 7, 7).SetObstacle(list...).FindPath(astar.NewCoordinate(0, 1), astar.NewCoordinate(5, 2))

	for _, v := range r {
		fmt.Println(fmt.Sprintf("最终路线的列表:(x=%d,y=%d,z=%d)", v.X(), v.Y(), v.Z()))
	}

}

// Test3FindPath 六边形三维寻路
func Test3FindPath(t *testing.T) {
	var list []*astar.Coordinate

	list = append(list,
		astar.NewCoordinate(1, 2),
		astar.NewCoordinate(3, 2),
		astar.NewCoordinate(3, 4),
		astar.NewCoordinate(4, 3),
	)

	star, end := astar.NewCoordinate(1, 3), astar.NewCoordinate(5, 2)

	r := astar.NewAstar(astar.Model2, 7, 7).SetObstacle(list...).FindPath(star, end)

	for _, v := range r {
		fmt.Println(fmt.Sprintf("最终路线的列表:(x=%d,y=%d)", v.X(), v.Y()))
	}
}
