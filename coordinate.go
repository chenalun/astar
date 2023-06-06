package astar

type Coordinate struct {
	x          int
	y          int
	g          uint // G值
	isObstacle bool // 是否是障碍物
}

// NewCoordinate 新建一个坐标
func NewCoordinate(x, y int) *Coordinate {
	return &Coordinate{
		x: x,
		y: y,
	}
}

func (c *Coordinate) X() int {
	return c.x
}

func (c *Coordinate) Y() int {
	return c.y
}

// Z x + y + z = 0
func (c *Coordinate) Z() int {
	return 0 - c.x - c.y
}

// setGVal 设置G值
func (c *Coordinate) setGVal(g uint) {
	c.g = g
}

// GetFVal 获取F值
func (c *Coordinate) GetFVal(s *astar) uint {
	var h uint
	if s.model == Model2 {
		h = GetHVal(c, s.end)
	} else {
		h = Get3HVal(c, s.end)
	}

	return c.g + h
}
