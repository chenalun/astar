package astar

import (
	"fmt"
	"log"
	"math"
	"sort"
)

type Model uint8

const (
	Model2 Model = 2 // 二维
	Model3 Model = 3 // 三维
)

// 四边形和六边形 A* 寻路算法
type astar struct {
	w, h        int         // 矩形大小 w * h
	start, end  *Coordinate // start 起点坐标(x,y)，end寻找目标坐标（x,y）
	coordinates map[string]*Coordinate
	open        []*Coordinate // 打开的坐标列表
	close       []*Coordinate // 关闭的坐标列表
	closeMap    map[string]struct{}
	result      []*Coordinate // 最终结果
	resultMap   map[string]struct{}
	model       Model
}

// NewAstar 实例化一个模型
func NewAstar(model Model, w, h int) *astar {
	a := &astar{
		w:           w,
		h:           h,
		coordinates: make(map[string]*Coordinate, w*h),
		closeMap:    make(map[string]struct{}),
		resultMap:   make(map[string]struct{}),
		model:       model,
	}

	a.init()

	return a
}

// Init 初始化坐标
func (a *astar) init() {
	var w, h int
	for w = 0; w < a.w; w++ {
		for h = 0; h < a.h; h++ {
			a.coordinates[GetKey(w, h)] = NewCoordinate(w, h)
		}
	}
}

// FindPath 寻找最终路线
func (a *astar) FindPath(start, end *Coordinate) []*Coordinate {
	if start.x >= a.w || start.y >= a.h {
		log.Fatalln("起点坐标超限制")
	}

	if end.x >= a.w || end.y >= a.h {
		log.Fatalln("终点坐标超限制")
	}

	a.start = start
	a.end = end

	// 将起点放入打开列表中
	a.open = append(a.open, a.start)

	// 递归查找
	a.loopFind()

	// 将终点放入列表中
	target := a.end

	// 加入到关闭列表中
	a.addCloseCoordinate(target)

	// 回溯
	a.back(target)

	return a.result
}

// loopFind 开始循环找
func (a *astar) loopFind() []*Coordinate {
	if len(a.open) <= 0 {
		log.Fatalln("无法找到路径！")
	}

	target := a.getMinF()

	if target.x == a.end.x && target.y == a.end.y {
		return a.close
	}

	// 从打开列表中移除
	a.removeOpenCoordinate(target)

	// 放入关闭列表中
	a.addCloseCoordinate(target)

	// 寻找相邻的放入开放列表中
	var list []*Coordinate
	if a.model == Model2 {
		list = a.GetAdjoinCoordinates(target)
	} else {
		list = a.Get3AdjoinCoordinates(target)
	}
	a.addOpen(list...)

	return a.loopFind()
}

// back 回溯（从目标图块开始，然后返回起点。在每一步中，我们只选择一个相邻的瓦片，如果它在封闭列表中并且比前一个瓦片具有更低的移动成本（G））
func (a *astar) back(c *Coordinate) []*Coordinate {
	if len(a.close) > 0 {
		var _c *Coordinate
		if a.model == Model2 {
			_c = a.GetAdjoinCloseCoordinate(c)
		} else {
			_c = a.Get3AdjoinCloseCoordinate(c)
		}

		if _c.x == a.start.x && _c.y == a.start.y { // 回到起点位置
			// 反序
			sort.SliceStable(a.result, func(i, j int) bool {
				return a.result[i].g < a.result[j].g
			})

			return a.result
		} else {
			return a.back(_c)
		}
	}

	return a.result
}

// addOpen 添加一个打开的坐标
func (a *astar) addOpen(c ...*Coordinate) {
	a.open = append(a.open, c...)
}

// SetObstacle 设置不可用坐标（障碍物）
func (a *astar) SetObstacle(list ...*Coordinate) *astar {
	for _, v := range list {
		a.coordinates[GetKey(v.x, v.y)].isObstacle = true
	}

	return a
}

// GetKey 获取map key
func GetKey(x, y int) string {
	return fmt.Sprintf("%d:%d", x, y)
}

// Get3LeftTopCoordinate 左上
func (a *astar) Get3LeftTopCoordinate(c *Coordinate) *Coordinate {
	key := GetKey(c.x-1, c.y+1)

	if _, ok := a.coordinates[key]; ok {
		return a.coordinates[key]
	}
	return nil
}

// Get3RightTopCoordinate 右上
func (a *astar) Get3RightTopCoordinate(c *Coordinate) *Coordinate {
	key := GetKey(c.x, c.y+1)

	if _, ok := a.coordinates[key]; ok {
		return a.coordinates[key]
	}
	return nil
}

// Get3RightCoordinate 右
func (a *astar) Get3RightCoordinate(c *Coordinate) *Coordinate {
	key := GetKey(c.x+1, c.y)

	if _, ok := a.coordinates[key]; ok {
		return a.coordinates[key]
	}

	return nil
}

// Get3RightBottomCoordinate 右下
func (a *astar) Get3RightBottomCoordinate(c *Coordinate) *Coordinate {
	key := GetKey(c.x+1, c.y-1)
	if _, ok := a.coordinates[key]; ok {
		return a.coordinates[key]
	}

	return nil
}

// Get3LeftCoordinate 左
func (a *astar) Get3LeftCoordinate(c *Coordinate) *Coordinate {
	key := GetKey(c.x-1, c.y)
	if _, ok := a.coordinates[key]; ok {
		return a.coordinates[key]
	}

	return nil
}

// Get3LeftBottomCoordinate 左下
func (a *astar) Get3LeftBottomCoordinate(c *Coordinate) *Coordinate {
	key := GetKey(c.x, c.y-1)
	if _, ok := a.coordinates[key]; ok {
		return a.coordinates[key]
	}

	return nil
}

// GetBottomCoordinate 获取相邻底部的坐标
func (a *astar) GetBottomCoordinate(c *Coordinate) *Coordinate {
	key := GetKey(c.x, c.y-1)

	if _, ok := a.coordinates[key]; ok {
		return a.coordinates[key]
	}
	return nil
}

// GetTopCoordinate 获取相邻上面的坐标
func (a *astar) GetTopCoordinate(c *Coordinate) *Coordinate {
	key := GetKey(c.x, c.y+1)

	if _, ok := a.coordinates[key]; ok {
		return a.coordinates[key]
	}
	return nil
}

// GetLeftCoordinate 获取相邻左边的坐标
func (a *astar) GetLeftCoordinate(c *Coordinate) *Coordinate {
	key := GetKey(c.x-1, c.y)

	if _, ok := a.coordinates[key]; ok {
		return a.coordinates[key]
	}
	return nil
}

// GetRightCoordinate 获取相邻右边的坐标
func (a *astar) GetRightCoordinate(c *Coordinate) *Coordinate {
	key := GetKey(c.x+1, c.y)

	if _, ok := a.coordinates[key]; ok {
		return a.coordinates[key]
	}
	return nil
}

// hasClose 指定坐标是否在关闭列表中
func (a *astar) hasClose(c *Coordinate) bool {
	if _, ok := a.closeMap[GetKey(c.x, c.y)]; ok {
		return true
	}

	return false
}

// hasResult 是否已经在结果列表中
func (a *astar) hasResult(c *Coordinate) bool {
	if _, ok := a.resultMap[GetKey(c.x, c.y)]; ok {
		return true
	}

	return false
}

// GetAdjoinCloseCoordinate 获取相邻G值最小的关闭坐标
func (a *astar) GetAdjoinCloseCoordinate(c *Coordinate) *Coordinate {
	var target *Coordinate
	if a.model == Model2 {
		// 相邻的结果
		var slice []*Coordinate

		top := a.GetTopCoordinate(c)
		bottom := a.GetBottomCoordinate(c)
		left := a.GetLeftCoordinate(c)
		right := a.GetRightCoordinate(c)

		if top != nil && a.hasClose(top) && !a.hasResult(top) {
			slice = append(slice, top)
		}

		if bottom != nil && a.hasClose(bottom) && !a.hasResult(bottom) {
			slice = append(slice, bottom)
		}

		if left != nil && a.hasClose(left) && !a.hasResult(left) {
			slice = append(slice, left)
		}

		if right != nil && a.hasClose(right) && !a.hasResult(right) {
			slice = append(slice, right)
		}

		if len(slice) > 0 {
			// G值排序
			sort.SliceStable(slice, func(i, j int) bool {
				return slice[i].g < slice[j].g
			})

			target = slice[0]

			a.addResultCoordinate(target)
		}
	}

	return target
}

// Get3AdjoinCloseCoordinate 获取相邻G值最小的关闭坐标
func (a *astar) Get3AdjoinCloseCoordinate(c *Coordinate) *Coordinate {
	var target *Coordinate
	if a.model == Model3 {
		// 相邻的结果
		var slice []*Coordinate

		left := a.Get3LeftCoordinate(c)
		leftB := a.Get3LeftBottomCoordinate(c)
		leftT := a.Get3LeftTopCoordinate(c)
		right := a.Get3RightCoordinate(c)
		rightB := a.Get3RightBottomCoordinate(c)
		rightT := a.Get3RightTopCoordinate(c)

		if left != nil && a.hasClose(left) && !a.hasResult(left) {
			slice = append(slice, left)
		}

		if leftB != nil && a.hasClose(leftB) && !a.hasResult(leftB) {
			slice = append(slice, leftB)
		}

		if leftT != nil && a.hasClose(leftT) && !a.hasResult(leftT) {
			slice = append(slice, leftT)
		}

		if right != nil && a.hasClose(right) && !a.hasResult(right) {
			slice = append(slice, right)
		}

		if rightB != nil && a.hasClose(rightB) && !a.hasResult(rightB) {
			slice = append(slice, rightB)
		}

		if rightT != nil && a.hasClose(rightT) && !a.hasResult(rightT) {
			slice = append(slice, rightT)
		}

		if len(slice) > 0 {
			// G值排序
			sort.SliceStable(slice, func(i, j int) bool {
				return slice[i].g < slice[j].g
			})

			target = slice[0]

			a.addResultCoordinate(target)
		}
	}

	return target
}

// GetAdjoinCoordinates 获取相邻可用的坐标
func (a *astar) GetAdjoinCoordinates(c *Coordinate) []*Coordinate {
	var slice []*Coordinate

	if a.model == Model2 {
		top := a.GetTopCoordinate(c)
		bottom := a.GetBottomCoordinate(c)
		left := a.GetLeftCoordinate(c)
		right := a.GetRightCoordinate(c)

		if top != nil && !top.isObstacle && !a.hasClose(top) {
			top.setGVal(c.g + 1)
			slice = append(slice, top)
		}

		if bottom != nil && !bottom.isObstacle && !a.hasClose(bottom) {
			bottom.setGVal(c.g + 1)
			slice = append(slice, bottom)
		}

		if left != nil && !left.isObstacle && !a.hasClose(left) {
			left.setGVal(c.g + 1)
			slice = append(slice, left)
		}

		if right != nil && !right.isObstacle && !a.hasClose(right) {
			right.setGVal(c.g + 1)
			slice = append(slice, right)
		}
	}

	return slice
}

// Get3AdjoinCoordinates 获取相邻可用的坐标
func (a *astar) Get3AdjoinCoordinates(c *Coordinate) []*Coordinate {
	var slice []*Coordinate

	if a.model == Model3 {
		left := a.Get3LeftCoordinate(c)
		leftB := a.Get3LeftBottomCoordinate(c)
		leftT := a.Get3LeftTopCoordinate(c)
		right := a.Get3RightCoordinate(c)
		rightB := a.Get3RightBottomCoordinate(c)
		rightT := a.Get3RightTopCoordinate(c)

		if left != nil && !left.isObstacle && !a.hasClose(left) {
			left.setGVal(c.g + 1)
			slice = append(slice, left)
		}

		if leftB != nil && !leftB.isObstacle && !a.hasClose(leftB) {
			leftB.setGVal(c.g + 1)
			slice = append(slice, leftB)
		}

		if leftT != nil && !leftT.isObstacle && !a.hasClose(leftT) {
			leftT.setGVal(c.g + 1)
			slice = append(slice, leftT)
		}

		if right != nil && !right.isObstacle && !a.hasClose(right) {
			right.setGVal(c.g + 1)
			slice = append(slice, right)
		}

		if rightB != nil && !rightB.isObstacle && !a.hasClose(rightB) {
			rightB.setGVal(c.g + 1)
			slice = append(slice, rightB)
		}

		if rightT != nil && !rightT.isObstacle && !a.hasClose(rightT) {
			rightT.setGVal(c.g + 1)
			slice = append(slice, rightT)
		}
	}

	return slice
}

// GetHVal 获取两个点的H值
// 曼哈顿距离:计算到目标点还有多少地格。 H = |x1 – x2| + |y1 – y2|
func GetHVal(start, end *Coordinate) uint {
	return uint(math.Abs(float64(start.x-end.x)) + math.Abs(float64(start.y-end.y)))
}

// Get3HVal H = Max(|x1 – x2|, |y1 – y2|, |z1 – z2|)
func Get3HVal(start, end *Coordinate) uint {
	num := math.Max(math.Max(math.Abs(float64(start.x-end.x)), math.Abs(float64(start.y-end.y))), math.Abs(float64(start.Z()-end.Z())))
	return uint(num)
}

// SearchMinF 打开列表中搜索F值最小的
func (a *astar) getMinF() *Coordinate {
	if len(a.open) == 1 {
		return a.open[0]
	}

	coords := a.open

	sort.SliceStable(coords, func(i, j int) bool {
		if coords[i].GetFVal(a) < coords[j].GetFVal(a) { // 优先F值最低的排序
			return true
		} else if coords[i].GetFVal(a) > coords[j].GetFVal(a) {
			return false
		}

		//  F值相等，G值排序
		return coords[i].g > coords[j].g
	})

	return coords[0]
}

// removeOpenCoordinate 移除开启列表中指定的坐标
func (a *astar) removeOpenCoordinate(c *Coordinate) {
	i := 0
	for _, v := range a.open {
		if v != c {
			a.open[i] = v
			i++
		}
	}

	a.open = a.open[:i]
}

// addOpenCoordinate 加入到关闭列表中
func (a *astar) addCloseCoordinate(c *Coordinate) {
	key := GetKey(c.x, c.y)

	if _, ok := a.closeMap[key]; !ok {
		a.closeMap[key] = struct{}{}

		a.close = append(a.close, c)
	}

}

// addResultCoordinate 添加最终结果坐标
func (a *astar) addResultCoordinate(c *Coordinate) {
	key := GetKey(c.x, c.y)

	if _, ok := a.resultMap[key]; !ok {
		a.resultMap[key] = struct{}{}

		a.result = append(a.result, c)
	}
}
