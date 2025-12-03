package algos

import (
	"errors"
	"math"
	"math/rand"
	"sync"
)

// we use spatial grid for object check optimization
type SpatialGrid struct {
	Cells    map[int][]Position
	CellSize float64
	Cols     int
	Rows     int
	Width    float64
	Height   float64
}

func NewSpatialGrid(width, height, cellSize float64) *SpatialGrid {
	cols := int(math.Ceil(width / cellSize))
	rows := int(math.Ceil(height / cellSize))
	return &SpatialGrid{
		Cells:    make(map[int][]Position),
		CellSize: cellSize,
		Cols:     cols,
		Rows:     rows,
		Width:    width,
		Height:   height,
	}
}

func (g *SpatialGrid) Clear() {
	// for extreme performance, you might pool slice allocations, 
	// but this is sufficient for <100k agents in Go.
	g.Cells = make(map[int][]Position)
}

func (g *SpatialGrid) Add(p Position) {
	c := p.GetCoord()
	col := int(c.X / g.CellSize)
	row := int(c.Y / g.CellSize)
	
	// boundary check
	if col < 0 { col = 0 }
	if col >= g.Cols { col = g.Cols - 1 }
	if row < 0 { row = 0 }
	if row >= g.Rows { row = g.Rows - 1 }

	idx := row*g.Cols + col
	g.Cells[idx] = append(g.Cells[idx], p)
}

func (g *SpatialGrid) GetNearby(coord Coordinate, searchRadius float64) []Position {
	var nearby []Position

	// determine the range of cells to check
	startCol := int((coord.X - searchRadius) / g.CellSize)
	endCol := int((coord.X + searchRadius) / g.CellSize)
	startRow := int((coord.Y - searchRadius) / g.CellSize)
	endRow := int((coord.Y + searchRadius) / g.CellSize)

	// clamp to grid boundaries
	if startCol < 0 { startCol = 0 }
	if endCol >= g.Cols { endCol = g.Cols - 1 }
	if startRow < 0 { startRow = 0 }
	if endRow >= g.Rows { endRow = g.Rows - 1 }

	for r := startRow; r <= endRow; r++ {
		for c := startCol; c <= endCol; c++ {
			idx := r*g.Cols + c
			if items, found := g.Cells[idx]; found {
				nearby = append(nearby, items...)
			}
		}
	}
	return nearby
}

type Coordinate struct {
	X, Y float64
}

type Position interface {
	GetCoord() Coordinate
	GetRadius() float64
}

type FoodSource struct {
	Coordinate
	Radius       float64
	Quality      float64
	Quantity     int
	MaxQuality   float64
	MaxQuantity  int
	RespawnTicks int
	Config       *ABCConfig
	mu           sync.Mutex
}

type Bee struct {
	Coordinate
	Target        Coordinate
	Home          *BeeHive
	Goal          *FoodSource
	Memory        *FoodSource
	Speed         float64
	PathDecayRate float64
	Path          []Coordinate
	SearchRadius  float64
	Role          string
	Color         string
	Nectar        float64
	ForageTicks   int
	NectarCap     float64

	// waggle dance
	DanceTicks     int
	DanceIntensity float64

	Direction float64
	MapWidth  float64
	MapHeight float64
	Config    *ABCConfig
}

type BeeHive struct {
	Coordinate
	Radius           float64
	HoneyCap         float64
	KnownFoodSources []*FoodSource
	Trials           []int
	MaxTrials        int
	Config           *ABCConfig
	mu               sync.Mutex
}

// getters
func (f *FoodSource) GetCoord() Coordinate { return f.Coordinate }
func (f *FoodSource) GetRadius() float64   { return f.Radius }
func (h *BeeHive) GetCoord() Coordinate    { return h.Coordinate }
func (h *BeeHive) GetRadius() float64      { return h.Radius }
func (b *Bee) GetCoord() Coordinate        { return b.Coordinate }
func (b *Bee) GetRadius() float64          { return b.SearchRadius }


func distance(a, b Coordinate) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return math.Sqrt(dx*dx + dy*dy)
}

func distanceSq(a, b Coordinate) float64 {
	dx := a.X - b.X
	dy := a.Y - b.Y
	return dx*dx + dy*dy
}

func getRandomCoordinateExcludingHive(mapWidth, mapHeight, centerX, centerY, radius float64, maxAttempts int) Coordinate {
	attempts := 0
	for attempts < maxAttempts {
		x := rand.Float64()*mapWidth + 1e-9
		y := rand.Float64()*mapHeight + 1e-9
		dist := math.Hypot(x-centerX, y-centerY)
		if dist > radius {
			return Coordinate{X: x, Y: y}
		}
		attempts++
	}
	return Coordinate{X: centerX + radius + 10, Y: centerY + radius + 10}
}

func getRandomCoordinateInCircle(centerX, centerY, radius float64) Coordinate {
	theta := rand.Float64() * 2 * math.Pi
	r := radius * math.Sqrt(rand.Float64())
	x := centerX + r*math.Cos(theta)
	y := centerY + r*math.Sin(theta)
	return Coordinate{X: x, Y: y}
}

func segmentsIntersect(a, b, c, d Coordinate) bool {
	orient := func(p, q, r Coordinate) float64 {
		return (q.Y-p.Y)*(r.X-q.X) - (q.X-p.X)*(r.Y-q.Y)
	}
	return (orient(a, b, c)*orient(a, b, d) < 0) && (orient(c, d, a)*orient(c, d, b) < 0)
}

func LevyStep(beta float64) float64 {
	sigmaU := math.Pow((math.Gamma(1+beta)*math.Sin(math.Pi*beta/2))/
		(math.Gamma((1+beta)/2)*beta*math.Pow(2, (beta-1)/2)), 1/beta)
	u := rand.NormFloat64() * sigmaU
	v := rand.NormFloat64()
	step := u / math.Pow(math.Abs(v), 1/beta)
	return math.Abs(step)
}


func (b *Bee) CalculateFitness(src *FoodSource) float64 {
	d := distance(b.Home.Coordinate, src.Coordinate)
	return (src.Quality * float64(src.Quantity)) / (1.0 + d)
}

func (f *FoodSource) Consume() bool {
	f.mu.Lock()
	defer f.mu.Unlock()

	if f.Quantity > 0 {
		f.Quantity--
		
		if f.Quantity == 0 {
			f.RespawnTicks = 0
			f.Quality = 0
		} else {
			respawnDuration := f.Config.RespawnBaseTime + float64(f.MaxQuantity)*f.Config.RespawnScale
			f.RespawnTicks = int((float64(f.Quantity) / float64(f.MaxQuantity)) * respawnDuration)
		}
		return true
	}
	return false
}

func (b *Bee) PerformDance(onlookers []*Bee) {
	if b.DanceTicks <= 0 || b.Memory == nil {
		return
	}

	// if b.Memory.Quantity <= 0 {
	// 	b.DanceTicks = 0
	// 	return
	// }

	// calculate intensity
	b.DanceIntensity = b.Config.DanceIntensityBase + b.Config.DanceIntensityScale*(b.Nectar/b.NectarCap)
	if b.DanceIntensity > 1.0 {
		b.DanceIntensity = 1.0
	}

	// jitter movement
	jitter := b.Config.Jitter
	b.X += (rand.Float64()*jitter*2 - jitter)
	b.Y += (rand.Float64()*jitter*2 - jitter)

	// recruit logic
	for _, ol := range onlookers {
		if b.Config.DanceAffectsOnlookerOnly && ol.Role != "onlooker" {
			continue
		}
		if ol.Goal != nil {
			continue
		}

		distSq := distanceSq(ol.Coordinate, b.Coordinate)
		
		// approximate attention decay (using squared distance for perf check)
		// e^(-d * 0.2). If d > 20, attention is ~0.01.
		if distSq > 400 { // 20^2
			continue 
		}

		dist := math.Sqrt(distSq)
		attention := math.Exp(-dist * b.Config.AttentionDecay)
		fit := b.CalculateFitness(b.Memory)
		normFit := 1 - math.Exp(-fit * 0.1)   // smooth 0–1 mapping
		prob := b.DanceIntensity * attention * normFit

		if rand.Float64() < prob {
			ol.Goal = b.Memory
			
			angle := rand.Float64() * 2 * math.Pi
			radius := rand.Float64() * 10.0

			if ol.Role == "onlooker" {
				ol.OnlookerSelectSource()
			} else {
				ol.Target = Coordinate{
					X: b.Memory.X + radius*math.Cos(angle),
					Y: b.Memory.Y + radius*math.Sin(angle),
				}
			}
			ol.Speed = b.Config.SpeedEmployed
		}
	}
	b.DanceTicks--
}

func (b *Bee) Update(grid *SpatialGrid) {
	if b.DanceTicks > 0 {
		b.handleDancing(grid)
		return
	}
	
	arrived := b.moveTowardsTarget()
	if arrived {
		b.handleArrival(grid)
		b.pickNextTarget()
	}
}

func (b *Bee) handleArrival(grid *SpatialGrid) {
	nearby := b.checkNearbyObject(grid)
	
	for _, obj := range nearby {
		if foodPtr, ok := obj.(*FoodSource); ok {
			if b.Goal == nil && foodPtr.Quantity > 0 {
				b.Goal = foodPtr
				b.Speed = b.Config.SpeedOnlooker
				break
			}
		}
	}
}

func (b *Bee) moveTowardsTarget() bool {
	dx := b.Target.X - b.X
	dy := b.Target.Y - b.Y
	distSq := dx*dx + dy*dy

	if distSq < b.Speed*b.Speed {
		b.X = b.Target.X
		b.Y = b.Target.Y
		b.Path = append(b.Path, b.Target)
		if len(b.Path) > int(b.PathDecayRate) {
			b.Path = b.Path[1:]
		}
		return true
	}

	dist := math.Sqrt(distSq)
	b.X += (dx / dist) * b.Speed
	b.Y += (dy / dist) * b.Speed
	return false
}

func (b *Bee) handleDancing(grid *SpatialGrid) {
	nearby := b.checkNearbyObject(grid)
	
	var nearbyBees []*Bee
	for _, obj := range nearby {
		if beePtr, ok := obj.(*Bee); ok {
			nearbyBees = append(nearbyBees, beePtr)
		}
	}

	b.PerformDance(nearbyBees)

	if b.DanceTicks <= 0 {
		b.pickNextTarget()
	}
}

func (f *FoodSource) Update() {
	f.mu.Lock()
	defer f.mu.Unlock()

	respawnDuration := f.Config.RespawnBaseTime + float64(f.MaxQuantity)*f.Config.RespawnScale

	if f.Quantity == 0 {
		f.RespawnTicks++
		
		progress := float64(f.RespawnTicks) / respawnDuration
		if progress >= 1.0 {
			f.Quantity = f.MaxQuantity
			f.Quality = f.MaxQuality
			f.RespawnTicks = 0
		}
		
	} else if f.Quantity < f.MaxQuantity {
		f.RespawnTicks++

		progress := float64(f.RespawnTicks) / respawnDuration
		if progress > 1.0 {
			progress = 1.0
		}

		targetQty := int(float64(f.MaxQuantity) * progress)
		
		if targetQty > f.Quantity {
			f.Quantity = targetQty
		}
		
		f.Quality = f.MaxQuality * progress

		if progress >= 1.0 {
			f.RespawnTicks = 0
			f.Quantity = f.MaxQuantity
			f.Quality = f.MaxQuality
		}
	} else {
		f.RespawnTicks = 0
	}
}

func (b *Bee) CheckAbandonedTarget() {
	if b.Goal == nil {
		return
	}

	idx := indexOf(b.Goal, b.Home.KnownFoodSources)
	if idx == -1 {
		return
	}

	b.Home.mu.Lock()
	defer b.Home.mu.Unlock()

	if idx >= len(b.Home.Trials) {
		return 
	}

	if b.Home.Trials[idx] <= b.Home.MaxTrials {
		return
	}

	copy(b.Home.KnownFoodSources[idx:], b.Home.KnownFoodSources[idx+1:])
	b.Home.KnownFoodSources[len(b.Home.KnownFoodSources)-1] = nil 
	b.Home.KnownFoodSources = b.Home.KnownFoodSources[:len(b.Home.KnownFoodSources)-1]

	copy(b.Home.Trials[idx:], b.Home.Trials[idx+1:])
	b.Home.Trials = b.Home.Trials[:len(b.Home.Trials)-1]

	// update bee role
	b.Goal = nil
	switch b.Role {
	case "employed":
		b.Role = "scout"
		b.SearchRadius = b.Config.SearchRadiusScout
		b.Speed = b.Config.SpeedScout
	case "scout":
		b.Role = "employed"
		b.SearchRadius = b.Config.SearchRadiusEmployed
		b.Speed = b.Config.SpeedEmployed
	}
}

func (b *Bee) pickNextTarget() {
	attempts := 0
	distToHome := distance(b.Coordinate, b.Home.Coordinate)

	if distToHome < b.Home.Radius {
		if b.Nectar > b.Config.Epsilon {
			b.depositNectar()
			b.DanceTicks = b.Config.DanceTicksBase + rand.Intn(b.Config.DanceTicksVar)
			b.Target = b.randomHivePatrol()
			return
		}
	}

	if b.Role == "onlooker" && b.Goal == nil {
		b.Target = b.randomHivePatrol()
		return
	}

	if b.Goal != nil {
		distToFood := distance(b.Coordinate, b.Goal.Coordinate)
		
		if distToFood > b.Goal.Radius {
			b.Target = b.Goal.Coordinate
			b.Speed = b.Config.SpeedEmployed
			return
		}

		batch := b.Goal
		if b.ForageTicks == 0 {
			if b.Role == "employed" {
				b.EmployedLocalSearch()
			}
			b.CheckAbandonedTarget()
			
			if batch == b.Goal {
				qty := batch.Quantity 
				b.ForageTicks = int(math.Min(float64(qty), b.NectarCap))
				b.Speed = 0.2
			}
		}

		if b.Nectar < b.NectarCap && b.ForageTicks > 0 && batch.Quantity > 0 {
			b.Target = b.randomFlowerPatrol()
			collected := b.collectNectar()
			
			spaceLeft := b.NectarCap - b.Nectar
			if collected > spaceLeft {
				collected = spaceLeft
			}

			b.Nectar += collected
			b.ForageTicks--
			
			batch.Consume()
			
			return
		}

		if b.ForageTicks <= 0 || batch.Quantity <= 0 || b.Nectar >= b.NectarCap {
			if b.Nectar > b.Config.Epsilon {
				b.Memory = b.Goal
				b.Home.storeInHive(b.Goal)
			}
			b.Target = b.Home.Coordinate
			b.Goal = nil
			b.Speed = b.Config.SpeedEmployed
			b.ForageTicks = 0
			return
		}
	}

	lastCoord := b.Coordinate
	if len(b.Path) > 0 {
		lastCoord = b.Path[len(b.Path)-1]
	}
	if b.Direction == 0 {
		b.Direction = rand.Float64() * 2 * math.Pi
	}

	for attempts < b.Config.MaxAttempts {
		step := math.Min(LevyStep(b.Config.LevyBeta)*b.SearchRadius, 0.3*b.MapWidth)
		newDir := b.Direction + (rand.Float64()-0.5)*0.3
		newX := lastCoord.X + step*math.Cos(newDir)
		newY := lastCoord.Y + step*math.Sin(newDir)

		if newX < 0 || newX > b.MapWidth || newY < 0 || newY > b.MapHeight {
			attempts++
			continue
		}

		newCoord := Coordinate{X: newX, Y: newY}
		
		crossed := false
		if len(b.Path) > 1 {
			checkCount := 5
			startIdx := len(b.Path) - 2 - checkCount
			if startIdx < 0 { startIdx = 0 }
			
			for i := startIdx; i < len(b.Path)-2; i++ {
				if segmentsIntersect(b.Path[i], b.Path[i+1], lastCoord, newCoord) {
					crossed = true
					break
				}
			}
		}

		if !crossed {
			b.Target = newCoord
			b.Direction = newDir
			return
		}
		attempts++
	}

	dirToHive := math.Atan2(b.Home.X-b.Y, b.Home.Y-b.X)
	b.Target = Coordinate{
		X: b.X + 20*math.Cos(dirToHive),
		Y: b.Y + 20*math.Sin(dirToHive),
	}
	b.Direction = dirToHive
}

func (h *BeeHive) storeInHive(src *FoodSource) {
	h.mu.Lock()
	defer h.mu.Unlock()

	for _, known := range h.KnownFoodSources {
		if known == src {
			return
		}
	}
	h.KnownFoodSources = append(h.KnownFoodSources, src)
	h.Trials = append(h.Trials, 0)
}

func indexOf(target *FoodSource, list []*FoodSource) int {
	for i, f := range list {
		if f == target {
			return i
		}
	}
	return -1
}

func (b *Bee) EmployedLocalSearch() {
	b.Home.mu.Lock()
	defer b.Home.mu.Unlock()

	allSources := b.Home.KnownFoodSources
	count := len(allSources)

	if b.Goal == nil || count < 2 {
		return
	}

	var neighbor *FoodSource
	for i := 0; i < 10; i++ {
		neighbor = allSources[rand.Intn(count)]
		if neighbor != b.Goal {
			break
		}
	}

	if neighbor == nil || neighbor == b.Goal {
		return
	}

	currentFit := b.CalculateFitness(b.Goal)
	neighborFit := b.CalculateFitness(neighbor)

	idx := indexOf(b.Goal, b.Home.KnownFoodSources)

	if idx != -1 {
		if neighborFit > currentFit {
			b.Goal = neighbor
			b.ForageTicks = 0
			b.Speed = b.Config.SpeedEmployed
			b.Home.Trials[idx] = 0
		} else {
			b.Home.Trials[idx]++
		}
	}
}

func (b *Bee) OnlookerSelectSource() {
	hive := b.Home
	hive.mu.Lock()
	defer hive.mu.Unlock()

	count := len(hive.KnownFoodSources)
	if count == 0 {
		return
	}

	totalFitness := 0.0
	fitnesses := make([]float64, count)

	for i, src := range hive.KnownFoodSources {
		fit := b.CalculateFitness(src)
		fitnesses[i] = fit
		totalFitness += fit
	}

	if totalFitness < b.Config.Epsilon {
		b.Goal = hive.KnownFoodSources[rand.Intn(count)]
	} else {
		r := rand.Float64() * totalFitness
		acc := 0.0
		selected := false
		for i, fit := range fitnesses {
			acc += fit
			if acc >= r {
				b.Goal = hive.KnownFoodSources[i]
				selected = true
				break
			}
		}
		if !selected {
			b.Goal = hive.KnownFoodSources[count-1]
		}
	}
	b.Target = b.randomFlowerPatrol()
	b.Speed = b.Config.SpeedEmployed
	b.ForageTicks = 0
}

func (b *Bee) randomHivePatrol() Coordinate {
	angle := rand.Float64() * 2 * math.Pi
	r := rand.Float64() * b.Home.Radius
	return Coordinate{
		X: b.Home.Coordinate.X + r*math.Cos(angle),
		Y: b.Home.Coordinate.Y + r*math.Sin(angle),
	}
}

func (b *Bee) randomFlowerPatrol() Coordinate {
	center := b.Coordinate
	if b.Goal != nil {
		center = b.Goal.Coordinate
	}

	angle := rand.Float64() * 2 * math.Pi
	radius := rand.Float64() * 10.0
	return Coordinate{
		X: center.X + radius*math.Cos(angle),
		Y: center.Y + radius*math.Sin(angle),
	}
}

func (b *Bee) checkNearbyObject(grid *SpatialGrid) []Position {
	var nearby []Position
	
	candidates := grid.GetNearby(b.Coordinate, b.SearchRadius)

	for _, obj := range candidates {
		if beeObj, ok := obj.(*Bee); ok && beeObj == b {
			continue
		}

		c := obj.GetCoord()
		dx := c.X - b.X
		dy := c.Y - b.Y
		distSq := dx*dx + dy*dy

		r := obj.GetRadius()
		// (r1 + r2)^2
		totalRadSq := (b.SearchRadius + r) * (b.SearchRadius + r)

		if distSq <= totalRadSq {
			nearby = append(nearby, obj)
		}
	}
	return nearby
}

func (b *Bee) collectNectar() float64 {
	const baseNectar = 1.0
	maxNectar := baseNectar * (b.Goal.Quality / 10.0)
	taken := maxNectar * (0.3 + rand.Float64()*(0.4))
	return taken
}

func (b *Bee) depositNectar() {
	b.Home.HoneyCap += 10e-5 * b.Nectar
	b.Nectar = 0
}

func SpawnBee(populationSize int, width, height float64, beehive *BeeHive, config *ABCConfig) ([]Bee, error) {
	var bees []Bee

	if populationSize < 2 {
		return nil, errors.New("min population not met")
	}

	numEmployed := int(float32(populationSize) * 0.5)
	numOnLooker := populationSize - numEmployed

	createBee := func(role string, color string, searchRadius float64, speed float64) Bee {
		curr := getRandomCoordinateInCircle(beehive.X, beehive.Y, beehive.Radius)
		initDir := rand.Float64() * 2 * math.Pi

		return Bee{
			Coordinate:    curr,
			Target:        curr,
			Home:          beehive,
			SearchRadius:  searchRadius,
			Speed:         speed,
			PathDecayRate: config.PathDecayRate,
			Path:          []Coordinate{curr},
			Role:          role,
			NectarCap:     config.NectarCapBase + rand.Float64()*config.NectarCapVar,
			Color:         color,
			Direction:     initDir,
			MapWidth:      width,
			MapHeight:     height,
			Config:        config,
		}
	}

	for i := 0; i < numEmployed; i++ {
		bees = append(bees, createBee("employed", "#FFD700", config.SearchRadiusEmployed, config.SpeedEmployed))
	}
	for i := 0; i < numOnLooker; i++ {
		bees = append(bees, createBee("onlooker", "#FF8C00", config.SearchRadiusOnlooker, config.SpeedOnlooker))
	}

	return bees, nil
}

func ApproxFlowerCount(circleRadius, flowerRadius float64) int {
	const packingDensity = 0.9069
	return int(packingDensity * (circleRadius*circleRadius) / (flowerRadius*flowerRadius))
}

func SpawnFood(quantity int, width, height float64, beehive *BeeHive, config *ABCConfig) ([]FoodSource, error) {
	var foods []FoodSource
	
	createFood := func() FoodSource {
		loc := getRandomCoordinateExcludingHive(width, height, beehive.X, beehive.Y, beehive.Radius, config.MaxAttempts)
		radius := rand.Float64()*config.MaxFoodRadius + 10.0
		quality := rand.Float64()*config.MaxQuality + config.MinQuality
		quant := ApproxFlowerCount(radius, 1)

		return FoodSource{
			Coordinate:   loc,
			Radius:       radius,
			Quality:      quality,
			Quantity:     quant,
			MaxQuality:   config.MaxQuality,
			MaxQuantity:  quant,
			RespawnTicks: 0,
			Config:       config,
		}
	}

	for i := 0; i < quantity; i++ {
		foods = append(foods, createFood())
	}

	return foods, nil
}