//go:build js && wasm

package main

import (
	"encoding/json"
	"fmt"
	"math"
	"math/rand"
	"sort"
	"syscall/js"

	"github.com/F1xedbot/bio-algo-viz/pkg/algos"
)

var stopCurrentAlgo func()

// Structure to send to JavaScript
type UIUpdate struct {
	Pop       int
	FoodCount int
	GridSize  float64
	Employed  int
	Onlooker  int
	Scout     int
	Honey     float64
	Sources   []SourceInfo
}

type SourceInfo struct {
	ID      int
	Fitness float64
	Quality float64
}

type SimConfigUpdate struct {
	PopulationSize int
	FoodCount      int
	
	// Config params
	SpeedEmployed        float64
	SpeedOnlooker        float64
	SpeedScout           float64
	RespawnScale         float64
	SearchRadiusEmployed float64
	SearchRadiusOnlooker float64
	SearchRadiusScout    float64
	
	// Detailed Params
	MaxAttempts         int
	DanceIntensityBase  float64
	DanceIntensityScale float64
	DanceTicksBase      int
	DanceTicksVar       int
	Jitter              float64
	AttentionDecay      float64
	PathDecayRate       float64
	NectarCapBase       float64
	NectarCapVar        float64
	LevyBeta            float64
	
	MaxQuality               float64
	MaxFoodRadius            float64
	DanceAffectsOnlookerOnly bool
}

func main() {
	js.Global().Set("switchAlgo", js.FuncOf(func(this js.Value, args []js.Value) interface{} {
		if len(args) > 0 {
			handleAlgoSwitch(args[0].String())
		}
		return nil
	}))
	select {}
}

func handleAlgoSwitch(name string) {
	if stopCurrentAlgo != nil {
		stopCurrentAlgo()
		stopCurrentAlgo = nil
	}

	doc := js.Global().Get("document")
	canvas := doc.Call("getElementById", "viz-canvas")
	if canvas.IsNull() { return }
	
	width := js.Global().Get("innerWidth").Float()
	height := js.Global().Get("innerHeight").Float()
	canvas.Set("width", width)
	canvas.Set("height", height)
	
	ctx := canvas.Call("getContext", "2d")

	switch name {
	case "bees":
		config := algos.DefaultABCConfig()
		startBeeColony(ctx, width, height, &config, 100, 25)
	}
}

func startBeeColony(ctx js.Value, screenWidth, screenHeight float64, initialConfig *algos.ABCConfig, initPop, initFood int) {
	zoom := 0.35
	simWidth := screenWidth / zoom
	simHeight := screenHeight / zoom
	
	config := *initialConfig

	minDim := simWidth
	if simHeight < simWidth { minDim = simHeight }

	hive := algos.BeeHive{
		Coordinate: algos.Coordinate{X: simWidth / 2, Y: simHeight / 2},
		Radius:     minDim * 0.15,
		MaxTrials:  config.MaxAttempts,
		Config:     &config,
	}

	populationSize := initPop
	bees, err := algos.SpawnBee(populationSize, simWidth, simHeight, &hive, &config)
	if err != nil { return }

	foodCount := initFood
	foods, err := algos.SpawnFood(foodCount, simWidth, simHeight, &hive, &config)
	if err != nil { return }

	grid := algos.NewSpatialGrid(simWidth, simHeight, config.GridCellSize)

	selectedSourceID := -1

	js.Global().Set("selectSource", js.FuncOf(func(this js.Value, args []js.Value) interface{} {
		if len(args) > 0 {
			selectedSourceID = args[0].Int()
		}
		return nil
	}))

	var applyConfig js.Func
	applyConfig = js.FuncOf(func(this js.Value, args []js.Value) interface{} {
		if len(args) > 0 {
			var update SimConfigUpdate
			err := json.Unmarshal([]byte(args[0].String()), &update)
			if err == nil {
				// Stop current loop
				stopCurrentAlgo()

				// Create new config
				newConfig := config // copy current
				newConfig.SpeedEmployed = update.SpeedEmployed
				newConfig.SpeedOnlooker = update.SpeedOnlooker
				newConfig.SpeedScout = update.SpeedScout
				newConfig.RespawnScale = update.RespawnScale
				newConfig.SearchRadiusEmployed = update.SearchRadiusEmployed
				newConfig.SearchRadiusOnlooker = update.SearchRadiusOnlooker
				newConfig.SearchRadiusEmployed = update.SearchRadiusEmployed
				newConfig.SearchRadiusOnlooker = update.SearchRadiusOnlooker
				newConfig.SearchRadiusScout = update.SearchRadiusScout

				// Detailed Params
				newConfig.MaxAttempts = update.MaxAttempts
				newConfig.DanceIntensityBase = update.DanceIntensityBase
				newConfig.DanceIntensityScale = update.DanceIntensityScale
				newConfig.DanceTicksBase = update.DanceTicksBase
				newConfig.DanceTicksVar = update.DanceTicksVar
				newConfig.Jitter = update.Jitter
				newConfig.AttentionDecay = update.AttentionDecay
				newConfig.PathDecayRate = update.PathDecayRate
				newConfig.NectarCapBase = update.NectarCapBase
				newConfig.NectarCapVar = update.NectarCapVar
				newConfig.NectarCapBase = update.NectarCapBase
				newConfig.NectarCapVar = update.NectarCapVar
				newConfig.LevyBeta = update.LevyBeta
				
				newConfig.MaxQuality = update.MaxQuality
				newConfig.MaxFoodRadius = update.MaxFoodRadius
				newConfig.DanceAffectsOnlookerOnly = update.DanceAffectsOnlookerOnly
				
				go func() {
					startBeeColony(ctx, screenWidth, screenHeight, &newConfig, update.PopulationSize, update.FoodCount)
				}()
			}
		}
		return nil
	})
	js.Global().Set("applyConfig", applyConfig)

	running := true
	stopCurrentAlgo = func() { 
		running = false
		applyConfig.Release()
	}
	
	frameCount := 0

	var renderFrame js.Func
	renderFrame = js.FuncOf(func(this js.Value, args []js.Value) interface{} {
		if !running {
			renderFrame.Release()
			return nil
		}
		frameCount++

		// update particles
		updateParticles()

		// physics & grid logic
		grid.Clear()
		for i := range foods {
			foods[i].Update()
			grid.Add(&foods[i])
		}
		for i := range bees {
			grid.Add(&bees[i])
		}
		roleCounts := map[string]int{"employed": 0, "onlooker": 0, "scout": 0}
		for i := range bees {
			b := &bees[i]
			b.Update(grid)
			roleCounts[b.Role]++
		}

		// render canvas (bees & food only)
		ctx.Call("setTransform", 1, 0, 0, 1, 0, 0)
		ctx.Set("fillStyle", "#222")
		ctx.Call("fillRect", 0, 0, screenWidth, screenHeight)
		ctx.Call("scale", zoom, zoom)

		// Borders & Hive
		ctx.Set("strokeStyle", "#333"); ctx.Set("lineWidth", 10)
		ctx.Call("strokeRect", 0, 0, simWidth, simHeight)
		
		// Hive
		ctx.Set("shadowBlur", 20); ctx.Set("shadowColor", "#33ff33") // Glow
		ctx.Call("beginPath"); ctx.Call("arc", hive.X, hive.Y, hive.Radius, 0, 2*math.Pi)
		ctx.Set("fillStyle", "rgba(50, 255, 50, 0.05)"); ctx.Call("fill")
		ctx.Set("strokeStyle", "#33ff33"); ctx.Set("lineWidth", 5); ctx.Call("stroke")
		ctx.Set("shadowBlur", 0) // Reset glow

		ctx.Call("beginPath"); ctx.Call("arc", hive.X, hive.Y, 15, 0, 2*math.Pi)
		ctx.Set("fillStyle", "#33ff33"); ctx.Call("fill")

		// Food
		for i := range foods {
			f := &foods[i]
			ctx.Call("beginPath")
			ctx.Call("arc", f.X, f.Y, f.Radius, 0, 2*math.Pi)
			
			// Highlight Selection
			if i == selectedSourceID {
				ctx.Set("shadowBlur", 30)
				ctx.Set("shadowColor", "#00FFFF")
				ctx.Set("strokeStyle", "#00FFFF")
				ctx.Set("lineWidth", 6)
				ctx.Call("stroke")
				ctx.Set("shadowBlur", 0)
			}
			
			// Draw Flower Patch
			if f.Quantity > 0 {
				ctx.Set("fillStyle", "#FF4444")
				ctx.Call("fill")
				
				maxQual := f.MaxQuality; if maxQual == 0 { maxQual = 10.0 }
				qualRatio := f.Quality / maxQual
				if qualRatio > 1.0 { qualRatio = 1.0 }
				
				hue := qualRatio * 50.0 
				
				if qualRatio > 0.8 {
					ctx.Set("shadowBlur", 15)
					ctx.Set("shadowColor", fmt.Sprintf("hsl(%.0f, 100%%, 50%%)", hue))
				}

				ctx.Set("strokeStyle", fmt.Sprintf("hsl(%.0f, 100%%, 50%%)", hue))
				ctx.Set("lineWidth", 4)
				ctx.Call("stroke")
				ctx.Set("shadowBlur", 0) // Reset
				
				ctx.Set("fillStyle", "#FFF"); ctx.Set("font", "bold 24px monospace"); ctx.Set("textAlign", "center")
				ctx.Call("fillText", fmt.Sprintf("%d", f.Quantity), f.X, f.Y - 5)
			} else {
				ctx.Set("fillStyle", "#555"); ctx.Call("fill")
				ctx.Set("strokeStyle", "#333"); ctx.Set("lineWidth", 2); ctx.Call("stroke")
			}

			if f.RespawnTicks > 0 {
				respawnDuration := 500.0 + float64(f.MaxQuantity)*20.0
				prog := float64(f.RespawnTicks)/respawnDuration
				if prog > 1 { prog = 1 }
				
				barWidth := f.Radius * 2
				barHeight := 10.0
				barX := f.X - f.Radius
				barY := f.Y + f.Radius + 10

				ctx.Set("fillStyle", "#000"); ctx.Call("fillRect", barX, barY, barWidth, barHeight)
				ctx.Set("fillStyle", "#00FFFF"); ctx.Call("fillRect", barX, barY, barWidth*prog, barHeight)
			}
		}

		// Bees
		for i := range bees {
			b := &bees[i]
			if len(b.Path) > 0 {
				// Optimization: Don't draw path if inside hive
				distToHive := math.Sqrt(math.Pow(b.X-hive.X, 2) + math.Pow(b.Y-hive.Y, 2))
				startDistToHive := math.Sqrt(math.Pow(b.Path[0].X-hive.X, 2) + math.Pow(b.Path[0].Y-hive.Y, 2))
				
				if distToHive > hive.Radius || startDistToHive > hive.Radius {
					ctx.Call("beginPath"); ctx.Call("moveTo", b.Path[0].X, b.Path[0].Y)
					for _, p := range b.Path { ctx.Call("lineTo", p.X, p.Y) }
					ctx.Call("lineTo", b.X, b.Y)
					ctx.Set("strokeStyle", "rgba(255, 255, 255, 0.1)"); ctx.Set("lineWidth", 1); ctx.Call("stroke")
				}
			}
			ctx.Call("beginPath"); ctx.Call("arc", b.X, b.Y, 6, 0, 2*math.Pi)
			ctx.Set("fillStyle", b.Color); ctx.Call("fill")

			if b.ForageTicks > 0 && frameCount % 5 == 0 {
				spawnParticle(b.X, b.Y, b.Color)
			}
		}
		drawParticles(ctx)	

		if frameCount % 10 == 0 {
			// prepare source data
			topSources := make([]SourceInfo, 0)
			for i, src := range hive.KnownFoodSources {
				dist := math.Sqrt(math.Pow(hive.X-src.X, 2) + math.Pow(hive.Y-src.Y, 2))
				fit := (src.Quality * float64(src.Quantity)) / (1.0 + dist)
				topSources = append(topSources, SourceInfo{i, fit, src.Quality})
			}
			sort.Slice(topSources, func(i, j int) bool { return topSources[i].Fitness > topSources[j].Fitness })

			uiData := UIUpdate{
				Pop:       populationSize,
				FoodCount: foodCount,
				GridSize:  config.GridCellSize,
				Employed:  roleCounts["employed"],
				Onlooker:  roleCounts["onlooker"],
				Scout:     roleCounts["scout"],
				Honey:     hive.HoneyCap,
				Sources:   topSources,
			}

			jsonData, _ := json.Marshal(uiData)
			js.Global().Call("updateUI", string(jsonData))
		}

		js.Global().Call("requestAnimationFrame", renderFrame)
		return nil
	})
	js.Global().Call("requestAnimationFrame", renderFrame)
}

// particle system
type Particle struct {
	X, Y, VX, VY float64
	Life         float64
	Color        string
}

var particles []Particle

func spawnParticle(x, y float64, color string) {
	for i := 0; i < 3; i++ {
		angle := rand.Float64() * 2 * math.Pi
		speed := rand.Float64() * 2.0
		p := Particle{
			X: x, Y: y,
			VX: math.Cos(angle) * speed,
			VY: math.Sin(angle) * speed,
			Life: 1.0,
			Color: color,
		}
		particles = append(particles, p)
	}
}

func updateParticles() {
	active := particles[:0]
	for _, p := range particles {
		p.X += p.VX
		p.Y += p.VY
		p.Life -= 0.05
		if p.Life > 0 {
			active = append(active, p)
		}
	}
	particles = active
}

func drawParticles(ctx js.Value) {
	for _, p := range particles {
		ctx.Set("globalAlpha", p.Life)
		ctx.Set("fillStyle", p.Color)
		ctx.Call("beginPath")
		ctx.Call("arc", p.X, p.Y, 2, 0, 2*math.Pi)
		ctx.Call("fill")
	}
	ctx.Set("globalAlpha", 1.0)
}