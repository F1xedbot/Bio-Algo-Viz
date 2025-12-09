package algos

// ABCConfig holds all the magic numbers and configuration parameters for the simulation.
type ABCConfig struct {
	// Grid
	GridCellSize float64

	// General
	MaxAttempts int
	Epsilon     float64

	// Respawn Logic
	RespawnBaseTime float64
	RespawnScale    float64 // Ticks per quantity

	// Dance / Communication
	DanceIntensityBase  float64
	DanceIntensityScale float64
	DanceTicksBase      int
	DanceTicksVar       int
	Jitter              float64
	AttentionDecay      float64 // Coefficient for exp(-dist * decay)
	DanceAffectsOnlookerOnly bool

	// Bee Behavior
	SpeedEmployed        float64
	SpeedOnlooker        float64
	SpeedScout           float64
	SearchRadiusEmployed float64
	SearchRadiusOnlooker float64
	SearchRadiusScout    float64
	PathDecayRate        float64
	NectarCapBase        float64
	NectarCapVar         float64
	EmployedWanderLimit  int
	ScoutWanderLimit     int

	// Food Source
	MaxFoodRadius float64
	MaxQuality    float64
	MinQuality    float64

	// Levy Flight
	LevyBeta float64
}

func DefaultABCConfig() ABCConfig {
	return ABCConfig{
		GridCellSize: 50.0,
		MaxAttempts:  50,
		Epsilon:      1e-9,

		RespawnBaseTime: 500.0,
		RespawnScale:    30.0,

		DanceIntensityBase:  0.2,
		DanceIntensityScale: 1.0,
		DanceTicksBase:      50,
		DanceTicksVar:       20,
		Jitter:              1.5,
		AttentionDecay:      0.01,
		DanceAffectsOnlookerOnly: true,

		SpeedEmployed:        0.8,
		SpeedOnlooker:        0.4, 
		SpeedScout:           1.6, 

		SearchRadiusEmployed: 20.0,
		SearchRadiusOnlooker: 20.0,
		SearchRadiusScout:    40.0, 

		PathDecayRate: 20.0,
		NectarCapBase: 30.0,
		NectarCapVar:  10.0,
		EmployedWanderLimit: 120,
		ScoutWanderLimit:    90,

		MaxFoodRadius: 40.0,
		MaxQuality:    10.0,
		MinQuality:    1.0,

		LevyBeta: 1.5,
	}
}
