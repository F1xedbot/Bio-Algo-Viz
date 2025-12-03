# Bio-Algo Visualization

A web-based visualization of biological algorithms, built with Go (WASM) and HTML5 Canvas.

## Overview

This project simulates natural swarm intelligence behaviors to demonstrate how simple individual rules can lead to complex collective intelligence.

### Current Visualizations

*   **Artificial Bee Colony (ABC)**: Simulates the foraging behavior of honey bees. Watch as employed bees, onlookers, and scouts work together to optimize nectar collection from food sources.

### Coming Soon

*   **Ant Colony Optimization**: Trail laying and pheromone-based pathfinding.

## Running Locally

1.  **Build WASM**:
    ```bash
    ./scripts/build_wasm.sh
    ```

2.  **Start Server**:
    ```bash
    cd docs
    python3 -m http.server 8080
    ```

3.  **View**: Open `http://localhost:8080` in your browser.

## Tech Stack

*   **Backend**: Go (compiled to WebAssembly)
*   **Frontend**: HTML, CSS, JavaScript
*   **Rendering**: HTML5 Canvas
