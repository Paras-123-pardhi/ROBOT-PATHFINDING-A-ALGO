/**
 * ============================================================================
 * PathBot A* - Robot Pathfinding Visualizer
 * Design and Analysis of Algorithms (DAA) Interactive Project
 * 
 * Key Concepts Implemented:
 * 1. A* Graph Search Algorithm with f(n) = g(n) + h(n)
 * 2. Manhattan Distance Heuristic: h(n) = |x1 - x2| + |y1 - y2|
 * 3. Strict 4-Directional Movement: Up, Down, Left, Right (No Diagonals)
 * 4. Open List Priority Queue & Closed Set Visited Tracking
 * 5. Optimal Shortest Path Reconstruction via Parent Node Pointers
 * 6. Visual Animation of Node Exploration & Step-by-Step Robot Traversal
 * 7. Comprehensive Mazes & Benchmark Layouts
 * ============================================================================
 */

// --- 1. CONFIGURATION & STATE CONSTANTS ---
const GRID_ROWS = 15;
const GRID_COLS = 15;

const START_NODE = { row: 0, col: 0 };
const GOAL_NODE = { row: 14, col: 14 };

// Visual Cell Labels (as per Project Poster & Specifications)
const LABEL_START = "S";
const LABEL_GOAL = "G";
const ICON_ROBOT = "🤖";

// Directions: 4-Way strictly (Up, Down, Left, Right) - NO Diagonal Movement
const DIRECTIONS = [
  { dr: -1, dc: 0, name: "Up" },
  { dr: 1, dc: 0, name: "Down" },
  { dr: 0, dc: -1, name: "Left" },
  { dr: 0, dc: 1, name: "Right" }
];

// --- 2. APPLICATION STATE ---
let grid = []; // 2D matrix of GridNode objects
let isMouseDown = false;
let isDrawingObstacles = true; // true = draw obstacle, false = erase obstacle
let isRunning = false; // Prevents concurrent pathfinding executions

// Speed Settings (in milliseconds)
const SPEEDS = {
  1: { explore: 65, robot: 140, label: "Slow" },
  2: { explore: 30, robot: 80, label: "Normal" },
  3: { explore: 12, robot: 45, label: "Fast" }
};
let currentSpeed = SPEEDS[3];

// --- 3. DOM ELEMENTS ---
const gridContainer = document.getElementById("grid");
const findPathBtn = document.getElementById("findPathBtn");
const clearObstaclesBtn = document.getElementById("clearObstaclesBtn");
const resetGridBtn = document.getElementById("resetGridBtn");
const presetSelect = document.getElementById("presetSelect");
const speedRange = document.getElementById("speedRange");
const speedLabel = document.getElementById("speedLabel");

const statusVal = document.getElementById("statusVal");
const pathLengthVal = document.getElementById("pathLengthVal");
const cellsExploredVal = document.getElementById("cellsExploredVal");
const notificationBanner = document.getElementById("notificationBanner");
const notificationIcon = document.getElementById("notificationIcon");
const notificationText = document.getElementById("notificationText");

// --- 4. NODE CLASS REPRESENTATION ---
/**
 * Represents each cell in the 15x15 grid for the A* search graph.
 */
class GridNode {
  constructor(row, col) {
    this.row = row;
    this.col = col;
    this.isObstacle = false;
    this.isStart = (row === START_NODE.row && col === START_NODE.col);
    this.isGoal = (row === GOAL_NODE.row && col === GOAL_NODE.col);
    
    // A* Evaluation Costs:
    this.g = Infinity; // g(n): Actual cost from Start to this node
    this.h = 0;        // h(n): Heuristic estimated cost from this node to Goal
    this.f = Infinity; // f(n) = g(n) + h(n): Total estimated cost
    
    this.parent = null;  // Pointer to parent node for path reconstruction
    this.element = null; // DOM reference
  }

  resetCosts() {
    this.g = Infinity;
    this.h = 0;
    this.f = Infinity;
    this.parent = null;
  }
}

// --- 5. INITIALIZE GRID ---
function initGrid() {
  gridContainer.innerHTML = "";
  grid = [];

  for (let r = 0; r < GRID_ROWS; r++) {
    const rowArray = [];
    for (let c = 0; c < GRID_COLS; c++) {
      const node = new GridNode(r, c);

      // Create DOM element for the cell
      const cellEl = document.createElement("div");
      cellEl.classList.add("cell");
      cellEl.dataset.row = r;
      cellEl.dataset.col = c;

      if (node.isStart) {
        cellEl.classList.add("start");
        cellEl.textContent = LABEL_START;
        cellEl.title = "Start Node (S)";
      } else if (node.isGoal) {
        cellEl.classList.add("goal");
        cellEl.textContent = LABEL_GOAL;
        cellEl.title = "Goal Node (G)";
      }

      // Mouse & Touch Event Listeners for Drawing Obstacles
      cellEl.addEventListener("mousedown", (e) => handleCellMouseDown(node, e));
      cellEl.addEventListener("mouseenter", () => handleCellMouseEnter(node));

      node.element = cellEl;
      gridContainer.appendChild(cellEl);
      rowArray.push(node);
    }
    grid.push(rowArray);
  }

  // Window mouseup listener to stop obstacle dragging
  window.addEventListener("mouseup", () => {
    isMouseDown = false;
  });

  // Touch Support for mobile / touchscreens
  gridContainer.addEventListener("touchmove", handleTouchMove, { passive: false });
  window.addEventListener("touchend", () => {
    isMouseDown = false;
  });
}

// --- 6. USER INTERACTION: OBSTACLE DRAWING & DRAGGING ---
function handleCellMouseDown(node, event) {
  if (isRunning) return;
  if (node.isStart || node.isGoal) return;

  isMouseDown = true;
  // If clicked cell is obstacle, dragging will erase; otherwise dragging will draw
  isDrawingObstacles = !node.isObstacle;
  toggleObstacle(node, isDrawingObstacles);
}

function handleCellMouseEnter(node) {
  if (!isMouseDown || isRunning) return;
  if (node.isStart || node.isGoal) return;

  toggleObstacle(node, isDrawingObstacles);
}

function handleTouchMove(event) {
  if (isRunning) return;
  event.preventDefault();
  const touch = event.touches[0];
  const targetEl = document.elementFromPoint(touch.clientX, touch.clientY);
  if (targetEl && targetEl.classList.contains("cell")) {
    const r = parseInt(targetEl.dataset.row, 10);
    const c = parseInt(targetEl.dataset.col, 10);
    if (!isNaN(r) && !isNaN(c)) {
      const node = grid[r][c];
      if (!node.isStart && !node.isGoal) {
        toggleObstacle(node, true);
      }
    }
  }
}

function toggleObstacle(node, shouldBeObstacle) {
  node.isObstacle = shouldBeObstacle;
  if (shouldBeObstacle) {
    node.element.classList.add("obstacle");
    node.element.textContent = "";
  } else {
    node.element.classList.remove("obstacle");
    node.element.textContent = "";
  }
  // Clear any previous path or exploration visualization
  cleanVisualizationOnly();
}

// --- 7. HEURISTIC CALCULATION (MANHATTAN DISTANCE) ---
/**
 * Manhattan Distance Heuristic:
 * h(n) = |x1 - x2| + |y1 - y2|
 * Admissible and consistent heuristic for 4-directional grid pathfinding.
 */
function calculateManhattanDistance(nodeA, nodeB) {
  return Math.abs(nodeA.row - nodeB.row) + Math.abs(nodeA.col - nodeB.col);
}

// --- 8. A* (A-STAR) ALGORITHM IMPLEMENTATION ---
/**
 * Pure A* search algorithm:
 * - Initializes Open List (min-heap logic) and Closed Set
 * - Evaluates f(n) = g(n) + h(n)
 * - Returns the order of explored nodes and reconstructed shortest path
 */
function runAStar() {
  const startNode = grid[START_NODE.row][START_NODE.col];
  const goalNode = grid[GOAL_NODE.row][GOAL_NODE.col];

  // Reset algorithm costs for all nodes
  for (let r = 0; r < GRID_ROWS; r++) {
    for (let c = 0; c < GRID_COLS; c++) {
      grid[r][c].resetCosts();
    }
  }

  // Open List: Nodes to be evaluated
  // Closed Set: Nodes already evaluated
  const openList = [];
  const closedSet = new Set();
  const exploredOrder = []; // Track sequence for step-by-step animation

  // Initialize start node
  startNode.g = 0;
  startNode.h = calculateManhattanDistance(startNode, goalNode);
  startNode.f = startNode.g + startNode.h;
  openList.push(startNode);

  while (openList.length > 0) {
    // 1. Select the node in Open List with the LOWEST f(n) value
    // Tie-breaker: If f-scores are equal, choose the node with lower heuristic h(n)
    openList.sort((a, b) => (a.f === b.f ? a.h - b.h : a.f - b.f));
    const currentNode = openList.shift();

    // 2. If Goal reached -> Success!
    if (currentNode === goalNode) {
      const shortestPath = reconstructPath(goalNode);
      return {
        success: true,
        exploredOrder,
        path: shortestPath
      };
    }

    // 3. Move current node from Open List to Closed Set
    closedSet.add(currentNode);
    if (!currentNode.isStart && !currentNode.isGoal) {
      exploredOrder.push(currentNode);
    }

    // 4. Generate and inspect valid 4-way neighbors
    const neighbors = getValidNeighbors(currentNode);

    for (const neighbor of neighbors) {
      // Ignore if neighbor is an obstacle or already evaluated in Closed Set
      if (neighbor.isObstacle || closedSet.has(neighbor)) {
        continue;
      }

      // Orthogonal step cost is 1
      const tentativeG = currentNode.g + 1;

      // Check if this path to neighbor is better than any previously discovered
      const isAlreadyInOpen = openList.includes(neighbor);

      if (!isAlreadyInOpen || tentativeG < neighbor.g) {
        neighbor.parent = currentNode;
        neighbor.g = tentativeG;
        neighbor.h = calculateManhattanDistance(neighbor, goalNode);
        neighbor.f = neighbor.g + neighbor.h;

        if (!isAlreadyInOpen) {
          openList.push(neighbor);
        }
      }
    }
  }

  // Open list exhausted and goal was not reached -> No path exists
  return {
    success: false,
    exploredOrder,
    path: []
  };
}

/**
 * Returns valid orthogonal neighbors (Up, Down, Left, Right).
 */
function getValidNeighbors(node) {
  const neighbors = [];
  for (const dir of DIRECTIONS) {
    const newRow = node.row + dir.dr;
    const newCol = node.col + dir.dc;

    // Boundary check
    if (newRow >= 0 && newRow < GRID_ROWS && newCol >= 0 && newCol < GRID_COLS) {
      neighbors.push(grid[newRow][newCol]);
    }
  }
  return neighbors;
}

/**
 * Reconstructs the shortest path from Goal to Start by following parent pointers.
 */
function reconstructPath(goalNode) {
  const path = [];
  let current = goalNode;
  while (current !== null) {
    path.unshift(current);
    current = current.parent;
  }
  return path;
}

// --- 9. VISUALIZATION & ANIMATION ENGINE ---

const delay = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

async function startPathfindingVisualization() {
  if (isRunning) return;

  // Clean previous search states
  cleanVisualizationOnly();
  setControlState(true);

  // Update Status UI
  setStatus("Finding Path...", "status-searching");
  hideNotification();

  // Run pure A* logic
  const { success, exploredOrder, path } = runAStar();

  // 1. Animate Explored Cells (Open/Closed List exploration)
  for (let i = 0; i < exploredOrder.length; i++) {
    const node = exploredOrder[i];
    node.element.classList.add("explored");
    cellsExploredVal.textContent = i + 1;
    await delay(currentSpeed.explore);
  }

  // 2. Handle Outcome
  if (success) {
    setStatus("Path Found", "status-found");
    pathLengthVal.textContent = path.length - 1; // Number of steps (edges)
    cellsExploredVal.textContent = exploredOrder.length;

    // Highlight final optimal shortest path
    for (let i = 0; i < path.length; i++) {
      const node = path[i];
      if (!node.isStart && !node.isGoal) {
        node.element.classList.add("path");
      }
      await delay(currentSpeed.explore * 1.5);
    }

    // 3. Animate Robot Traversal along the Path
    await animateRobotMovement(path);

    // Show Success Notification Banner
    showNotification("Robot reached the destination successfully!", false);
  } else {
    setStatus("No Path Found", "status-failed");
    pathLengthVal.textContent = "-";
    cellsExploredVal.textContent = exploredOrder.length;

    // Show Error Alert Banner
    showNotification("No Path Found! Destination is completely blocked by obstacles.", true);
  }

  setControlState(false);
}

/**
 * Animates the robot moving cell-by-cell along the calculated path.
 */
async function animateRobotMovement(path) {
  const startEl = grid[START_NODE.row][START_NODE.col].element;
  const goalEl = grid[GOAL_NODE.row][GOAL_NODE.col].element;

  for (let i = 0; i < path.length; i++) {
    const currentNode = path[i];
    currentNode.element.classList.add("has-robot");

    await delay(currentSpeed.robot);

    // Remove robot indicator from current cell before stepping to next
    if (i < path.length - 1) {
      currentNode.element.classList.remove("has-robot");
    }
  }

  // Ensure Start and Goal retain their clear S and G labels
  startEl.textContent = LABEL_START;
  goalEl.textContent = LABEL_GOAL;
}

// --- 10. HELPER & CLEANUP FUNCTIONS ---

function cleanVisualizationOnly() {
  for (let r = 0; r < GRID_ROWS; r++) {
    for (let c = 0; c < GRID_COLS; c++) {
      const node = grid[r][c];
      node.element.classList.remove("explored", "path", "has-robot");
      if (node.isStart) {
        node.element.textContent = LABEL_START;
      } else if (node.isGoal) {
        node.element.textContent = LABEL_GOAL;
      } else if (!node.isObstacle) {
        node.element.textContent = "";
      }
    }
  }
}

function clearAllObstacles() {
  if (isRunning) return;
  for (let r = 0; r < GRID_ROWS; r++) {
    for (let c = 0; c < GRID_COLS; c++) {
      const node = grid[r][c];
      node.isObstacle = false;
      node.element.classList.remove("obstacle");
      if (node.isStart) {
        node.element.textContent = LABEL_START;
      } else if (node.isGoal) {
        node.element.textContent = LABEL_GOAL;
      } else {
        node.element.textContent = "";
      }
    }
  }
  cleanVisualizationOnly();
  resetMetrics();
  presetSelect.value = "none";
  hideNotification();
}

function resetEntireGrid() {
  if (isRunning) return;
  clearAllObstacles();
  initGrid();
  resetMetrics();
  presetSelect.value = "none";
  hideNotification();
}

function resetMetrics() {
  setStatus("Ready", "status-ready");
  pathLengthVal.textContent = "-";
  cellsExploredVal.textContent = "-";
}

function setStatus(text, className) {
  statusVal.textContent = text;
  statusVal.className = `metric-value ${className}`;
}

function showNotification(message, isError) {
  notificationBanner.classList.remove("hidden");
  if (isError) {
    notificationBanner.classList.add("alert-error");
    notificationIcon.textContent = "❌";
  } else {
    notificationBanner.classList.remove("alert-error");
    notificationIcon.textContent = "✅";
  }
  notificationText.textContent = message;
}

function hideNotification() {
  notificationBanner.classList.add("hidden");
}

function setControlState(disabled) {
  isRunning = disabled;
  findPathBtn.disabled = disabled;
  clearObstaclesBtn.disabled = disabled;
  resetGridBtn.disabled = disabled;
  presetSelect.disabled = disabled;
}

// --- 11. PRESET SAMPLE MAZES FOR EDUCATIONAL DEMONSTRATIONS ---
function applyPresetMaze(preset) {
  if (isRunning) return;
  clearAllObstacles();

  const walls = [];

  switch (preset) {
    case "poster-maze":
      // Layout inspired by Figure 2 & Figure 3 of the college research poster
      // Interlocking central wall structures with bypass pathways
      for (let r = 2; r <= 8; r++) walls.push({ r, c: 4 });
      for (let c = 4; c <= 10; c++) walls.push({ r: 8, c });
      for (let r = 4; r <= 12; r++) walls.push({ r, c: 10 });
      for (let c = 1; c <= 7; c++) walls.push({ r: 12, c });
      break;

    case "simple-wall":
      // Single vertical wall with a middle opening at row 7
      for (let r = 1; r < 14; r++) {
        if (r !== 7) walls.push({ r, c: 7 });
      }
      break;

    case "double-wall":
      // Two vertical barrier walls with offset gates (Gate at row 2 on first, row 12 on second)
      for (let r = 0; r < 13; r++) {
        if (r !== 2) walls.push({ r, c: 5 });
      }
      for (let r = 2; r < 15; r++) {
        if (r !== 12) walls.push({ r, c: 10 });
      }
      break;

    case "zigzag":
      // 3 horizontal corridors creating an S-curve path
      for (let c = 0; c < 12; c++) walls.push({ r: 4, c });
      for (let c = 3; c < 15; c++) walls.push({ r: 8, c });
      for (let c = 0; c < 12; c++) walls.push({ r: 12, c });
      break;

    case "trap":
      // U-shaped dead end facing Start, forcing A* to expand into trap then backtrack
      for (let r = 2; r <= 9; r++) walls.push({ r, c: 4 });
      for (let c = 4; c <= 11; c++) walls.push({ r: 9, c });
      for (let r = 2; r <= 9; r++) walls.push({ r, c: 11 });
      break;

    case "chokepoint":
      // Diagonal compression walls leading to a single 1-cell bottleneck at (7, 7)
      for (let i = 0; i < 7; i++) {
        walls.push({ r: i, c: 7 });
        walls.push({ r: 14 - i, c: 7 });
      }
      // Top & bottom side blocks
      for (let c = 3; c <= 11; c++) {
        if (c !== 7) {
          walls.push({ r: 3, c });
          walls.push({ r: 11, c });
        }
      }
      break;

    case "diagonal-steps":
      // Stepped barrier showing how 4-way Manhattan movement cleanly routes around diagonals
      for (let i = 2; i < 13; i++) {
        walls.push({ r: i, c: 14 - i });
      }
      break;

    case "spiral":
      // Concentric labyrinth forcing robot to loop through grid
      for (let c = 2; c <= 13; c++) walls.push({ r: 2, c });
      for (let r = 2; r <= 13; r++) walls.push({ r, c: 13 });
      for (let c = 4; c <= 13; c++) walls.push({ r: 13, c });
      for (let r = 4; r <= 13; r++) walls.push({ r, c: 4 });
      for (let c = 4; c <= 11; c++) walls.push({ r: 4, c });
      for (let r = 4; r <= 11; r++) walls.push({ r, c: 11 });
      for (let c = 6; c <= 11; c++) walls.push({ r: 11, c });
      for (let r = 6; r <= 11; r++) walls.push({ r, c: 6 });
      break;

    case "random":
      // Randomized 25% obstacles ensuring Start and Goal corners remain open
      for (let r = 0; r < GRID_ROWS; r++) {
        for (let c = 0; c < GRID_COLS; c++) {
          if ((r <= 1 && c <= 1) || (r >= 13 && c >= 13)) continue;
          if (Math.random() < 0.25) {
            walls.push({ r, c });
          }
        }
      }
      break;

    case "blocked":
      // Completely seal the Goal corner (14, 14)
      walls.push({ r: 13, c: 14 });
      walls.push({ r: 14, c: 13 });
      walls.push({ r: 13, c: 13 });
      break;

    default:
      break;
  }

  // Apply obstacles to the grid
  for (const { r, c } of walls) {
    if ((r !== START_NODE.row || c !== START_NODE.col) &&
        (r !== GOAL_NODE.row || c !== GOAL_NODE.col)) {
      grid[r][c].isObstacle = true;
      grid[r][c].element.classList.add("obstacle");
      grid[r][c].element.textContent = "";
    }
  }
}

// --- 12. EVENT LISTENERS SETUP ---
findPathBtn.addEventListener("click", startPathfindingVisualization);
clearObstaclesBtn.addEventListener("click", clearAllObstacles);
resetGridBtn.addEventListener("click", resetEntireGrid);

presetSelect.addEventListener("change", (e) => {
  applyPresetMaze(e.target.value);
});

speedRange.addEventListener("input", (e) => {
  const val = parseInt(e.target.value, 10);
  currentSpeed = SPEEDS[val];
  speedLabel.textContent = currentSpeed.label;
});

// Initialize on page load
document.addEventListener("DOMContentLoaded", () => {
  initGrid();
  resetMetrics();
});
