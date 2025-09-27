# Simulator WebSocket + Flask API

A bridge between a simulator (WebSocket) and Flask REST API for canvas capture functionality and robot control.

## System Overview

The system consists of three main components:
- **Robot Simulator** - Visual interface showing robot, obstacles, and goal
- **Remote Controller** - Web-based control panel for robot commands  
- **Python Server** - Backend handling WebSocket communication and API endpoints

## Features

- Real-time robot movement with visual feedback
- Canvas capture functionality with base64 image data
- Collision detection with obstacles
- Goal-reaching detection with visual notifications
- WebSocket-based communication
- RESTful API for robot control and canvas capture
- Clean, responsive UI design
- Server-side state tracking

## Architecture

```
┌─────────────────┐    WebSocket     ┌──────────────────┐
│  Robot Simulator│ ←──────────────→ │   Python Server  │
└─────────────────┘                  └──────────────────┘
                                             ↑
                                        HTTP REST API
                                             │
┌─────────────────┐                         │
│ Remote Controller│ ←───────────────────────┘
└─────────────────┘
```

## Getting Started

### Prerequisites

- Python 3.7+
- Modern web browser
- Required Python packages:
  ```bash
  pip install flask flask_cors websockets asyncio
  ```

### Installation & Setup

1. **Start the Python Server**
   ```bash
   python server.py
   ```
   - WebSocket Server: `ws://localhost:8080`
   - Flask API Server: `http://localhost:5001`

2. **Open the Robot Simulator**
   - Open `robot_simulator.html` in your web browser
   - The simulator will automatically connect to the WebSocket

3. **Open the Remote Controller**
   - Open `robot_controller.html` in your web browser
   - Use this interface to send commands to the robot

## Canvas Specifications

- **Dimensions**: 650x600 pixels
- **Robot Size**: 18 pixels radius
- **Goal Size**: 15 pixels radius
- **Obstacle Size**: 25 pixels (default)
- **Detection Range**: ~33 pixels (robot + goal radius)

## API Endpoints

### Canvas Capture

| Endpoint | Method | Description | Parameters |
|----------|--------|-------------|------------|
| `/capture` | GET | Trigger canvas capture from simulator | None |

### Movement Commands

| Endpoint | Method | Description | Parameters |
|----------|--------|-------------|------------|
| `/move` | POST | Move robot to absolute position | `{"x": 325, "y": 300}` |
| `/move_rel` | POST | Move robot relative to current position | `{"angle": 45, "distance": 80}` |
| `/stop` | POST | Stop robot movement | None |

### Goal Management

| Endpoint | Method | Description | Parameters |
|----------|--------|-------------|------------|
| `/goal` | POST | Set goal position | `{"x": 550, "y": 80}` or `{"corner": "NE"}` |
| `/goal/status` | GET | Check if goal is reached | None |

### Obstacle Management

| Endpoint | Method | Description | Parameters |
|----------|--------|-------------|------------|
| `/obstacles/random` | POST | Generate random obstacles | `{"count": 8}` |
| `/obstacles/positions` | POST | Set custom obstacles | `{"obstacles": [...]}` |

### System Status

| Endpoint | Method | Description | Returns |
|----------|--------|-------------|---------|
| `/status` | GET | Get system status | Connection count, collisions, goal status |
| `/collisions` | GET | Get collision count | Current collision count |
| `/reset` | POST | Reset entire system | Clears collisions, goal status, obstacles |

## WebSocket Messages

### Outgoing Commands (Controller → Simulator)
```json
{"command": "move", "target": {"x": 325, "y": 300}}
{"command": "move_relative", "angle": 45, "distance": 80}
{"command": "stop"}
{"command": "set_goal", "position": {"x": 550, "y": 80}}
{"command": "set_obstacles", "obstacles": [...]}
{"command": "capture_canvas"}
{"command": "reset"}
```

### Incoming Events (Simulator → Server)
```json
{"type": "collision", "collision": true, "robot_position": {...}, "obstacle_position": {...}}
{"type": "goal_reached", "robot_position": {...}, "goal_position": {...}}
{"type": "connection", "message": "2D Robot simulator connected"}
{"type": "canvas_captured", "image_data": "base64_data", "timestamp": "2024-01-01T00:00:00Z"}
```

## Usage Examples

### Canvas Capture
```bash
curl http://localhost:5001/capture
```

```javascript
fetch('http://localhost:5001/capture')
  .then(response => response.json())
  .then(data => console.log('Canvas captured:', data));
```

### Basic Movement
```javascript
// Move to specific coordinates
fetch('http://localhost:5001/move', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({x: 325, y: 300})
});

// Move relative to current position
fetch('http://localhost:5001/move_rel', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({angle: 90, distance: 100})
});
```

### Goal Management
```javascript
// Set goal to corner
fetch('http://localhost:5001/goal', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({corner: 'NE'})
});

// Check if goal is reached
fetch('http://localhost:5001/goal/status')
  .then(response => response.json())
  .then(data => console.log('Goal reached:', data.goal_reached));
```

## Canvas Capture Response

**Success Response:**
```json
{
  "status": "success",
  "image_data": "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAA...",
  "timestamp": "2024-01-01T12:00:00.000Z",
  "canvas_size": {"width": 650, "height": 600},
  "robot_position": {"x": 320, "y": 300, "angle": 45},
  "goal_position": {"x": 550, "y": 80},
  "obstacles_count": 8,
  "message": "Canvas image captured successfully"
}
```

**Error Response:**
```json
{
  "status": "error", 
  "message": "No connected simulators"
}
```

## How It Works

1. **Client Request**: Client calls API endpoint
2. **WebSocket Broadcast**: Server sends command via WebSocket to simulator
3. **Simulator Response**: Simulator processes command and responds with data
4. **API Response**: Server returns processed data to client
5. **Canvas Capture**: Special flow where simulator captures canvas as base64 image

**Canvas Capture Flow:**
1. Client calls `/capture` endpoint
2. Server broadcasts `capture_canvas` command
3. Simulator captures canvas and responds with `canvas_captured` event containing base64 image data
4. Server returns image data and metadata to client
5. 3-second timeout if simulator doesn't respond

## Corner Positions

| Corner | Coordinates |
|--------|-------------|
| NW (Top-Left) | (20, 20) |
| NE (Top-Right) | (630, 20) |
| SW (Bottom-Left) | (20, 580) |
| SE (Bottom-Right) | (630, 580) |

## File Structure

```
robot-simulator/
├── server.py                 # Python backend server
├── simulator.html      # Main simulator interface
├── controller.html     # Remote control interface
└── README.md                 # This file
```

## Troubleshooting

### WebSocket Connection Issues
- Ensure Python server is running on port 8080
- Check browser console for connection errors
- Try clicking "Reconnect WebSocket" button

### API Connection Issues
- Verify Flask server is running on port 5001
- Check for CORS errors in browser console
- Ensure correct server URLs in controller

### Canvas Capture Issues
- Ensure simulator is connected to WebSocket
- Check for canvas_captured messages in WebSocket logs
- Verify simulator handles capture_canvas commands

### Robot Not Moving
- Check WebSocket connection status
- Verify coordinates are within canvas bounds (0-650, 0-600)
- Ensure robot is not colliding with obstacles