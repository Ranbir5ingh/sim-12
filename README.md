# Intelligent Robot Simulator with Autonomous Navigation

A complete robotic simulation system featuring intelligent path planning, real-time obstacle detection, and autonomous navigation capabilities. The robot uses A* pathfinding, computer vision-based obstacle mapping, and adaptive movement strategies to navigate complex environments.

## 🎯 System Overview

This system demonstrates advanced robotics concepts including:
- **Autonomous Navigation**: A* pathfinding with dynamic replanning
- **Computer Vision**: Real-time obstacle detection using image processing
- **Adaptive Behavior**: Dynamic movement strategies based on environmental conditions
- **Real-time Communication**: WebSocket-based robot control and monitoring

## 🏗️ Architecture

```
┌─────────────────┐    WebSocket     ┌──────────────────┐
│  Robot Simulator│ ←──────────────→ │   Python Server  │
│   (simulator.html)                  │   (server.py)    │
└─────────────────┘                  └──────────────────┘
                                             ↑
                                        HTTP REST API
                                             │
┌─────────────────┐                         │
│ Robot Controller│ ←───────────────────────┘
│ (controller.py) │
└─────────────────┘
```

## 🚀 Features

### Core Capabilities
- **Intelligent Path Planning**: Multi-strategy A* pathfinding for dense environments
- **Dynamic Obstacle Avoidance**: Real-time obstacle detection and map updates
- **Adaptive Movement**: Speed optimization based on obstacle proximity
- **Collision Recovery**: Automatic reset and replanning after collisions
- **Visual Feedback**: Real-time simulation with collision detection

### Advanced Features
- **Computer Vision Integration**: Canvas capture and image processing for obstacle detection
- **Escape Planning**: Advanced algorithms for navigating out of trapped situations
- **Path Smoothing**: Intelligent waypoint reduction for efficient movement
- **Multi-strategy Planning**: Fallback algorithms for complex scenarios

## 📋 Requirements

### System Requirements
- Python 3.7+
- Modern web browser (Chrome, Firefox, Safari, Edge)
- At least 4GB RAM recommended

### Python Dependencies
```bash
pip install flask flask_cors websockets asyncio pillow numpy opencv-python
```

## 🛠️ Installation & Setup

### 1. Clone or Download the Project
```bash
# If using git
git clone <repository-url>
cd robot-simulator

# Or download and extract the files
```

### 2. Install Python Dependencies
```bash
pip install -r requirements.txt
```

Or install manually:
```bash
pip install flask flask_cors websockets asyncio pillow numpy opencv-python
```

### 3. Verify File Structure
```
robot-simulator/
├── server.py              # Backend server (WebSocket + Flask API)
├── controller.py           # Autonomous robot controller
├── planner.py             # A* pathfinding algorithm
├── sensor.py              # Computer vision obstacle detection
├── simulator.html         # Visual simulation interface
└── README.md              # This file
```

## 🎮 How to Run

### Method 1: Autonomous Navigation (Recommended)

1. **Start the Backend Server**
   ```bash
   python server.py
   ```
   You should see:
   ```
   🚀 Starting 2D Robot Server...
   WebSocket server started on ws://localhost:8080
   Flask server starting on http://localhost:5001
   ```

2. **Open the Visual Simulator**
   - Open `simulator.html` in your web browser
   - The simulator will automatically connect to the WebSocket server
   - You'll see a robot (red circle) at position (320, 300)

3. **Run the Autonomous Controller**
   ```bash
   python controller.py
   ```
   
   The robot will:
   - Perform initial environment scanning
   - Plan an optimal path to the goal
   - Navigate autonomously while avoiding obstacles
   - Adapt movement speed based on obstacle proximity
   - Automatically recover from collisions

### Method 2: Manual Control via API

You can also control the robot manually using the REST API:

```bash
# Move robot to specific position
curl -X POST http://localhost:5001/move \
  -H "Content-Type: application/json" \
  -d '{"x": 400, "y": 200}'

# Move robot relative to current position
curl -X POST http://localhost:5001/move_rel \
  -H "Content-Type: application/json" \
  -d '{"angle": 90, "distance": 50}'

# Set new goal position
curl -X POST http://localhost:5001/goal \
  -H "Content-Type: application/json" \
  -d '{"x": 500, "y": 100}'

# Capture current environment
curl http://localhost:5001/capture
```

## 🎛️ Configuration

### Canvas Settings
- **Dimensions**: 650×600 pixels
- **Robot Size**: 18px radius
- **Goal Size**: 15px radius  
- **Obstacle Size**: 25px (default)

### Algorithm Parameters

**Path Planning (`planner.py`):**
```python
grid_size = 15          # Fine grid for tight spaces
robot_radius = 18       # Robot collision radius
safety_margin = 3-15    # Adaptive safety margins
```

**Movement Control (`controller.py`):**
```python
min_step_size = 25      # Minimum movement step
max_step_size = 50      # Maximum movement step  
safe_distance = 80      # "Safe" distance from obstacles
sensing_interval = 0.05-0.15  # Adaptive sensing frequency
```

**Vision System (`sensor.py`):**
```python
min_contour_area = 200       # Minimum obstacle size
min_obstacle_separation = 35 # Minimum distance between obstacles
```

## 🧠 Algorithm Details

### 1. Path Planning Strategy
The system uses a multi-strategy approach:

1. **Direct Path**: Attempts straight-line navigation
2. **A* with Adaptive Margins**: Uses different safety margins (3px → 8px → 15px)
3. **Escape-Based Planning**: Creates intermediate waypoints in dense areas
4. **Path Smoothing**: Reduces waypoints while maintaining safety

### 2. Obstacle Detection Pipeline
1. **Image Capture**: Canvas-to-base64 conversion
2. **Preprocessing**: Grayscale conversion, blur, adaptive thresholding
3. **Contour Detection**: Shape analysis with area/aspect ratio filtering
4. **Validation**: Edge filtering, robot area exclusion, duplicate removal
5. **Integration**: Merge with existing obstacle map

### 3. Adaptive Movement System
- **Fast Movement**: Large steps (50px) when >80px from obstacles
- **Medium Movement**: Medium steps (37px) when moderately close
- **Careful Movement**: Small steps (25px) when <50px from obstacles
- **Dynamic Sensing**: Faster sensing when safe, careful sensing when close

### 4. Collision Recovery
1. **Immediate Stop**: Robot stops at collision point
2. **Obstacle Registration**: Add collision point to obstacle map
3. **Environment Reset**: Robot returns to start position (320, 300)
4. **Replanning**: Generate new path with updated obstacle knowledge

## 📊 Performance Characteristics

### Speed Optimization
- **Base sensing interval**: 50ms (when safe)
- **Careful sensing interval**: 150ms (near obstacles)
- **Step sizes**: 25-50px adaptive
- **Path smoothing**: Reduces waypoints by ~60-80%

### Robustness Features
- **Multi-strategy planning**: 4 fallback algorithms
- **Collision recovery**: Automatic reset and replanning
- **Dense environment handling**: Escape-based pathfinding
- **Vision filtering**: Multiple validation layers

## 🔧 Troubleshooting

### Common Issues

**WebSocket Connection Failed**
```bash
# Check if server is running
netstat -an | grep 8080
# Restart server if needed
python server.py
```

**Robot Not Moving**
- Verify WebSocket connection (green status in simulator)
- Check browser console for errors
- Ensure coordinates are within bounds (0-650, 0-600)

**Path Planning Failures**
- Environment may be too dense
- Try reducing obstacle count
- Check for obstacles blocking start/goal positions

**Computer Vision Issues**
- Ensure PIL, numpy, opencv-python are installed
- Check browser supports canvas.toDataURL()
- Verify image capture returns valid base64 data

**High CPU Usage**
- Reduce sensing frequency in controller.py
- Increase step sizes for faster movement
- Lower A* node exploration limit

### Debug Mode
Add debug prints to see algorithm behavior:
```python
# In controller.py
print(f"🎯 Planning path from {start} to {goal}")
print(f"📊 Known obstacles: {len(self.known_obstacles)}")
print(f"🚶 Step size: {step_size}, Nearest obstacle: {nearest_dist}")
```

## 📈 Performance Tips

### For Dense Environments
1. Reduce grid size in planner.py (e.g., 10px)
2. Increase inflation radius for more clearance
3. Enable escape-based planning earlier

### For Speed Optimization  
1. Increase base sensing interval (e.g., 0.1s)
2. Use larger step sizes (e.g., 75px max)
3. Reduce path smoothing samples

### For Reliability
1. Lower safety margins (3-5px)
2. Enable more aggressive path smoothing
3. Increase collision detection sensitivity

## 🎯 Example Use Cases

### 1. Warehouse Robot Navigation
- Dense obstacle environments
- Precise positioning requirements
- Collision avoidance critical

### 2. Autonomous Vehicle Testing
- Path planning validation
- Sensor fusion simulation  
- Dynamic replanning scenarios

### 3. Educational Robotics
- Algorithm visualization
- Real-time behavior analysis
- Interactive parameter tuning

## 🔬 Research Applications

This system demonstrates several advanced robotics concepts:

- **SLAM (Simultaneous Localization and Mapping)**: Real-time environment mapping
- **Multi-Agent Systems**: WebSocket communication protocols
- **Computer Vision**: Real-time image processing for navigation
- **Adaptive Algorithms**: Dynamic parameter adjustment based on environment
- **Robust Navigation**: Multiple fallback strategies for complex scenarios

## 📝 API Reference

### Movement Commands
- `POST /move` - Absolute positioning
- `POST /move_rel` - Relative movement  
- `POST /stop` - Emergency stop

### Environment Management
- `GET /capture` - Environment scanning
- `POST /obstacles/random` - Generate test obstacles
- `POST /obstacles/positions` - Set custom obstacles

### System Status
- `GET /status` - Connection and system state
- `GET /collisions` - Collision statistics
- `POST /reset` - Full system reset

## 🤝 Contributing

To extend this system:

1. **Add New Algorithms**: Implement in `planner.py`
2. **Improve Vision**: Enhance `sensor.py` processing
3. **Add Behaviors**: Extend `controller.py` logic
4. **UI Enhancements**: Modify `simulator.html`

## 📄 License

This project is open source. Feel free to use, modify, and distribute.

## 👥 Credits

Developed as a comprehensive robotics simulation demonstrating modern autonomous navigation techniques.

---

**Happy Robot Navigation! 🤖✨**
