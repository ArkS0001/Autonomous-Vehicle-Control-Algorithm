from flask import Flask, render_template, request
import subprocess
import numpy as np

app = Flask(__name__)

# List of available maps
MAPS = {
    "map1": {
        "x_points": [0.0, 10.0, 20, 30, 40, 50],
        "y_points": [0.0, 4, 10, 5, 20, -13],
        "speed": 20,
        "obstacles": [
            {"x": 10.0, "y": 15.0, "length": 10, "width": 3},
            {"x": 40.0, "y": 0.0, "length": 2, "width": 10},
            {"x": 20.0, "y": 0.0, "yaw": np.rad2deg(45), "length": 5, "width": 5},
            {"x": 30.0, "y": 15.0, "yaw": np.rad2deg(10), "length": 5, "width": 2},
            {"x": 50.0, "y": 15.0, "yaw": np.rad2deg(15), "length": 5, "width": 2},
            {"x": 25.0, "y": -10.0, "length": 2, "width": 6},
            {"x": 35.0, "y": -15.0, "length": 7, "width": 2},
        ],
    },
    "map2": {
        "x_points": [0.0, 10.0, 25, 40, 50],
        "y_points": [0.0, 4, -12, 15, -13],
        "speed": 40,
        "obstacles": [
            {"x": 10.0, "y": 15.0, "length": 10, "width": 8},
            {"x": 40.0, "y": 0.0, "length": 2, "width": 10},
            {"x": 10.0, "y": -10.0, "yaw": np.rad2deg(45), "length": 5, "width": 5},
            {"x": 30.0, "y": 15.0, "yaw": np.rad2deg(10), "length": 5, "width": 2},
            {"x": 50.0, "y": 15.0, "yaw": np.rad2deg(15), "length": 5, "width": 2},
            {"x": 25.0, "y": 0.0, "length": 2, "width": 2},
            {"x": 35.0, "y": -15.0, "length": 7, "width": 2},
            {"x": 40.0, "y": 20.0, "length": 2, "width": 2},
        ],
    },
    "map3": {
        "x_points": [0.0, 10.0, 25, 35, 40, 45, 50],
        "y_points": [0.0, 0, 5, 10, 15, 10, -13],
        "speed": 30,
        "obstacles": [
            {"x": 10.0, "y": 15.0, "length": 10, "width": 8},
            {"x": 40.0, "y": 0.0, "length": 2, "width": 10},
            {"x": 10.0, "y": -10.0, "yaw": np.rad2deg(45), "length": 5, "width": 5},
            {"x": 30.0, "y": 15.0, "yaw": np.rad2deg(10), "length": 5, "width": 2},
            {"x": 50.0, "y": 15.0, "yaw": np.rad2deg(15), "length": 5, "width": 2},
            {"x": 25.0, "y": 0.0, "length": 2, "width": 2},
            {"x": 35.0, "y": -15.0, "length": 7, "width": 2},
            {"x": 40.0, "y": 20.0, "length": 2, "width": 2},
        ],
    },
    "map4": {
        "x_points": [0.0, 10.0, 25, 40, 50],
        "y_points": [0.0, 4, -10, 5, -13],
        "speed": 30,
        "obstacles": [{"x": j, "y": (-1) ** (j // 10) * 10, "length": 3, "width": 4} for j in range(5, 50, 10)],
    },
    "map5": {
        "x_points": [0.0, 12, 20, 30, 50],
        "y_points": [0.0, 6, 13, 10, -13],
        "speed": 30,
        "obstacles": [
            {"x": 25 + 8 * np.cos(np.radians(angle)), "y": 0 + 8 * np.sin(np.radians(angle)), "length": 3, "width": 3}
            for angle in range(0, 360, 45)
        ],
    },
    "map6": {
        "x_points": [0.0, 10, 20, 30, 40, 51],
        "y_points": [-5, 8, -7, 7, -8, 11],
        "speed": 30,
        "obstacles": [{"x": x, "y": y, "length": 3, "width": 1} for x in range(5, 50, 10) for y in [10, -10]],
    },
    "map7": {
        "x_points": [0.0, 10, 20, 30, 32, 40, 50],
        "y_points": [0, -5, -10, 1, -5, -10, 5],
        "speed": 30,
        "obstacles": [{"x": x, "y": y, "length": 5, "width": 10} for x, y in [(20, 5), (40, 5), (30, 15)]],
    },
    "map8": {
        "x_points": [0.0, 0.0, 10, 15, 15, 30, 45, 50],
        "y_points": [0, -16, -17, -15, 15, 20, 15, 0],
        "speed": 30,
        "obstacles": [{"x": x, "y": y, "length": 2, "width": 10} for x in range(10, 50, 10) for y in [5, -5]]
        + [{"x": 30, "y": y, "length": 10, "width": 2} for y in range(-10, 15, 10)],
    },
}


@app.route('/')
def home():
    return render_template('index1.html', maps=MAPS.keys())

@app.route('/run_simulation', methods=['POST'])
def run_simulation():
    selected_map = request.form.get('map')
    
    if selected_map in MAPS:
        # Pass the selected obstacle list as arguments to the simulation script
        subprocess.Popen(["python", "src/simulations/mapping/ndt_map_construction/ndt.py", selected_map])
    
    return f"Running simulation for {selected_map}..."

if __name__ == '__main__':
    app.run(debug=True)
