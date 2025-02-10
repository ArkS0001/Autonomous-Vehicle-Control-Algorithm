from flask import Flask, render_template, request
import subprocess

app = Flask(__name__)

# List of available maps
MAPS = {
    "map1": [
        {"x": 10.0, "y": 15.0, "length": 10, "width": 3},
        {"x": 40.0, "y": 0.0, "length": 2, "width": 10}
    ],
    "map2": [
        {"x": 10.0, "y": -10.0, "length": 5, "width": 5},
        {"x": 30.0, "y": 15.0, "length": 5, "width": 2}
    ],
    "map3": [
        {"x": 50.0, "y": 15.0, "length": 5, "width": 2},
        {"x": 25.0, "y": -10.0, "length": 2, "width": 6}
    ]
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
