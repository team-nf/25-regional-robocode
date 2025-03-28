
from flask import Flask, request, jsonify
from solver import generate_trajectory

app = Flask(__name__)

# Load config once
import json
with open("robot_config.json") as f:
    robot_config = json.load(f)

@app.route("/generate", methods=["POST"])
def generate():
    data = request.get_json()
    poses = [data["from"], data["to"]]
    trajectory = generate_trajectory(poses, robot_config)
    return jsonify(trajectory)

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
