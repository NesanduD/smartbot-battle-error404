
from flask import Flask, request, jsonify, render_template

from flask_cors import CORS  # Import CORS

app = Flask(__name__)
CORS(app)  # Enable CORS for all routes

scores = {"R1": 0, "R2": 0}

@app.route('/score', methods=['POST'])
def update_score():
    data = request.get_json()
    robot = data.get('robot')
    event = data.get('event')
    score = data.get('score')

    if robot in scores:
        scores[robot] += score
        return jsonify({"status": "success", "scores": scores})
    return jsonify({"status": "failure", "message": "Invalid robot"}), 400

# API to get scores
@app.route('/get_scores', methods=['GET'])
def get_scores():
    return jsonify({"scores": scores})

# Route to show the scoreboard HTML page
@app.route('/scoreboard')
def scoreboard():
    return render_template("scoreboard.html")

if __name__ == "__main__":
    app.run(debug=True)
