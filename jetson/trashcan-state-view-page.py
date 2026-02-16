from flask import Flask, request, jsonify, render_template_string
from datetime import datetime

app = Flask(__name__)

# 5개의 쓰레기통 상태 초기화
bin_names = ["비닐", "플라스틱", "종이", "일반쓰레기 1", "일반쓰레기 2"]
sensor_status = [
    {"id": name, "status": "empty", "last_update": ""} for name in bin_names
]


# HTML + JavaScript로 실시간 갱신 및 원 색상 표시
page_template = """
<!DOCTYPE html>
<html>
<head>
    <title>Trash Bin Status</title>
    <style>
        body { font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }
        .bin-box { display: inline-block; margin: 20px; padding: 20px; border: 1px solid #ccc; border-radius: 10px; width: 200px; }
        .indicator {
            display: inline-block;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            margin-left: 10px;
        }
        .green { background-color: green; }
        .red { background-color: red; }
    </style>
</head>
<body>
    <h1>Trash Bin Status Monitor</h1>
    <div id="bins"></div>

    <script>
        function updateStatus() {
            fetch('/api/status')
                .then(response => response.json())
                .then(data => {
                    const container = document.getElementById('bins');
                    container.innerHTML = '';
                    data.forEach(bin => {
                        const div = document.createElement('div');
                        div.className = 'bin-box';
                        const circleClass = bin.status === 'full' ? 'red' : 'green';
                        div.innerHTML = `
                            <h3>${bin.id}</h3>
                            <p>Status: ${bin.status}
                                <span class="indicator ${circleClass}"></span>
                            </p>
                            <p>Last Update:<br>${bin.last_update}</p>
                        `;
                        container.appendChild(div);
                    });
                });
        }

        setInterval(updateStatus, 2000);
        window.onload = updateStatus;
    </script>
</body>
</html>
"""

@app.route('/')
def home():
    return render_template_string(page_template)

@app.route("/api/alert", methods=["POST"])
def alert():
    data = request.get_json()

    if data is None or "binary" not in data:
        return {"error": "Missing 'binary' field in request"}, 400

    binary_str = data["binary"]

    if len(binary_str) != len(sensor_status):
        return {"error": "Invalid binary length"}, 400

    for i, bit in enumerate(binary_str):
        status = "empty" if bit == "1" else "full"
        sensor_status[i]["status"] = status
        sensor_status[i]["last_update"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    return jsonify({"message": "Status updated"}), 200



@app.route('/api/status')
def status():
    return jsonify(sensor_status)
@app.route('/test')
def test():
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>Alert 테스트</title>
    </head>
    <body>
        <h2>/api/alert 테스트</h2>
        <button onclick="sendAlert()">상태 업데이트 보내기</button>

        <script>
            function sendAlert() {
                const data = [
                    {"status": "full"},
                    {"status": "empty"},
                    {"status": "full"},
                    {"status": "empty"},
                    {"status": "empty"}
                ];

                fetch("/api/alert", {
                    method: "POST",
                    headers: {
                        "Content-Type": "application/json"
                    },
                    body: JSON.stringify(data)
                })
                .then(response => response.json())
                .then(result => alert("업데이트 성공!"))
                .catch(error => alert("업데이트 실패: " + error));
            }
        </script>
    </body>
    </html>
    '''

if __name__ == '__main__':
    app.run(host="0.0.0.0", port=5002)

