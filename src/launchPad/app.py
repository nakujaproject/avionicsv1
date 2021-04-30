from flask import Flask, jsonify, request, json

app = Flask(__name__)
    
@app.route('/avionics/start-logging', methods = ['GET'])
def get_tasks():
    response = {
        "status": "Start"
    }
    return jsonify(response)



@app.route('/avionics/init-done', methods = ['POST'])
def avionics_init_done():
    request_data = json.loads(request.data)
    print(request_data)
    status = request_data['status']
    if status == "Done":
        response = {
            "status": "Success"
        }
    else:
        response = {
            "status": "Failed"
        }
    return jsonify(response)
    
if __name__ == '__main__':
    app.run(debug = True)

