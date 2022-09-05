import argparse
import json
from datetime import datetime

from flask import Flask, request
from flask import send_file

from environment import Environment

parser = argparse.ArgumentParser(description='Server')
parser.add_argument(
    '-hs',
    '--host',
    default='127.0.0.1',
    type=str,
    help='IP address for V-Rep server (default: 127.0.0.1)'
)
parser.add_argument(
    '-p',
    '--port',
    default=5000,
    type=int,
    help='Port for SLAM-Sim server (default: 5000)'
)

namespace = parser.parse_args()

env = Environment()

app = Flask(__name__)


@app.route("/start-v-rep-server", methods=['POST'])
def start_vrep_server():
    vrep_ip = request.args.get('ip')
    port = request.args.get('port')
    env.connect(vrep_ip, int(port))
    if env.client_id != -1:
        time_ = datetime.now().time().isoformat()
        env.print_message(time_ + ': connected')
        answer = 'Successfully connected to V-REP server'
        # print('\033[1;32;40m ' + answer)
        print(answer)
        return answer
    else:
        answer = 'Connection to V-REP server failed'
        # print('\033[1;31;40m ' + answer)
        print(answer)
        return answer


@app.route("/start-simulation", methods=['POST'])
def start_simulation():
    env.start_simulation()
    return 'OK'


@app.route("/pause-simulation", methods=['POST'])
def pause_simulation():
    env.pause_simulation()
    return 'OK'


@app.route("/stop-simulation", methods=['POST'])
def stop_simulation():
    env.stop_simulation()
    return 'OK'

# TODO Rewrite to JSON format
@app.route("/print-message", methods=['POST'])
def print_message():
    text = request.args.get('text')
    env.print_message(text)
    return 'OK'


@app.route("/load-scene", methods=['POST'])
def load_scene():
    scene = request.args.get('scene')
    env.load_scene(str(scene))
    return 'OK'


@app.route("/add-robot", methods=['POST'])
def add_robot():
    model = request.args.get('model')
    env.add_robot(model)
    return 'OK'


@app.route("/forward", methods=['POST'])
def forward():
    robot_id = request.args.get('robot-id')
    speed = request.args.get('speed')
    env.robots[int(robot_id)].forward(float(speed))
    return 'OK'


@app.route("/backward", methods=['POST'])
def backward():
    robot_id = request.args.get('robot-id')
    speed = request.args.get('speed')
    env.robots[int(robot_id)].backward(float(speed))
    return 'OK'


@app.route("/right", methods=['POST'])
def right():
    robot_id = request.args.get('robot-id')
    speed = request.args.get('speed')
    env.robots[int(robot_id)].right(float(speed))
    return 'OK'


@app.route("/left", methods=['POST'])
def left():
    robot_id = request.args.get('robot-id')
    speed = request.args.get('speed')
    env.robots[int(robot_id)].left(float(speed))
    return 'OK'


@app.route("/replace", methods=['POST'])
def replace():
    robot_id = request.args.get('robot-id')
    x = request.args.get('x')
    y = request.args.get('y')
    z = request.args.get('z')
    env.robots[int(robot_id)].replace_robot(float(x), float(y), float(z))
    return 'OK'


@app.route("/get-image/<int:robotid>", methods=['GET'])
def get_image(robotid):
    env.robots[robotid].get_image_from_kinect()
    return send_file('tmp/tmp.gif', mimetype='image/gif')


@app.route("/get-depth/<int:robotid>")
def get_depth_from_kinect(robotid):
    answer = env.robots[robotid].get_depth_from_kinect()
    answer_json = json.dumps(answer)
    return answer_json


@app.route("/get-lidar/<int:robotid>")
def get_lidar_data(robotid):
    answer = env.robots[robotid].get_lidar_data()
    answer_json = json.dumps(answer)
    return answer_json


@app.route("/set-sensor-params", methods=['POST'])
def set_sensor_params():
    robot_id = request.args.get('robot-id')
    sensor_name = request.args.get('sensor-name')
    parameter_name = request.args.get('parameter-name')
    parameter_value = request.args.get('parameter-value')
    res = env.robots[int(robot_id)].set_parameter(
        sensor_name,
        parameter_name,
        float(parameter_value)
    )
    return json.dumps(res)


@app.route("/get-sensor-params/<int:robot_id>/<string:sensor_name>/<string:parameter_name>", methods=['GET'])
def get_sensor_params(robot_id, sensor_name, parameter_name):
    answer = env.robots[int(robot_id)].get_parameter(sensor_name, parameter_name)
    return json.dumps(answer)


@app.route("/get-sensor-params-list/<int:robot_id>", methods=['GET'])
def get_sensor_params_list(robot_id):
    return json.dumps(env.robots[robot_id].sensor_parameters_list())


@app.route("/get-standard-deviation/<int:robot_id>", methods=['GET'])
def get_standard_deviation(robot_id):
    return str(env.robots[robot_id].standard_deviation)


@app.route("/set-standard-deviation", methods=['POST'])
def set_standard_deviation():
    robot_id = request.args.get('robot-id')
    standard_deviation = request.args.get('standard-deviation')
    env.robots[int(robot_id)].standard_deviation = float(standard_deviation)
    return 'OK'


@app.route("/turn-angle", methods=['POST'])
def turn_angle():
    robot_id = request.args.get('robot-id')
    angle = request.args.get('angle')
    speed = request.args.get('speed')
    env.robots[int(robot_id)].turn_angle(float(angle), float(speed))
    return 'OK'


@app.route("/move-distance", methods=['POST'])
def move_distance():
    robot_id = request.args.get('robot-id')
    distance = request.args.get('distance')
    speed = request.args.get('speed')
    env.robots[int(robot_id)].move_distance(float(distance), float(speed))
    return 'OK'


@app.route("/get-robot-position/<int:robot_id>", methods=['GET'])
def get_robot_position(robot_id):
    return json.dumps(env.robots[robot_id].get_position())


@app.route("/get-robot-speed/<int:robot_id>", methods=['GET'])
def get_robot_speed(robot_id):
    return json.dumps(env.robots[robot_id].get_robot_speed())


@app.route("/get-robot-rotation/<int:robot_id>", methods=['GET'])
def get_robot_rotation(robot_id):
    return json.dumps(env.robots[robot_id].get_rotation())


if __name__ == "__main__":
    app.run(namespace.host, namespace.port, debug=True, use_reloader=False)
