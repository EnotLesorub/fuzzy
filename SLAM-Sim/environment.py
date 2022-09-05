import vrep
from youbot import YouBot

ONE_SHOT_MODE = vrep.simx_opmode_oneshot_wait
STREAM_MODE = vrep.simx_opmode_streaming
BLOCKING_MODE = vrep.simx_opmode_blocking
SCENES_DIR = 'Scenes/'
MODELS_DIR = 'Models/'
POSTFIXES = ['', '#0', '#1', '#2', '#3', '#4', '#5', '#6', '#7', '#8', '#9']


class Environment(object):
    def __init__(self, ip_address='127.0.0.1', port=19997):
        self.ip_address = ip_address
        self.port = port
        self.client_id = None
        # self.scene = self.load_scene('Initial.ttt')
        self.robots = []

    # Public functions
    def connect(self, ip_address='127.0.0.1', port=19997):
        """Connect to v-rep"""
        self.ip_address = ip_address
        self.port = port
        vrep.simxFinish(-1)
        self.client_id = vrep.simxStart(self.ip_address, self.port, True, True, 5000, 5)
        if self.client_id != -1:
            self.load_scene('Simple.ttt')

    def print_message(self, text):
        """Print a message to v-rep console"""
        vrep.simxAddStatusbarMessage(self.client_id, text, ONE_SHOT_MODE)

    def load_scene(self, scene):
        """Load existing scene by it's name"""
        return vrep.simxLoadScene(self.client_id, SCENES_DIR + scene, 0xFF, BLOCKING_MODE)

    def add_robot(self, model):
        """Add new robot to the scene"""
        postfix = self.__get_next_postfix()
        vrep.simxLoadModel(self.client_id, MODELS_DIR + model, 0xFF, BLOCKING_MODE)
        new_youbot = YouBot(self.client_id, postfix)
        self.robots.append(new_youbot)

    def add_block(self, model):
        vrep.simxLoadModel(self.client_id, MODELS_DIR + model, 0xFF, BLOCKING_MODE)

    # Simulation commands
    def start_simulation(self):
        """Start or resume simulation"""
        vrep.simxStartSimulation(self.client_id, ONE_SHOT_MODE)

    def stop_simulation(self):
        """Stop simulation"""
        vrep.simxStopSimulation(self.client_id, ONE_SHOT_MODE)

    def pause_simulation(self):
        """Temporary pause simulation. To resume it use start command."""
        vrep.simxPauseSimulation(self.client_id, ONE_SHOT_MODE)

    # Private functions
    def __get_next_postfix(self):
        """Returns next available postfix for next robot name"""
        for possible_postfix in POSTFIXES:
            if not self.__robot_exists(possible_postfix):
                return possible_postfix
        return '-1'

    def __robot_exists(self, postfix):
        """Returns True if robot with specified id already exists"""
        for robot in self.robots:
            if robot.postfix == postfix:
                return True
        return False
