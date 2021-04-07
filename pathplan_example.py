#path plan example
from Robot import Robot

robot = Robot()
robot.map.load_saved("pathplan_eg.json")
robot.show()
robot.pathplan((200,0),(-80,-120),animation=True)
