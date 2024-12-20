from robot_descriptions.loaders.robomeshcat import load_robot_description
import robomeshcat

q0 = [-1.5, 0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0]
q0  = [-0.1, 0.8, -1.5, 0.1, 0.8, -1.5, -0.1, 1.0, -1.5, 0.1, 1.0, -1.5] # Above but corrected for joint order

robot = load_robot_description("go2_description")

scene = robomeshcat.Scene()
scene.add_robot(robot)
robot.pos = [0, 0, 0.4]
robot[:] = q0
print(dir(robot))

with scene.animation(fps=30):
    scene.render()