import os
import inspect

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0, parentdir)

from MPC_Controller.utils import DTYPE
from MPC_Controller.Parameters import Parameters
from MPC_Controller.robot_runner.RobotRunnerFSM import RobotRunnerFSM
from MPC_Controller.robot_runner.RobotRunnerPolicy import RobotRunnerPolicy
from MPC_Controller.common.Quadruped import RobotType
from RL_Environment import gamepad_reader
from isaacgym import gymapi
from RL_Environment.sim_utils import *

use_gamepad = True
dt =  Parameters.controller_dt
gym = gymapi.acquire_gym()
sim = acquire_sim(gym, dt)
add_ground(gym, sim)
add_terrain(gym, sim, "slope", width=4)
add_terrain(gym, sim, "stair", 3.95, True, width=4)

robot = RobotType.ALIENGO

# mini_cheetah = load_asset(gym, sim, RobotType.MINI_CHEETAH, False)
# a1 = load_asset(gym, sim, RobotType.A1, False)
aliengo = load_asset(gym, sim, RobotType.ALIENGO, False)

asset = aliengo

ctrls = [RobotRunnerFSM(), RobotRunnerPolicy()]
# set up the env grid
num_envs = 4
envs_per_row = 2
env_spacing = 1

env_lower = gymapi.Vec3(-env_spacing, -env_spacing, 0.0)
env_upper = gymapi.Vec3(env_spacing*2, env_spacing, env_spacing)

# cache some common handles for later use
envs = []
actors = []
height = 0.5

# create and populate the environments
for i in range(num_envs):
    env = gym.create_env(sim, env_lower, env_upper, envs_per_row)
    if i%2 == 0:
        envs.append(env)

    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 0.0, height)

    # pose.r = gymapi.Quat(-0.707107, 0.0, 0.0, 0.707107) # rotate -90deg about x
    # pose.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(1, 0, 0), -0.5*math.pi)
    if i%2 == 0:
        actor_handle = gym.create_actor(env, asset, pose, "MyActor", group=i, filter=1)
        actors.append(actor_handle)


cam_pos = gymapi.Vec3(1.4, 2.8, 1.5) # w.r.t target env
# viewer = add_viewer(gym, sim, envs[0], cam_pos)
# add viewer
cam_props = gymapi.CameraProperties()
viewer = gym.create_viewer(sim, cam_props)
# Look at the env
cam_target = gymapi.Vec3(0.8, 0, 0.0)
gym.viewer_camera_look_at(viewer, env, cam_pos, cam_target)

controllers = []
for idx in range(int(num_envs/2)):
    # configure the joints for effort control mode (once)
    props = gym.get_actor_dof_properties(envs[idx], actors[idx])
    props["driveMode"].fill(gymapi.DOF_MODE_EFFORT)
    props["stiffness"].fill(0.0)
    props["damping"].fill(0.0)
    gym.set_actor_dof_properties(envs[idx], actors[idx], props)

    # Setup MPC Controller
    robotRunner = ctrls[idx]
    robotRunner.init(robot)
    controllers.append(robotRunner)

# Setup MPC Controller
if use_gamepad:
    gamepad = gamepad_reader.Gamepad(vel_scale_x=2, vel_scale_y=1.5, vel_scale_rot=3)

count = 0
render_fps = 30
render_count = int(1/render_fps/Parameters.controller_dt)

# simulation loop
while not gym.query_viewer_has_closed(viewer):
    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # current_time = gym.get_sim_time(sim)
    commands = np.zeros(3, dtype=DTYPE)
    if use_gamepad:
        lin_speed, ang_speed, e_stop = gamepad.get_command()
        Parameters.cmpc_gait = gamepad.get_gait()
        Parameters.control_mode = gamepad.get_mode()
        if not e_stop:
            commands = np.array([lin_speed[0], lin_speed[1], ang_speed], dtype=DTYPE)

    # run controllers
    for idx, (env, actor, controller) in enumerate(zip(envs, actors, controllers)):
        dof_states = gym.get_actor_dof_states(env, actor, gymapi.STATE_ALL)
        body_idx = gym.find_actor_rigid_body_index(env, actor, controller._quadruped._bodyName, gymapi.DOMAIN_ACTOR)
        body_states = gym.get_actor_rigid_body_states(env, actor, gymapi.STATE_ALL)[body_idx]
        legTorques = controller.run(dof_states, body_states, commands)
        gym.apply_actor_dof_efforts(env, actor, legTorques / (Parameters.controller_dt*100))

    if Parameters.locomotionUnsafe:
        gamepad.fake_event(ev_type='Key',code='BTN_TR',value=0)
        Parameters.locomotionUnsafe = False

    if count % render_count == 0:
        # update the viewer
        gym.step_graphics(sim)
        gym.draw_viewer(viewer, sim, True)
        
        # Wait for dt to elapse in real time.
        count = 0
    gym.sync_frame_time(sim)

    count += 1

if use_gamepad:
    gamepad.stop()
gym.destroy_viewer(viewer)
gym.destroy_sim(sim)