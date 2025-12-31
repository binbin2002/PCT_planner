import sys
import argparse
import numpy as np

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

from utils import *
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config

parser = argparse.ArgumentParser()
parser.add_argument('--scene', type=str, default='Spiral', help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\']')
args = parser.parse_args()

cfg = Config()


if args.scene == 'Spiral':
    tomo_file = 'spiral0.3_2'
    start_pos = np.array([-16.0, -6.0], dtype=np.float32)
    end_pos = np.array([-26.0, -5.0], dtype=np.float32)
elif args.scene == 'Building':
    tomo_file = 'building2_9'
    start_pos = np.array([5.0, 5.0], dtype=np.float32)
    end_pos = np.array([-6.0, -1.0], dtype=np.float32)
else:
    tomo_file = 'plaza3_10'
    start_pos = np.array([0.0, 0.0], dtype=np.float32)
    end_pos = np.array([23.0, 10.0], dtype=np.float32)

path_pub = rospy.Publisher("/pct_path", Path, latch=True, queue_size=1)
planner = TomogramPlanner(cfg)

current_start_pos = None
current_goal_pos = None
tomo_loaded = False
last_planned_start = None
last_planned_goal = None

def amcl_callback(msg):
    global current_start_pos
    position = msg.pose.pose.position
    current_start_pos = np.array([position.x, position.y,position.z], dtype=np.float32)
    rospy.loginfo(f"Updated start position: {current_start_pos}")

def goal_callback(msg):
    global current_goal_pos

    position = msg.pose.position
    current_goal_pos = np.array([position.x, position.y,position.z], dtype=np.float32)
    rospy.loginfo(f"Received new goal: {current_goal_pos}")


def pct_plan():
    planner.loadTomogram(tomo_file)
    traj_3d = planner.plan(start_pos, end_pos)
    if traj_3d is not None:
        path_pub.publish(traj2ros(traj_3d))
        print("Trajectory published")

def poses_equal(a, b, tol=1e-2):
    if a is None or b is None:
        return False
    return np.linalg.norm(np.asarray(a) - np.asarray(b)) < tol


def do_periodic_plan(event):
    global tomo_loaded, last_planned_start, last_planned_goal

    # 只在起点和终点都存在且发生变化时规划（避免频繁重复）
    if current_start_pos is None or current_goal_pos is None:
        return

    if poses_equal(current_start_pos, last_planned_start) and poses_equal(current_goal_pos, last_planned_goal):
        return

    # 延迟加载 tomogram（仅第一次）
    if not tomo_loaded:
        try:
            rospy.loginfo("Loading tomogram: %s", tomo_file)
            planner.loadTomogram(tomo_file)
            tomo_loaded = True
        except Exception as e:
            rospy.logerr("Failed to load tomogram: %s", e)
            return
    rospy.loginfo("Planning with start: %s, goal: %s", current_start_pos, current_goal_pos)
    #rospy.loginfo("Planning from [%.3f, %.3f] -> [%.3f, %.3f]",current_start_pos[0], current_start_pos[1],current_goal_pos[0], current_goal_pos[1])

    try:
        traj_3d = planner.plan(current_start_pos, current_goal_pos)
    except Exception as e:
        rospy.logerr("planner.plan exception: %s", e)
        return

    if traj_3d is not None:
        path_pub.publish(traj2ros(traj_3d))
        rospy.loginfo("Trajectory published")
        last_planned_start = current_start_pos.copy()
        last_planned_goal = current_goal_pos.copy()
    else:
        rospy.logwarn("Planning failed or returned empty trajectory.")

def pct_plan_node():
    rospy.init_node("pct_planner", anonymous=True)
    # 订阅机器人位姿和 RViz goal
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, amcl_callback, queue_size=1)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback, queue_size=1)
    # 定时器：每 2 秒检查一次并在需要时规划（可按需调整间隔）
    timer = rospy.Timer(rospy.Duration(2.0), do_periodic_plan)

    rospy.loginfo("PCT planner node ready. Publish goal via RViz '2D Nav Goal' (topic /move_base_simple/goal).")
    rospy.spin()


if __name__ == '__main__':
    pct_plan_node()
   
    