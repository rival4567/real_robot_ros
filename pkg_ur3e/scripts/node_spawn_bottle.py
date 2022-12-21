#!/usr/bin/env python3

'''Spawn Coke Can in UR5e gazebo environment.'''

import rospy
import rospkg

from copy import deepcopy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler

# get an instance ofRosPack with the default search paths
rospack = rospkg.RosPack()

path = rospack.get_path('pkg_ur3e')

bottle_model = open(path + '/models/coke_can/model.sdf', 'r').read()


def create_model_request(sdf_model, modelname, px, py, pz, rr, rp, ry):
    model = deepcopy(sdf_model)
    req = SpawnModelRequest()
    req.model_name = modelname
    req.model_xml = model
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req


def main():
    rospy.init_node('spawn_models')
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo("Waiting for /gazebo/spawn_sdf_model service...")
    spawn_srv.wait_for_service()
    rospy.loginfo("Connected to service!")

    req_coke_can = create_model_request(
        bottle_model, "coke_can", 0.45, 0.35, 0.75, 0, 0, 0)
    spawn_srv.call(req_coke_can)
    rospy.loginfo("Spawned Cup!")
    rospy.spin()


if __name__ == '__main__':
    main()
