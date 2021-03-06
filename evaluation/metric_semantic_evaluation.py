#!/usr/bin/env python

import open3d as o3d
import os
import glog as log
import numpy as np

from evaluation.tools.mesh import Mesh

# Rotation matrices:
# East North Up (ENU) frame to Unity's world frame of reference
enu_R_unity = np.array([[1, 0, 0],
                        [0, 0, 1],
                        [0, 1, 0]])
unity_R_enu = np.transpose(enu_R_unity)

# Right Handed frame to Unity's Left Handed frame of reference
righthand_R_lefthand = np.array([[1, 0, 0],
                                 [0, -1, 0],
                                 [0, 0, 1]])
lefthand_R_righthand = np.transpose(righthand_R_lefthand)

# o3d_R_unity = np.array([[0, -1, 0],
#                         [0, 0, 1],
#                         [1, 0, 0]])
# unity_R_o3d = np.transpose(o3d_R_unity)
# 
# ros_R_o3d = np.array([[1, 0, 0],
#                       [0, 0, -1],
#                       [0, 1, 0]])
# o3d_R_ros = np.transpose(ros_R_o3d)

# Rviz however uses a complicated and non-intuitive workaround where the default coordinate convention of Ogre is 
# used, but all Vectors and Quaternions coming from ROS and going back 
# need to be converted using rviz::robotToOgre and rviz::ogreToRobot respectively (see rviz/common.h).
#  Also, this is documented nowhere. What they do is basically this (though expressed more complicated in
#  the source code):
# ogreToRos(x,y,z) = (-z,-x,y)
# rosToOgre(x,y,z) = (-y,z,-x)
# ros_R_ogre = np.array([[0,  0, -1],
#                        [-1, 0,  0],
#                        [0,  1,  0]])
# ogre_R_ros = np.transpose(ros_R_ogre)

class MeshEvaluator:
    def __init__(self, est_mesh, gt_mesh):
        print("Init MeshEvaluator")
        self.est_mesh = est_mesh
        self.gt_mesh = gt_mesh

    def visualize_meshes(self):
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.get_render_option().mesh_show_back_face = True
        self.gt_mesh.add_to_vis(vis)
        self.est_mesh.add_to_vis(vis)
        mesh_frame = o3d.geometry.create_mesh_coordinate_frame(size=4,
                                                               origin=[0, 0, 0])
        vis.add_geometry(mesh_frame)
        vis.run()
        vis.destroy_window()

    def align_meshes(self, guess_tf):
        """ Aligns both meshes using ICP given a good guess of what is the initial registration """
        print("Align...")


if __name__ == '__main__':
    import argcomplete
    import sys

    def parser():
        import argparse
        basic_desc = "Full evaluation of SPARK VIO pipeline (APE trans + RPE trans + RPE rot) metric app"

        shared_parser = argparse.ArgumentParser(
            add_help=True, description="{}".format(basic_desc))

        input_opts = shared_parser.add_argument_group("input options")

        input_opts.add_argument("path_to_gt_ply", help="Path to the ground-truth ply file with the mesh.",
                                default="./gt_mesh.ply")
        input_opts.add_argument("path_to_est_ply", help="Path to the estimated ply file with the mesh.",
                                default="./est_mesh.ply")

        main_parser = argparse.ArgumentParser(
            description="{}".format(basic_desc))
        sub_parsers = main_parser.add_subparsers(dest="subcommand")
        sub_parsers.required = True
        return shared_parser

    log.setLevel("INFO")
    parser = parser()
    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    print("Loading Ground-truth mesh...")
    est_mesh = Mesh(args.path_to_est_ply)
 #    - 2 * np.asarray(est_mesh.mesh_o3d.vertices[1])
    est_mesh.mesh_o3d.translate([0, -5, 0])
    #est_mesh.visualize()
    gt_mesh = Mesh(args.path_to_gt_ply)
    print("Transforming gt_mesh from Left Hand frame of reference (Unity) to Right Hand (ROS)")
    #gt_mesh.transform_left(enu_R_unity)
    gt_mesh.transform_right(righthand_R_lefthand)
    print("Loading Estimated mesh...")
    mesh_eval = MeshEvaluator(gt_mesh, est_mesh)
    mesh_eval.visualize_meshes()
