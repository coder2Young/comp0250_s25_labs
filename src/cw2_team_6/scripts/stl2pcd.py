#!/usr/bin/env python3

import open3d as o3d 
import rospkg
import os

# script to convert all .STL files in models/ folder to .pcd
if __name__ == "__main__":
    
    # get path of the models folder (change to your team name!)
    base_path = rospkg.RosPack().get_path('cw2_team_x')
    base_path += "/models/"
    ext = ".STL"
    print("converting all .STL in :" + base_path)
    
    # cycle through all .STL files in folder :
    for file in os.listdir(base_path):
        if file.endswith(ext):
            print("convering: " + file)
            mesh = o3d.io.read_triangle_mesh( base_path + file)
            pointcloud = mesh.sample_points_poisson_disk(100000)
            save_name = file.split('.')[0] + ".pcd"
            print("saving: " + save_name)
            o3d.io.write_point_cloud(base_path + save_name , pointcloud)
