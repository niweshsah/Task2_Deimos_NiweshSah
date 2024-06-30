#!/usr/bin/env python3

import subprocess
import time

def find_package(package_name):
    # Find the package path using rospack
    try:
        package_path = subprocess.check_output(['rospack', 'find', package_name]).strip()
        return package_path.decode('utf-8')
    except subprocess.CalledProcessError:
        print(f"Package '{package_name}' not found.")
        return None

def start_roscore():
    # Start roscore in a new terminal window
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'roscore; exec bash'])


def launch_gazebo(package_path):
    # Launch turtlebot3_gazebo from found package path
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f"roslaunch {package_path}/launch/turtle_bookstore2.launch; exec bash"])

def launch_map():
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f"roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping; exec bash"])

def launch_teleop():
    
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', "roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch; exec bash"])

def main():
    start_roscore()
    time.sleep(3)  # Wait for roscore to initialize (adjust delay as necessary)


    package_name = "turtlebot3_gazebo"
    package_path = find_package(package_name)
    
    if package_path:
        print(f"Package '{package_name}' found at: {package_path}")
        
        # Launch turtlebot3_gazebo
        launch_gazebo(package_path)
        
        # Launch turtlebot3_slam with gmapping
        launch_map()
        
        # Launch turtlebot3_teleop
        launch_teleop()
    else:
        print("Unable to launch nodes because package was not found.")

if __name__ == "__main__":
    main()



