# Robotic Mosaic

MRAC Workshop 2.1
![](https://lh5.googleusercontent.com/7N1xqWCYj6u-g3KU0YsjYWCkiqHlNRFmo_k84CAUVT5GTmduvSVPjVW53jRiStTtjOCt-3EGmJEw9lkKEU2yOu2PionNbrTUy8YlCUxKCi9Xb_1kI3fdsrv-VC0ALj_vI7uhKtOA)Image by [LoggaWiggler](https://pixabay.com/users/loggawiggler-15/?utm_source=link-attribution&utm_medium=referral&utm_campaign=image&utm_content=5249) from [Pixabay](https://pixabay.com/?utm_source=link-attribution&utm_medium=referral&utm_campaign=image&utm_content=5249)  

In a world of excessive consumption, new solutions are needed to recycle and revalorize our waste. In particular, the construction industry responsible for up to 30% of all waste is currently looking at alternative models based on circular economy. Yet ancestral craftsman techniques have been recycling construction material, in particular stone and ceramic material into mosaic art.

Mosaic tessellation has a long history, starting in Mesopotamia in the 3rd millennium BC. Ancient crafts techniques combined with actual industrial processes are an amazing way to explore novel approaches and innovative outputs. Gaining a deeper understanding of those techniques and designs they generate can provoke an innovative line of thinking and designing. It becomes particularly interesting when ancient processes are linked to technology within a creative robotic framework.

Industrial robotic arms are very precise devices, used to carry out complex tasks in known and controlled environments. What if an industrial robotic arm had to perform different tasks in unknown environments?

In recent years, vision devices have evolved from being a device for taking pictures and saving memories to being the sense of sight for machines. Now, machines can see, orient themselves, make decisions and carry out tasks for which they were not explicitly programmed.

In this way, robots acquire a layer of intelligence that they did not have when they were used as reproducers of pre-programmed tasks, giving the robots a certain autonomy that allows them to act in new situations, new environments without supervision or having to be reprogrammed.

![](https://lh4.googleusercontent.com/ZzZ2FYkT1TQHTAvedpN-7wzIqbB2l5ecFv0UKEHn8beS3hwu6S6riemlus_9Q626KXVwBsCQbAbJ3PLHtkq6lW18rnZaqmWsWSDOy8-QChsRXB5Ll6KJrWqJIRwACuI5b2Sje9QN)
In this workshop we will see different vision sensors in 2D and 3D formats. How to use them, extract the information they provide us and analyze the provided information to be able to use it in different applications.

At the same time, we will see how to control the robotic arm in a real-time closed loop.

The application generated at the end of the workshop will be assembly of a mosaic using recycled leftover pieces where the closed loop system will guide the robotic arm to choose the right piece in each step of the process.

**Learning Objectives**

At course completion the student will:

-   Apply computer vision knowledge into a specific application of fabrication
    
-   Know how create a closed feedback loop system for robot arm processes
    
-   Learn and define Mass customized design based on material input.
    
-   Apply Robotic Technology to Circular Economy context.
    
-   Explore mosaic tessellation technique


Faculty – Alexandre Dubor, Angel Muñoz, Soroush Garivani  



## WORKFLOW

![Workflow](/images/diagram.jpg)

The robotic application developed for this workshop is based on [Compas Fab](https://gramaziokohler.github.io/compas_fab/latest/). A **Robotic fabrication package for the COMPAS Framework** that facilitates the planning and execution of robotic fabrication processes. It provides interfaces to existing software libraries and tools available in the field of robotics (e.g. OMPL, ROS) and makes them accessible from within the parametric design environment. The package builds upon [COMPAS](https://compas.dev/), an open-source Python-based framework for collaboration and research in architecture, engineering and digital fabrication.

![Workflow Extended](/images/diagram_extended.jpg)

### Setup
![Setup](/images/setup.jpg)

In our application we use Rhino and Grasshopper as CAD software interfacing through a websocket with the robotic platform ROS, Universal Robots ROS driver to control the robot, Moveit as motion planner and a Realsense D435 RGBD camera as sensor to detect the pieces contours using the openCV library.

![openCv](/images/opencv.jpg)

The system is divided in 3 main parts, 2 computers and a Universal Robots UR10e. 



## WINDOWS COMPUTER

### Requirements

* Minimum OS: Windows 10 Pro
* [Anaconda 3](https://www.anaconda.com/distribution/)
* [Rhino 6/7 & Grasshopper](https://www.rhino3d.com/download)
* [Visual Studio Code](https://code.visualstudio.com/): Any python editor works, but we recommend VS Code + extensions [as mentioned in the compas fab documentation](https://gramaziokohler.github.io/compas_fab/latest/getting_started.html#working-in-visual-studio-code-1)

### Compas Fab installation

We use `conda` to make sure we have clean, isolated environment for dependencies. Open Anaconda Prompt.

If it's your first time using conda make sure you run this at least once:

    (base) conda config --add channels conda-forge

Create a new conda environment and activate it:

    (base) conda create -n ws21 python=3.8 compas_fab
    (base) conda activate ws21

### Verify installation

        (ws21) python -m compas
    Yay! COMPAS is installed correctly!
    
    COMPAS: 1.7.1
    Python: 3.8.10 | packaged by conda-forge | (default, May 11 2021, 06:25:23) [MSC v.1916 64 bit (AMD64)]
    Extensions: ['compas-fab']

### Install on Rhino

    (ws21) python -m compas_rhino.install

    
## UBUNTU 18.04 COMPUTER

### Requirements

* OS: [Ubuntu 18.04](https://releases.ubuntu.com/18.04.5/) - [Installation guide](https://phoenixnap.com/kb/how-to-install-ubuntu-18-04)

### Setting up Ubuntu with a real-time kernel
In order to run the `universal_robot_driver`, we highly recommend to setup a ubuntu system with real-time capabilities. Especially with a robot from the e-Series the higher control frequency might lead to non-smooth trajectory execution if not run using a real-time-enabled system.

Follow this [guide](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/real_time.md) in order to compile the ubuntu real-time kernel

### ROS melodic full desktop installation

#### Configure your Ubuntu repositories
 Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." You can [follow the Ubuntu guide](https://help.ubuntu.com/community/Repositories/Ubuntu) for instructions on doing this.

#### Setup your sources.list
Setup your computer to accept software from packages.ros.org.

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#### Set up your keys

    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

#### ROS melodic installation

    sudo apt update
    sudo apt install ros-melodic-desktop-full

#### Environment setup
It's convenient if the ROS environment variables are automatically added to your bash session every time a new shell is launched:

    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    
#### Create a ROS Workspace

Let's create and build a [catkin workspace](http://wiki.ros.org/catkin/workspaces):

    mkdir -p ~/ur_ws/src
    cd ~/ur_ws/
    catkin_make
    source devel/setup.bash

#### Rosbridge installation

Rosbridge provides a JSON API to ROS functionality for non-ROS programs. There are a variety of front ends that interface with rosbridge, including a WebSocket server for web browsers to interact with. Rosbridge_suite is a meta-package containing rosbridge, various front end packages for rosbridge like a WebSocket package, and helper packages.

    sudo apt-get install ros-melodic-rosbridge-server

#### MoveIt installation
Open-source robotic manipulation framework that allows to develop complex manipulation applications using ROS.

`sudo apt install ros-melodic-moveit`

#### Universal Robots ROS Driver installation
[Driver](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) that allow us to control a UR robot from ROS.

    # source global ros
    $ source /opt/ros/melodic/setup.bash
    
    # create a catkin workspace
    $ mkdir -p ur_ws/src && cd ur_ws
    
    # clone the driver
    $ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
    
    # clone fork of the description. This is currently necessary, until the changes are merged upstream.
    $ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot
    
    # install dependencies
    $ sudo apt update -qq
    $ rosdep update
    $ rosdep install --from-paths src --ignore-src -y
    
    # build the workspace
    $ catkin_make
    
    # activate the workspace (ie: source it)
    $ source devel/setup.bash

#### RealSense D435 camera installation
**[Install](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages)** the latest release including the Intel RealSense SDK, Viewer and Depth Quality tools.

Install he python wrapper for Intel RealSense SDK 2.0 provides the C++ to Python binding required to access the SDK.

    pip3 install pyrealsense2


#### openCV installation
To install OpenCV from the Ubuntu 18.04 repositories, follow these steps:

1.  Refresh the packages index and install the OpenCV package by typing:
    
    ```
    sudo apt update
    sudo apt install python3-opencv
    ```
    

    
-   To verify the installation, import the `cv2` module and print the OpenCV version:
    
    ```
    python3 -c "import cv2; print(cv2.__version__)"
    ```
    

	```output
	3.2.0
	```

## UR robot setup for ur_robot_driver

### Prepare the robot

For using the _ur_robot_driver_ with a real robot you need to install the **externalcontrol-1.0.4.urcap**

**Note**: For installing this URCap a minimal PolyScope version of 3.7 or 5.1 (in case of e-Series) is necessary.

For installing the necessary URCap and creating a program, please see the individual tutorial on how to [setup an e-Series robot](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_e_series.md).

### Prepare the ROS PC (Ubuntu 18.04 computer)

For using the driver make sure it is installed.

#### Extract calibration information

Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also make use of this in ROS, you first have to extract the calibration information from the robot.

Though this step is not necessary to control the robot using this driver, it is highly recommended to do so, as otherwise endeffector positions might be off in the magnitude of centimeters.

For this, there exists a helper script:

```
$ roslaunch ur_calibration calibration_correction.launch \
  robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"

```

For the parameter `robot_ip` insert the IP address on which the ROS pc can reach the robot. As `target_filename` provide an absolute path where the result will be saved to.

#### Quick start

Once the driver is built and the **externalcontrol** URCap is installed on the robot, you are good to go ahead starting the driver. (**Note**: We do recommend, though, to [extract your robot's calibration](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#extract-calibration-information) first.)

To actually start the robot driver use one of the existing launch files

```
$ roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.56.101

```

If you calibrated your robot before, pass that calibration to the launch file:

```
$ roslaunch ur_robot_driver <robot_type>_bringup.launch robot_ip:=192.168.56.101 \
  kinematics_config:=$(rospack find ur_calibration)/etc/ur10_example_calibration.yaml

```

If the parameters in that file don't match the ones reported from the robot, the driver will output an error during startup, but will remain usable.

For more information on the launch file's parameters see its own documentation.

Once the robot driver is started, load the [previously generated program](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#prepare-the-robot) on the robot panel that will start the _External Control_ program node and execute it. From that moment on the robot is fully functional. You can make use of the _Pause_ function or even _Stop_ (⏹️) the program. Simply press the _Play_ button (▶️) again and the ROS driver will reconnect.

Inside the ROS terminal running the driver you should see the output `Robot ready to receive control commands.`

To control the robot using ROS, use the action server on

/scaled_pos_joint_traj_controller/follow_joint_trajectory

Use this with any client interface such as [MoveIt!](https://moveit.ros.org/) or simply the `rqt_joint_trajectory_controller` gui:

```
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller

```

You may need to install rqt_joint_trajectory_controller by running:

```
sudo apt install ros-melodic-rqt-joint-trajectory-controller

```


## USAGE



1. **ROS side**
In the Ubuntu 18.04 computer execute one the scripts you will find in the bash_scripts folder.

    ```bash
    #for just simulation
    ./ros_compas_fab.sh
    ```
    or

    ```bash
    #to control a real robot
    ./ros_compas_fab_live.sh
    ```

	Both files launch different ROS nodes:

	 - **roscore**, a collection of basic ros nodes like ROS Master, ROS Parameter Server and rosout logging node.
	 - **rosbridge**, to communicate with Grasshopper.
	 - **fileserver**, to exchange files.
	 - **ur_robot_driver**, to control the real robot.(only in the live bash script)
	 - **MoveIt** for the path planning
	 - **cv_basics** to get the pieces contour points and color.

2. **CAD side**
Open the GH file from the cad folder. The Grasshopper definition contains different groups of components, each of them related with a different functionality.

	**ROS client**
	It connects to ROS, just insert the IP of the ROS computer and set to True the connect button.

	**Robot setup**
	 - **Load robot from ROS**. It loads the URDF model from ROS, it creates robot model from URDF and loads the robot geometry.
	 - **Load tool to ROS**. It sends to MoveIt the tool geometry.
	 - **Load collision scene to ROS**. It sends to Moveit the scene geometries to avoid collisions.

	**Read Contours**
	It is used to subscribe to the ROS topics. /contours_rs,  /image_size and  /colors_rs. Once subscribed we get the contours points, and colors to feed our mosaic design.

	**Design**
	In this part is where the pick and place area are defined. Also, in this area is where our mosaic design algorithm should be embedded to be assembled.

	**Target commands**
	This group takes all the pick and place frames and merges them in one list together with the tool commands.

	**Request Path Planning**
	All the pick and place frames are send to Moveit to get all the robot joint trajectories to assemble our mosaic design. 

	**Execution**
	This part executes all the calculated joint trajectories in the real robot together with the tool commands. The tool is controlled through the ROS service /ur_hardware_interface/set_io.
	
	Subscribe to the /joint_states to get the actual robot joints values and visualize it.

	**Forward kinematics & Homing**
	Use this part to home the robot or to move the robot to any desired position using the provided sliders.

	**Visualization**
	Display the robot model in the Rhino workspace and also allows to simulate the motion plans.

3. **UR10e side**

	On the pendant load the URCaps program for external control and execute it by pressing the play button. Inside the ROS terminal running the driver you should see the output 
	`Robot ready to receive control commands.`


## Projects

![Result](/images/result.jpg)

Applications envisioned and developed by the students during the workshop.

[LINK](http://www.iaacblog.com/programs/courses/mrac/2020-2021-mrac/w-2-1-workshop-mrac-2020-2021-2nd/)


## References

 - https://www.ros.org
 - https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
 - https://opencv.org/ https://www.intelrealsense.com/depth-camera-d435/
 - https://compas.dev/index.html
 - https://gramaziokohler.github.io/compas_fab/latest/
 - https://github.com/compas-dev/compas_fab

## Acknowledges 

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45">

<!-- 
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

More details: <a href="https://iaac.net/rosin-new-robotic-setup/"> in this link. </a>
