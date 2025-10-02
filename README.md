## Docker
Very briefly, Docker is a tool that provides a simple and efficient way to pack everything needed for a specific application in one container. You can also look at it as a lightweight virtual machine running on your computer.

Basic information about Docker and its main concepts can be found [here](https://github.com/larics/docker_files/wiki). Of course, you can also take a look at the [official website](https://www.docker.com/). Don't follow any instructions from these links just yet. They are provided as a general overview and reference you can use in the future. Detailed step-by-step instructions are given below.

#### Prerequisites
You must have Ubuntu OS installed on your computer. Ideally, this would be Ubuntu 24.04, but another version should work as well. 

#### Step-by-step instructions
Follow these [instructions](https://docs.docker.com/engine/install/ubuntu/) to install the Docker engine.

Then follow these [optional steps](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) to manage docker as a non-root user. If you skip this, every `docker` command will have to be executed with `sudo`. Skip the _"Note: To run Docker without root privileges, see Run the Docker daemon as a non-root user (Rootless mode)."_ part. This is just a note and we do not need it.

Docker containers are intended to run inside your terminal. In other words, you won't see a desktop like in regular virtual machines. However, graphical applications can still run in the container if you give them permission. To do that, execute
```bash
xhost +local:docker
```
To avoid having to do this every time, we can add that command to our `.profile` file which executes on every login.
```bash
echo "xhost +local:docker > /dev/null" >> ~/.profile
```

If you have an NVIDIA GPU, please install `nvidia-container-toolkit` by following [these instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).


## Run Crazyflies in the Docker container

### Setting up

Clone the this repository:
```
git clone git@github.com:larics/icuas25_competition.git
```
Add  to  `~/.bashrc` and source it, or type in the current terminal:
```
export DOCKER_BUILDKIT=1
```
Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
docker build --ssh default -t crazysim_icuas_img .

# Run the crazysim_img2 container for the fist time
./first_run.sh

# This will create docker container crazysim_icuas_cont and position you into the container

```

For subsequent use of the container, you can use:
```bash
# Start the crazysim_icuas_cont:
docker start -i crazysim_icuas_cont

```

The following docker commands can also be helpful:
```bash
# Start another bash shell in the already running crazysim_icuas_cont container:
docker exec -it crazysim_icuas_cont bash

# Stop the container
docker stop crazysim_icuas_cont

# Delete the container
docker rm crazysim_icuas_cont

```

If you remove the container, you can always rebuild it from scratch by using `first_run.sh` script. 


The containers `crazysim_icuas_cont` consists of packages for Crazyflies simulator [CrazySim](https://github.com/gtfactslab/CrazySim). General information about Crazyflies can be found [here](https://www.bitcraze.io/products/crazyflie-2-1/).

> [!NOTE]
> The ros2 workspace is located in /root/CrazySim/ros2_ws

### RUN EMPTY WORLD EXAMPLE

Once inside the container, navigate to `/root/CrazySim/ros2_ws/src/icuas25_competition/startup` (you can use alias `cd_icuas25_competition` that will place you in icuas25_competition package). Start the example: 

```
./start.sh
```

If needed, make startup script executable with `chmod +x start.sh`. It starts the example with 5 Crazyflies and 5 aruco markers in an empty world. To test that everything is working, in the first pane of the second window you can start `teleop_twist`, which is already in history. It controls the Crazyflie with id `cf_1`.

### RUN CITY_1 WORLD EXAMPLE
Once inside the container, navigate to `/root/CrazySim/ros2_ws/src/icuas25_competition/startup`. Edit the `_setup.sh` script to export `ENV_NAME=city_1_world`, then start everything with:
```
./start.sh
```
> [!NOTE]
> This will not spawn aruco markers. Place them on your own to test!

#### Interesting topics

* `cf_x/odom` - odometry (can be used for feedback)
* `cf_x/image` - image from camera (the name of the topic can be changed)
* `cf_x/battery_status` - percentage of the battery and general state

#### How can crazyflies be controlled:
* `cf_x/cmd_attitude` - low level controller
* `cf_x/cmd_hover` - velocities for horizontal hovering, height can be changed by dynamically changing hovering height 
* `cf_x/start_trajectory`,`cf_x/upload_trajectory` - services for executing trajectories
* `cf_x/go_to` - service to define point to which cf should go
* `cf_x/cmd_vel` - horizontal velocity control, vel_mux.py node subscrbes to it and publishes to `cf_x/cmd_hover`

If you are working in the group and you are all using the same network, please check [ROS_DOMAIN_ID](https://docs.ros.org/en/eloquent/Tutorials/Configuring-ROS2-Environment.html#the-ros-domain-id-variable) in `.bashrc` in the container. Random number should be set during the build, however it is possible that some of you got the same number. If that is the situation please change it, so that your simulations do not crash.

## Creating and working with Octomaps
If you install `octomap_ros` package, and all its derivatives for ROS, you will be good to go to attempt to convert 3D models into octomaps. We have included a binary that converts an `.stl` file into a `.binvox` file. It is located in `scripts` folder and is called `binvox`. You can use it as follows:
```bash
# Convert stl to binvox
./binvox -e res path_to_stl_file

```
This will result in a `.binvox` file. As Antun said: `res is resolution in voxels, the larger this number better the resolution. Note that this greatly impacts memory usage and process might not even start. For res=4096 on a 26MB .stl file it used around 65GB of RAM.`. For the current version of the `city_1` Octomap, we used `res` of 1000. See https://github.com/larics/larics_gazebo_worlds for more examples and some troubleshooting guidelines. 

Once you have the `.binvox` file, you can use the tool `binvox2bt` provided by the `octomap` package to convert to binary tree structure that can be loaded by Octomap server:
```bash
# Convert stl to binvox
binvox2bt --bb <minx> <miny> <minz> <maxx> <maxy> <maxz> path_to_file

```
If you do not want to crop the model, you can omit the bounding box. The command will result in a file with extension `.binvox.bt`, this file can then be loaded in an octomap server ROS node. An example launch file for starting the octomap server node is included in the `launch` folder. 

Once the octomap server node is running, it exposes two services: `/octomap_binary` and `/octomap_full`, which can be used to access the octomap from another ROS node. 


## Bonus section
The provided Docker image comes with a few preinstalled tools and configs which may simplify your life.

**Tmuxinator** is a tool that allows you to start a tmux session with a complex layout and automatically run commands by configuring a simple yaml configuration file. Tmux is a terminal multiplexer - it can run multiple terminal windows inside a single window. This approach is simpler than having to do `docker exec` every time you need a new terminal.

You don't need to write new configuration files for your projects, but some examples will use Tmuxinator. You can move between terminal panes by holding down `Ctrl` key and navigating with arrow keys. Switching between tabs is done with `Shift` and arrow keys. If you have a lot of open panes and tabs in your tmux, you can simply kill everything and exit by pressing `Ctrl+b` and then `k`.

Here are some links: [Tmuxinator](https://github.com/tmuxinator/tmuxinator), [Getting starded with Tmux](https://linuxize.com/post/getting-started-with-tmux/), [Tmux Cheat Sheet](https://tmuxcheatsheet.com/)

**Ranger** is a command-line file browser for Linux. While inside the Docker container, you can run the default file browser `nautilus` with a graphical interface, but it is often easier and quicker to view the files directly in the terminal window. You can start ranger with the command `ra`. Moving up and down the folders is done with arrow keys and you can exit with a `q`. When you exit, the working directory in your terminal will be set to the last directory you opened while in Ranger.

**Htop** is a better version of `top` - command line interface task manager. Start it with the command `htop` and exit with `q`.

**VS Code** - If you normally use VS Code as your IDE, you can install [Dev Containers](https://code.visualstudio.com/docs/remote/containers#_sharing-git-credentials-with-your-container) extension which will allow you to continue using it inside the container. Simply start the container in your terminal (`docker start -i crazysim_icuas_cont `) and then attach to it from the VS code (open action tray with `Ctrl+Shift+P` and select `Dev Containers: Attach to Running Container`).
