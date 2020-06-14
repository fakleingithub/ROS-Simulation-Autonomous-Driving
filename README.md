# **ROS-Simulation-Autonmous-Driving** 

**Description**

This simulation is forked from [https://github.com/tum-phoenix] and adapted to the needs of our Carolo Cup Team. 

This repository does not contain the 1:10 autonomous car software (ROS nodes for camera preprocessing, lanedetection, data preprocessing, transverse control etc.).


[//]: # (Image References)

[image1]: ./output_images/itmoves-gazebosimulation.png "itmoves simulation"
[gif1]: ./output_images/ROS-simulation.gif "itmoves simulation recording"

---

### 1. How to run the simulation in your local environment

get ubuntu 18.04 on a virtual machine or primary system

install ros melodic full installation (gazebo is included there)

install this library in terminal:

* `sudo apt-get install ros-melodic-effort-controllers`

Create catkin-workspace [http://wiki.ros.org/catkin/Tutorials/create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) on your ubuntu machine

clone the repository into the src folder 

![Recording of simulation drive with autonomous car software][gif1]

**Important that simulation is working:**

Your `.bashrc` file is located on your home folder and is responsible for executing specific scripts whenever you open a new terminal. You find the script `.bashrc` file by opening a terminal and navigating to your homefolder. You can edit and look at it e.g. with `gedit .bashrc`

add `source /usr/share/gazebo/setup.sh` at the end of your `.bashrc` file before the line `source /opt/ros/melodic/setup.bash` 

add `source ~/catkin_ws/devel/setup.bash` at the end of your `.bashrc` file (if your catkin-ws is located in the home folder)

to launch simulation in autonomous mode set : <arg name="manual" default="false"/> in `/home/fabian/catkin_ws/src/simulation/drive_ros_gazebo_control/launch/gazebo_body_control.launch`

go into the `carolo` folder and run `run_simulation.bash`

first launch file includes all other necessary launch files

You need to write the ROS nodes for camera preprocessing, lanedetection, data preprocessing, transverse control etc. for yourself to drive autonomously in the simulation environment, it's not included in the repository.

Please press "**f**" for autonomous driving (car takes over control)
and press "**r**" for going back to remote control (look at the terminal for manual driving instructions)

![Look inside the simulation][image1]


THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

