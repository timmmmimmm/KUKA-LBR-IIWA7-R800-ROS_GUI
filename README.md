# KUKA-LBR-IIWA7-R800-ROS_GUI
This project is the main part of my bachelors thesis focused on creating a proof of concept GUI.
It is a little buggy but it get's the job done

## OS
Ubuntu 20.04 LTS

## ROS Distro
Noetic

## Sunrise.OS verison
1.16

## Instalation
The GUI is powered by the [iiwa_stack developed by IFL_CAMP](https://github.com/IFL-CAMP/iiwa_stack)

### ROS install
Follow all of their steps on how to get the iiwa_stack running on your linux machine and **after you complete Step 3 clone this repo into the src directory** and continue with their guide.
**MAKE SURE TO USE THE EXACT SAME NAMES AS THEY'VE USED** othervise Rviz won't work.

After you've successfully built the workspace, head into **iiwa_stack_ws > src > iiwa_gui > additionalFiles** and copy the contents of **moveit_replacements** into **iiwa_stack_ws > src > iiwa_stack > iiwa_moveit > launch**.

### Sunrise setup
Since I was too lazy to install a symlink onto the src folder of the Java code, you'll have to head again into **iiwa_stack_ws > src > iiwa_gui > additionalFiles > JavaReplacements** and replace the files in **de.tm.in.camp.kuka.ros** with the files from the **ros** folder. And also replace the files in **de.tum.in.camp.kuk.ros.app** with the files in the **ros.app** folder.
