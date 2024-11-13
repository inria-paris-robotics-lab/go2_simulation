# Unitree Go2 ros2 simulation 

This ros2 package allows you to simualate the go2 robot in pybullet. It replicates some of the real robot topics.

Installation
------------

For this to work you will need to have installed the unitree_ros stack (please find instructions, [here](https://github.com/unitreerobotics/unitree_ros2/tree/master)). You will also need pybullet, it is recommended to install it in a conda environment:

```
conda create -n pybullet_ros python=3.10
conda activate pybullet_ros
conda install pybullet
```

After this you can do `colcon build` at the root of your ros workspace, and you should be able to launch this package.