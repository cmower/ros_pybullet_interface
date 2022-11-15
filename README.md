# ROS PyBullet Interface

The ROS-PyBullet Interface is a framework that provides a bridge between the popular Robot Operating System (ROS) with a reliable impact/contact simulator PyBullet.
Furthermore, this framework provides several interfaces that allow humans to interact with the simulator that facilitates Human-Robot Interaction in a virtual world.

The documentation can be found [here](https://ros-pybullet.github.io/ros_pybullet_interface/).

# Requirements

* ROS Noetic
* Ubuntu 20.04
* Python 3

# Install

1. [Create a catkin workspace.](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace)
2. `cd` into the `src` directory.
3. Clone this repository: `git clone https://github.com/ros-pybullet/ros_pybullet_interface.git`
4. `cd ros_pybullet_interface`
5. Inspect `install.sh`, and if you are happy then run: `bash install.sh`

Now, you should be able to run the examples.

# Cite

If you use the interface in your work, please consider citing us.

```bibtex
@article{Mower2022,
  author = {Mower, Christopher E. and Stouraitis, Theodoros and Moura, João and Rauch, Christian and Yan, Lei and Behabadi, Nazanin Zamani and Gienger, Michael and Vercauteren, Tom and Bergeles, Christos and Vijayakumar, Sethu},
  title = {ROS-PyBullet Interface: A Framework for Reliable Contact Simulation and Human-Robot Interaction},
  journal = {[to appear] Proceedings of the Conference on Robot Learning},
  year = {2022},
}
```
