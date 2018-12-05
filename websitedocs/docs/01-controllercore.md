---
title: Controller Core
---

At the heart of the controller is a Quadratic Program (QP) that considers all objectives given to the controller (balancing, moving a hand, and others). This section gives an overview of the infrastructure surrounding the QP solver. Sometimes the robot will not be able to achieve all commands provided by the Managers. In that case the optimization makes sure they are achieved "as good as possible". We prioritize objectives by assigning a weight to them that can be changed by the Managers depending on the current state of the robot.

Each Manager outputs InverseDynamicsCommands and FeedbackControlComands that can contain desired accelerations for points on the robot, a desired linear momentum rate for maintaining balance, desired joint accelerations, or other desireds. These commands enter the part of our software that we call the "Controller Core". The bock diagram below shows an overview of this. Here, all commands get transformed into a cost function for the QP. From this point they are called motion objectives and momentum objectives. In addition, we try to minimize joint accelerations and the forces the robot exerts on its environment.

The QP will output the joint accelerations necessary to optimally (minimizing the sum of weighted objective costs) achieve all given goals. The result will also contain wrenches the robot exerts on the environment. Using this result and inverse dynamics we obtain desired joint torques. In a perfect world (with fully force controlable robots and perfect tracking) this would be sufficient. However, since some joints of the robot might not be force controlable or have bad force tracking, we also obtain desired joint velocities and positions by integration. In the case of Atlas, we use position control for the arms and a mix of joint torque and velocity control for other parts of the robot.


<a name="controllercore"></a>![Controller Core](/img/documentation/ihmcController/controller_core_overview.png)


For a more detailed description of the framework refer to T. Koolen et al. "Design of a momentum-based control framework and application to the humanoid robot Atlas" *International Journal of Humanoid Robotics* available [here from researchgate](https://www.researchgate.net/publication/280839675_Design_of_a_Momentum-Based_Control_Framework_and_Application_to_the_Humanoid_Robot_Atlas).