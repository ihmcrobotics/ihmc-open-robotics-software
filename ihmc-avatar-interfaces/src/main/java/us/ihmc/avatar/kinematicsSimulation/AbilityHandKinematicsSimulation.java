package us.ihmc.avatar.kinematicsSimulation;

import us.ihmc.handsros2.abilityHand.AbilityHandModel.AbilityHandJointName;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;

public class AbilityHandKinematicsSimulation
{
   private final RevoluteJoint[] joints = new RevoluteJoint[AbilityHandJointName.values.length];

   public AbilityHandKinematicsSimulation(RobotSide side, ROS2Node ros2Node, FullHumanoidRobotModel fullRobotModel)
   {
      for (AbilityHandJointName jointName : AbilityHandJointName.values)
         joints[jointName.ordinal()] = (RevoluteJoint) fullRobotModel.getOneDoFJointByName(jointName.getJointName(side));
   }

   public void update()
   {
      // throttle pubs
      double timeSeconds = System.nanoTime() * 1.0e-9;
      double phase = 2.0 * Math.PI * timeSeconds; // 1 Hz
      double sine01 = 0.5 * (1.0 + Math.sin(phase)); // [0, 1]
      double angle = sine01 * (Math.PI / 2.0);

      for (RevoluteJoint joint : joints)
      {
         joint.setQ(angle);
      }
   }
}
