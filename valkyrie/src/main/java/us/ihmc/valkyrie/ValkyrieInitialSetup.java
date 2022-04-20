package us.ihmc.valkyrie;

import us.ihmc.avatar.initialSetup.HumanoidRobotInitialSetup;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class ValkyrieInitialSetup extends HumanoidRobotInitialSetup
{
   public ValkyrieInitialSetup(RobotDefinition robotDefinition, HumanoidJointNameMap jointMap)
   {
      super(jointMap);

      for (RobotSide robotSide : RobotSide.values)
      {
         setJoint(robotSide, LegJointName.HIP_ROLL, 0.0);
         setJoint(robotSide, LegJointName.HIP_PITCH, -0.6);
         setJoint(robotSide, LegJointName.KNEE_PITCH, 1.3);
         setJoint(robotSide, LegJointName.ANKLE_PITCH, -0.7);
         setJoint(robotSide, LegJointName.ANKLE_ROLL, 0.0);

         setJoint(robotSide, ArmJointName.SHOULDER_ROLL, robotSide.negateIfRightSide(-1.2));
         setJoint(robotSide, ArmJointName.SHOULDER_PITCH, -0.2);
         setJoint(robotSide, ArmJointName.ELBOW_PITCH, robotSide.negateIfRightSide(-1.5));
         setJoint(robotSide, ArmJointName.ELBOW_ROLL, 1.3);
      }

      setRootJointHeightSuchThatLowestSoleIsAtZero(robotDefinition);
   }
}
