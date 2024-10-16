package us.ihmc.atlas.initialSetup;

import us.ihmc.avatar.initialSetup.HumanoidRobotInitialSetup;
import us.ihmc.commons.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.commons.robotics.partNames.LegJointName;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class AtlasSimInitialSetup extends HumanoidRobotInitialSetup
{
   public AtlasSimInitialSetup(RobotDefinition robotDefinition, HumanoidJointNameMap jointMap)
   {
      super(jointMap);

      for (RobotSide robotSide : RobotSide.values)
      {
         setJoint(robotSide, LegJointName.HIP_YAW, robotSide.negateIfRightSide(0.0));
         setJoint(robotSide, LegJointName.HIP_ROLL, robotSide.negateIfRightSide(0.062));
         setJoint(robotSide, LegJointName.HIP_PITCH, -0.233);
         setJoint(robotSide, LegJointName.KNEE_PITCH, 0.518);
         setJoint(robotSide, LegJointName.ANKLE_PITCH, -0.276);
         setJoint(robotSide, LegJointName.ANKLE_ROLL, robotSide.negateIfRightSide(-0.062));

         setJoint(robotSide, ArmJointName.SHOULDER_YAW, robotSide.negateIfRightSide(0.785398));
         setJoint(robotSide, ArmJointName.SHOULDER_ROLL, robotSide.negateIfRightSide(-0.52379));
         setJoint(robotSide, ArmJointName.ELBOW_PITCH, 2.33708);
         setJoint(robotSide, ArmJointName.ELBOW_ROLL, robotSide.negateIfRightSide(2.35619));
         setJoint(robotSide, ArmJointName.FIRST_WRIST_PITCH, -0.337807);
         setJoint(robotSide, ArmJointName.WRIST_ROLL, robotSide.negateIfRightSide(0.20773));
         setJoint(robotSide, ArmJointName.SECOND_WRIST_PITCH, -0.026599);
      }

      setRootJointHeightSuchThatLowestSoleIsAtZero(robotDefinition);
   }
}
