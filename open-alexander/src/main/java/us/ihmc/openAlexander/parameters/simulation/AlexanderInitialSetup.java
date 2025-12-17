package us.ihmc.openAlexander.parameters.simulation;

import us.ihmc.avatar.initialSetup.HumanoidRobotInitialSetup;
import us.ihmc.openAlexander.ZuluVersionInterface;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.robot.RobotDefinition;

public class AlexanderInitialSetup extends HumanoidRobotInitialSetup
{
   public AlexanderInitialSetup(ZuluVersionInterface version, RobotDefinition robotDefinition, HumanoidJointNameMap jointMap)
   {
      super(jointMap);

      for (RobotSide robotSide : RobotSide.values)
      {
         setJoint(robotSide, LegJointName.HIP_ROLL, 0.0);
         setJoint(robotSide, LegJointName.HIP_YAW, 0.0);
         setJoint(robotSide, LegJointName.HIP_PITCH, -0.175);
         setJoint(robotSide, LegJointName.KNEE_PITCH, 0.433);
         setJoint(robotSide, LegJointName.ANKLE_PITCH, -0.211);
         setJoint(robotSide, LegJointName.ANKLE_ROLL, 0.0);

         setJoint(robotSide, ArmJointName.SHOULDER_YAW, robotSide.negateIfLeftSide(0.202));
         setJoint(robotSide, ArmJointName.SHOULDER_ROLL, robotSide.negateIfRightSide(0.146));
         setJoint(robotSide, ArmJointName.SHOULDER_PITCH, 0.018);
         setJoint(robotSide, ArmJointName.ELBOW_PITCH, -0.312);
         setJoint(robotSide, ArmJointName.ELBOW_YAW, 0.0);
         setJoint(robotSide, ArmJointName.WRIST_ROLL, 0.0);
         setJoint(robotSide, ArmJointName.WRIST_YAW, 0.0);
      }

      setJoint(SpineJointName.SPINE_YAW, 0.0);
      setJoint(SpineJointName.SPINE_PITCH, 0.0);
      setJoint(NeckJointName.DISTAL_NECK_YAW, 0.0);
      setJoint(NeckJointName.DISTAL_NECK_PITCH, 0.0);

      setRootJointHeightSuchThatLowestSoleIsAtZero(robotDefinition);
   }
}
