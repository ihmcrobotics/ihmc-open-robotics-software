package us.ihmc.atlas.initialSetup;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;

public class PushUpDRCRobotInitialSetup implements DRCRobotInitialSetup<SDFRobot>
{
   public void initializeRobot(SDFRobot robot, DRCRobotJointMap jointMap)
   {
      double elbowBend = Math.PI / 2.0;
      double handRotation = Math.PI / 2.0;
      double bodyPitch = Math.PI / 2.0 - 0.4;
      
      robot.getOneDegreeOfFreedomJoint("l_arm_ely").setQ(handRotation);
      robot.getOneDegreeOfFreedomJoint("l_arm_elx").setQ(elbowBend);

      robot.getOneDegreeOfFreedomJoint("r_arm_ely").setQ(handRotation);
      robot.getOneDegreeOfFreedomJoint("r_arm_elx").setQ(-elbowBend);

      robot.getOneDegreeOfFreedomJoint("l_arm_wry").setQ(-handRotation);
      robot.getOneDegreeOfFreedomJoint("r_arm_wry").setQ(-handRotation);
      
      robot.getOneDegreeOfFreedomJoint("l_arm_wrx").setQ(bodyPitch);
      robot.getOneDegreeOfFreedomJoint("r_arm_wrx").setQ(-bodyPitch);
      
      robot.getOneDegreeOfFreedomJoint("l_leg_aky").setQ(-0.3);
      robot.getOneDegreeOfFreedomJoint("r_leg_aky").setQ(-0.3);

      robot.setOrientation(0.0, bodyPitch , 0.0);
   }
}
