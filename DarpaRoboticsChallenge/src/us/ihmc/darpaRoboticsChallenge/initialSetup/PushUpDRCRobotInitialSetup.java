package us.ihmc.darpaRoboticsChallenge.initialSetup;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;

public class PushUpDRCRobotInitialSetup implements RobotInitialSetup<SDFRobot>
{
   public void initializeRobot(SDFRobot robot)
   {
      double elbowBend = Math.PI / 2.0;
      double handRotation = Math.PI / 2.0;
      double bodyPitch = Math.PI / 2.0 - 0.4;
      
      robot.getPinJoint("l_arm_ely").setQ(handRotation);
      robot.getPinJoint("l_arm_elx").setQ(elbowBend);

      robot.getPinJoint("r_arm_ely").setQ(handRotation);
      robot.getPinJoint("r_arm_elx").setQ(-elbowBend);

      robot.getPinJoint("l_arm_uwy").setQ(-handRotation);
      robot.getPinJoint("r_arm_uwy").setQ(-handRotation);
      
      robot.getPinJoint("l_arm_mwx").setQ(bodyPitch);
      robot.getPinJoint("r_arm_mwx").setQ(-bodyPitch);
      
      robot.getPinJoint("l_leg_uay").setQ(-0.3);
      robot.getPinJoint("r_leg_uay").setQ(-0.3);

      robot.setOrientation(0.0, bodyPitch , 0.0);
   }
}
