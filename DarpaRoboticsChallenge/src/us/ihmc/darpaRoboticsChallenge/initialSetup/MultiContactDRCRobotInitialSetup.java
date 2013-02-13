package us.ihmc.darpaRoboticsChallenge.initialSetup;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;

public class MultiContactDRCRobotInitialSetup implements RobotInitialSetup<SDFRobot>
{
   public void initializeRobot(SDFRobot robot)
   {
      // right arm
      robot.getOneDoFJoint("r_arm_usy").setQ(0.3);
      robot.getOneDoFJoint("r_arm_shx").setQ(1.0);
      robot.getOneDoFJoint("r_arm_ely").setQ(1.0);
      robot.getOneDoFJoint("r_arm_elx").setQ(-1.6);
      
      // left arm
      robot.getOneDoFJoint("l_arm_usy").setQ(-1.2);
      robot.getOneDoFJoint("l_arm_shx").setQ(-0.9);
      robot.getOneDoFJoint("l_arm_ely").setQ(1.0);
      robot.getOneDoFJoint("l_arm_elx").setQ(1.3);
      robot.getOneDoFJoint("l_arm_mwx").setQ(0.5);
     
      // left leg
      robot.getOneDoFJoint("l_leg_uhz").setQ(0.4);
      robot.getOneDoFJoint("l_leg_mhx").setQ(0.4);
      robot.getOneDoFJoint("l_leg_lhy").setQ(0.3);
      robot.getOneDoFJoint("l_leg_kny").setQ(0.8);

      // right leg
      robot.getOneDoFJoint("r_leg_mhx").setQ(-0.4);
      robot.getOneDoFJoint("r_leg_lhy").setQ(0.3);
      robot.getOneDoFJoint("r_leg_kny").setQ(0.4);
   }

}
