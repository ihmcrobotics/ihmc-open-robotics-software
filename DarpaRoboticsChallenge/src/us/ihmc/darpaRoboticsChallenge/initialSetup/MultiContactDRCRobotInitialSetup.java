package us.ihmc.darpaRoboticsChallenge.initialSetup;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.projectM.R2Sim02.initialSetup.RobotInitialSetup;

public class MultiContactDRCRobotInitialSetup implements RobotInitialSetup<SDFRobot>
{
   public void initializeRobot(SDFRobot robot)
   {
      // left arm
      robot.getPinJoint("l_arm_usy").setQ(-1.2);
      robot.getPinJoint("l_arm_shx").setQ(-0.9);
      robot.getPinJoint("l_arm_ely").setQ(1.0);
      robot.getPinJoint("l_arm_elx").setQ(1.3);
      robot.getPinJoint("l_arm_mwx").setQ(0.5);

      // left leg
      robot.getPinJoint("l_leg_uhz").setQ(0.4);
      robot.getPinJoint("l_leg_mhx").setQ(0.5);
      robot.getPinJoint("l_leg_lhy").setQ(0.3);
      robot.getPinJoint("l_leg_kny").setQ(0.8);

      // right leg
      robot.getPinJoint("r_leg_mhx").setQ(-1.0);
      robot.getPinJoint("r_leg_lhy").setQ(0.3);
      robot.getPinJoint("r_leg_kny").setQ(0.4);
   }

}
