package us.ihmc.darpaRoboticsChallenge.initialSetup;

import us.ihmc.SdfLoader.SDFRobot;

public class SquaredUpDRCRobotInitialSetup implements DRCRobotInitialSetup
{

   public void initializeRobot(SDFRobot robot)
   {
      robot.getJoint("l_arm_shx").setQ(-1.4);
      robot.getJoint("r_arm_shx").setQ(1.4);
   }

}
