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
      
      robot.getOneDoFJoint("l_arm_ely").setQ(handRotation);
      robot.getOneDoFJoint("l_arm_elx").setQ(elbowBend);

      robot.getOneDoFJoint("r_arm_ely").setQ(handRotation);
      robot.getOneDoFJoint("r_arm_elx").setQ(-elbowBend);

      robot.getOneDoFJoint("l_arm_uwy").setQ(-handRotation);
      robot.getOneDoFJoint("r_arm_uwy").setQ(-handRotation);
      
      robot.getOneDoFJoint("l_arm_mwx").setQ(bodyPitch);
      robot.getOneDoFJoint("r_arm_mwx").setQ(-bodyPitch);
      
      robot.getOneDoFJoint("l_leg_uay").setQ(-0.3);
      robot.getOneDoFJoint("r_leg_uay").setQ(-0.3);

      robot.setOrientation(0.0, bodyPitch , 0.0);
   }
}
