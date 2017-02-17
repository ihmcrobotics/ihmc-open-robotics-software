package us.ihmc.atlas.initialSetup;

import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class PushUpDRCRobotInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
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
   
   public void getOffset(Vector3D offsetToPack)
   {
   }

   public void setOffset(Vector3D offset)
   {
   }

   @Override
   public void setInitialYaw(double yaw)
   {
   }

   @Override
   public void setInitialGroundHeight(double groundHeight)
   {
   }

   @Override
   public double getInitialYaw()
   {
      return 0;
   }

   @Override
   public double getInitialGroundHeight()
   {
      return 0;
   }
}
