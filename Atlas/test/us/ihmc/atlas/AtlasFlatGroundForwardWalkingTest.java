package us.ihmc.atlas;

import us.ihmc.avatar.AvatarFlatGroundForwardWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.robotics.partNames.ArmJointName;

public class AtlasFlatGroundForwardWalkingTest extends AvatarFlatGroundForwardWalkingTest
{
   private final AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
   private final AtlasRobotModel robotModel = new AtlasRobotModel(version, RobotTarget.SCS, false);
   private final AtlasJointMap jointMap = new AtlasJointMap(version, robotModel.getPhysicalProperties());

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Override
   public int getNumberOfSteps()
   {
      return 6;
   }

   @Override
   public double getStepWidth()
   {
      return 0.35;
   }

   @Override
   public double getStepLength()
   {
      return 0.5;
   }
   
   @Override
   protected boolean keepSCSUp()
   {
      return true;
   }
}
