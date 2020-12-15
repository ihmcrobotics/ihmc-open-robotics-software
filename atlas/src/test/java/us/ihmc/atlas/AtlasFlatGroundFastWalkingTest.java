package us.ihmc.atlas;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.AvatarFlatGroundFastWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;

public class AtlasFlatGroundFastWalkingTest extends AvatarFlatGroundFastWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS)
      {
         @Override
         public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
         {
            return super.createHumanoidFloatingRootJointRobot(createCollisionMeshes, false);
         }

         @Override
         public double getControllerDT()
         {
            return 0.002;
         }

         @Override
         public double getSimulateDT()
         {
            return 0.0005;
         }
      };
   }

   @Override
   public double getFastSwingTime()
   {
      return 0.4;
   }

   @Override
   public double getFastTransferTime()
   {
      return 0.05;
   }

   @Test
   @Override
   public void testForwardWalking() throws Exception
   {
      super.testForwardWalking();
   }
}
