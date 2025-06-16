package us.ihmc.alexander.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.alexander.OpenAlexanderVersion;
import us.ihmc.alexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarPushRecoveryWithCrossOverWalkingTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AlexanderPushRecoveryWithCrossOverWalkingTest extends AvatarPushRecoveryWithCrossOverWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new OpenAlexanderRobotModel(OpenAlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

   @Override
   public double getForwardPushDelta()
   {
      return 1.2;
   }

   @Override
   public double getOutwardPushDelta()
   {
      return 0.3;
   }

   @Override
   public double getBackwardPushDelta()
   {
      return 1.0;
   }

   @Override
   public double getInwardPushDelta()
   {
      return 0.45;
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testInwardPushInSwing()
   {
      super.testInwardPushInSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testOutwardPushInSwing()
   {
      super.testOutwardPushInSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testForwardPushInSwing()
   {
      super.testForwardPushInSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testBackwardPushInSwing()
   {
      super.testBackwardPushInSwing();
   }
}
