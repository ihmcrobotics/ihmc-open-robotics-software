package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarLongPushRecoveryWalkingTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AtlasLongPushRecoveryWalkingTest extends AvatarLongPushRecoveryWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   public double getForwardPushDelta()
   {
      return 0.6;
   }

   @Override
   public double getOutwardPushDelta()
   {
      return 0.5;
   }

   @Override
   public double getBackwardPushDelta()
   {
      return 0.6;
   }

   @Override
   public double getInwardPushDelta()
   {
      return 0.3;
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testInwardPushInSwing()
   {
      super.testInwardPushInSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testOutwardPushInSwing()
   {
      super.testOutwardPushInSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testForwardPushInSwing()
   {
      super.testForwardPushInSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testBackwardPushInSwing()
   {
      super.testBackwardPushInSwing();
   }
}
