package us.ihmc.zulu.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarLongPushRecoveryWalkingTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class ZuluLongPushRecoveryWalkingTest extends AvatarLongPushRecoveryWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

   @Override
   public double getForwardPushDelta()
   {
      return 0.6;
   }

   @Override
   public double getOutwardPushDelta()
   {
      return 0.35;
   }

   @Override
   public double getBackwardPushDelta()
   {
      return 0.5;
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
