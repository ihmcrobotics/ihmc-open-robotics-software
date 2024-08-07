package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarQuickPushRecoveryWalkingTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AtlasQuickPushRecoveryWalkingTest extends AvatarQuickPushRecoveryWalkingTest
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

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testInwardPushLeftAtDifferentTimes()
   {
      setPushChangeInVelocity(0.3);
      super.testInwardPushLeftAtDifferentTimes();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testOutwardPushInitialTransferToLeftStateAndLeftMidSwing()
   {
      setPushChangeInVelocity(0.5);
      super.testOutwardPushInitialTransferToLeftStateAndLeftMidSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testOutwardPushLeftSwingAtDifferentTimes()
   {
      setPushChangeInVelocity(0.6);
      super.testOutwardPushLeftSwingAtDifferentTimes();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushOutwardInRightThenLeftMidSwing()
   {
      setPushChangeInVelocity(0.5);
      super.testPushOutwardInRightThenLeftMidSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testOutwardPushTransferToLeftState()
   {
      setPushChangeInVelocity(0.4);
      super.testOutwardPushTransferToLeftState();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testBackwardPushInLeftSwingAtDifferentTimes()
   {
      setPushChangeInVelocity(0.8);
      super.testBackwardPushInLeftSwingAtDifferentTimes();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testForwardPushInLeftSwingAtDifferentTimes()
   {
      setPushChangeInVelocity(0.8);
      super.testForwardPushInLeftSwingAtDifferentTimes();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testForwardAndOutwardPushInLeftSwing()
   {
      setPushChangeInVelocity(0.8);
      super.testForwardAndOutwardPushInLeftSwing();
   }

}
