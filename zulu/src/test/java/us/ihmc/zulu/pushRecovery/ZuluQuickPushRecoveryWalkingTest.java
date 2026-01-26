package us.ihmc.zulu.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarQuickPushRecoveryWalkingTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class ZuluQuickPushRecoveryWalkingTest extends AvatarQuickPushRecoveryWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ZULU);
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testInwardPushLeftAtDifferentTimes()
   {
      setPushChangeInVelocity(0.2);
      super.testInwardPushLeftAtDifferentTimes();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testOutwardPushInitialTransferToLeftStateAndLeftMidSwing()
   {
      setPushHeightOffsetFromChest(-0.1);
      setPushChangeInVelocity(0.4);
      super.testOutwardPushInitialTransferToLeftStateAndLeftMidSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testOutwardPushLeftSwingAtDifferentTimes()
   {
      setPushChangeInVelocity(0.5);
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
