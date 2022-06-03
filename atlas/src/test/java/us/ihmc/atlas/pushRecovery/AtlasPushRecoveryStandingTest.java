package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarPushRecoveryStandingTest;

public class AtlasPushRecoveryStandingTest extends AvatarPushRecoveryStandingTest
{
   @Override
   public double getAngledPushMagnitude()
   {
      return 350.0;
   }

   @Override
   protected DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testControllerFailureKicksIn()
   {
      super.testControllerFailureKicksIn();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testLongBackwardPushWhileStanding()
   {
      super.testLongBackwardPushWhileStanding();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testLongBackwardPushWhileStandingAfterControllerFailureKickedIn()
   {
      setMagnitude(100.0);
      super.testLongBackwardPushWhileStandingAfterControllerFailureKickedIn();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testLongForwardPushWhileStanding()
   {
      super.testLongForwardPushWhileStanding();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testLongForwardPushWhileStandingAfterControllerFailureKickedIn()
   {
      setMagnitude(100.0);
      super.testLongForwardPushWhileStandingAfterControllerFailureKickedIn();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushWhileStanding()
   {
      super.testPushWhileStanding();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushWhileStandingRecoveringAfterControllerFailureKickedIn()
   {
      super.testPushWhileStandingRecoveringAfterControllerFailureKickedIn();
   }


   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testRecoveryWhileInFlamingoStance()
   {
      super.testRecoveryWhileInFlamingoStance();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testRecoveryForwardWhileInFlamingoStance()
   {
      super.testRecoveryForwardWhileInFlamingoStance();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testRecoverySidewaysWhileInFlamingoStance()
   {
      super.testRecoverySidewaysWhileInFlamingoStance();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testRecoveryAngledWhileInFlamingoStance()
   {
      super.testRecoveryAngledWhileInFlamingoStance();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testRecoveryPushForwardWhileInFlamingoStanceAndAfterTouchDown()
   {
      super.testRecoveryPushForwardWhileInFlamingoStanceAndAfterTouchDown();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testFailureAfterRecoveryStep()
   {
      super.testFailureAfterRecoveryStep();
   }
}
