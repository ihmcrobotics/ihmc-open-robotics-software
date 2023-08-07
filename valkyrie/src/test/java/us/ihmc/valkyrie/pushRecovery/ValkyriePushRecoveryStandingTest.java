package us.ihmc.valkyrie.pushRecovery;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarPushRecoveryStandingTest;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieSteppingParameters;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;

public class ValkyriePushRecoveryStandingTest extends AvatarPushRecoveryStandingTest
{
   @Override
   public double getAngledPushMagnitude()
   {
      return 320.0;
   }

   @Override
   protected DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS)
      {
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new ValkyrieWalkingControllerParameters(getJointMap(), getRobotPhysicalProperties(), getTarget(), getRobotVersion())
            {
               @Override
               public SteppingParameters getSteppingParameters()
               {
                  return new ValkyrieSteppingParameters(getRobotPhysicalProperties(), getTarget())
                  {
                     @Override
                     public double getMaxStepWidth()
                     {
                        return 0.8;
                     }
                  };

               };
            };
         }
      };

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
      setMagnitude(85.0);
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
      setMagnitude(90.0);
      super.testLongForwardPushWhileStandingAfterControllerFailureKickedIn();
   }

   @Disabled
   @Test
   public void testFailureAfterRecoveryStep() 
   {
      super.testFailureAfterRecoveryStep();
   }

   @Disabled
   @Test
   public void testRecoveryAngledWhileInFlamingoStance() 
   {
      super.testRecoveryAngledWhileInFlamingoStance();
   }

   @Disabled
   @Test
   public void testRecoveryPushForwardWhileInFlamingoStanceAndAfterTouchDown() 
   {
      super.testRecoveryPushForwardWhileInFlamingoStanceAndAfterTouchDown();
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
}
