package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasSteppingParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarICPOptimizationPushRecoveryATest;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;

public class AtlasICPOptimizationPushRecoveryATest extends AvatarICPOptimizationPushRecoveryATest
{
   @Override
   protected DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(getTarget(), getJointMap(), getContactPointParameters())
            {
               @Override
               public SteppingParameters getSteppingParameters()
               {
                  return new AtlasSteppingParameters(getJointMap())
                  {
                     @Override
                     public double getMaxStepLength()
                     {
                        return 0.8;
                     }

                     @Override
                     public double getMaxStepWidth()
                     {
                        return 0.7;
                     }
                  };
               }
            };
         }
      };
   }

   @Override
   public double getNominalHeight()
   {
      return 0.9;
   }

   @Override
   protected double getSizeScale()
   {
      return 1.0;
   }

   @Override
   public double getSlowSwingDuration()
   {
      return 1.2;
   }

   @Override
   public double getSlowTransferDuration()
   {
      return 0.8;
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushICPOptimizationBackwardPushInSwing() throws Exception
   {
      percentWeight = 0.5;
      super.testPushICPOptimizationBackwardPushInSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushICPOptimizationForwardPushInSlowSwing() throws Exception
   {
      percentWeight = 0.6;
      super.testPushICPOptimizationForwardPushInSlowSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushICPOptimizationForwardPushInSwing() throws Exception
   {
      percentWeight = 0.7;
      super.testPushICPOptimizationForwardPushInSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushICPOptimizationInwardPushInSwing() throws Exception
   {
      percentWeight = 0.15;
      super.testPushICPOptimizationInwardPushInSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushICPOptimizationLongInwardPushInSwing() throws Exception
   {
      percentWeight = 0.05;
      super.testPushICPOptimizationLongInwardPushInSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushICPOptimizationOutwardPushInTransfer() throws Exception
   {
      percentWeight = 0.25;
      super.testPushICPOptimizationOutwardPushInTransfer();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushICPOptimizationOutwardPushOnEachStep() throws Exception
   {
      percentWeight = 0.25;
      super.testPushICPOptimizationOutwardPushOnEachStep();
   }

   public static void main(String[] args)
   {
      AtlasICPOptimizationPushRecoveryATest test = new AtlasICPOptimizationPushRecoveryATest();
      try
      {
         test.testPushICPOptimizationOutwardPushOnEachStep();
      }
      catch (Exception e)
      {

      }
   }
}
