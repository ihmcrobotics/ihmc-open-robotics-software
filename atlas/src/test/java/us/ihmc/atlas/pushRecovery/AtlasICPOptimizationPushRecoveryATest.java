package us.ihmc.atlas.pushRecovery;

import org.junit.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasICPOptimizationParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarICPOptimizationPushRecoveryATest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import java.lang.Exception;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class AtlasICPOptimizationPushRecoveryATest extends AvatarICPOptimizationPushRecoveryATest
{
   @Override
   protected DRCRobotModel getRobotModel()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(RobotTarget.SCS, getJointMap(), getContactPointParameters())
            {
               @Override
               public boolean useOptimizationBasedICPController()
               {
                  return true;
               }

               @Override
               public ICPOptimizationParameters getICPOptimizationParameters()
               {
                  return new AtlasICPOptimizationParameters(false)
                  {
                     @Override
                     public boolean useAngularMomentum()
                     {
                        return true;
                     }

                     @Override
                     public boolean allowStepAdjustment()
                     {
                        return true;
                     }
                  };
               }
            };
         }
      };

      return atlasRobotModel;
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
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationBackwardPushInSwing() throws Exception
   {
      percentWeight = 0.2;
      super.testPushICPOptimizationBackwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationForwardPushInSlowSwing() throws Exception
   {
      percentWeight = 0.2;
      super.testPushICPOptimizationForwardPushInSlowSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationForwardPushInSwing() throws Exception
   {
      percentWeight = 0.29;
      super.testPushICPOptimizationForwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationInwardPushInSwing() throws Exception
   {
      percentWeight = 0.17;
      super.testPushICPOptimizationInwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationLongInwardPushInSwing() throws Exception
   {
      percentWeight = 0.05;
      super.testPushICPOptimizationLongInwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationOutwardPushInTransfer() throws Exception
   {
      percentWeight = 0.12;
      super.testPushICPOptimizationOutwardPushInTransfer();
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationOutwardPushOnEachStep() throws Exception
   {
      percentWeight = 0.12;
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
