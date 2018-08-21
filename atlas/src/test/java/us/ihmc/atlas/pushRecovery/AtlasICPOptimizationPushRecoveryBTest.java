package us.ihmc.atlas.pushRecovery;

import org.junit.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasICPOptimizationParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarICPOptimizationPushRecoveryBTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import java.lang.Exception;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class AtlasICPOptimizationPushRecoveryBTest extends AvatarICPOptimizationPushRecoveryBTest
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
   protected double getSizeScale()
   {
      return 1.0;
   }

   @Override
   public double getNominalHeight()
   {
      return 0.9;
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationDiagonalOutwardPushInSwing() throws Exception
   {
      percentWeight = 0.2;
      super.testPushICPOptimizationDiagonalOutwardPushInSwing();
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationDiagonalYawingOutwardPushInSwing() throws Exception
   {
      percentWeight = 0.13;
      super.testPushICPOptimizationDiagonalYawingOutwardPushInSwing();
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationLongBackwardPushInSwing() throws Exception
   {
      percentWeight = 0.15;
      super.testPushICPOptimizationLongBackwardPushInSwing();
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationLongForwardPushInSwing() throws Exception
   {
      percentWeight = 0.07;
      super.testPushICPOptimizationLongForwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationNoPush() throws Exception
   {
      super.testPushICPOptimizationNoPush();
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationOutwardPushInSlowSwing() throws Exception
   {
      percentWeight = 0.11;
      super.testPushICPOptimizationOutwardPushInSlowSwing();
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationOutwardPushInSwing() throws Exception
   {
      percentWeight = 0.25;
      super.testPushICPOptimizationOutwardPushInSwing();
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationRandomPushInSwing() throws Exception
   {
      percentWeight = 0.18;
      super.testPushICPOptimizationRandomPushInSwing();
   }

   public static void main(String[] args)
   {
      AtlasICPOptimizationPushRecoveryBTest test = new AtlasICPOptimizationPushRecoveryBTest();
      try
      {
         test.testPushICPOptimizationNoPush();
      }
      catch (Exception e)
      {

      }
   }
}
