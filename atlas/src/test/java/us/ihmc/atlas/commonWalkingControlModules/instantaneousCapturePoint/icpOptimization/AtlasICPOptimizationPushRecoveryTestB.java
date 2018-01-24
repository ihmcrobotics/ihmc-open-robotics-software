package us.ihmc.atlas.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import org.junit.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasICPOptimizationParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.AvatarICPOptimizationPushRecoveryTestB;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class AtlasICPOptimizationPushRecoveryTestB extends AvatarICPOptimizationPushRecoveryTestB
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
                     public boolean useStepAdjustment()
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

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationDiagonalOutwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationDiagonalOutwardPushInSwing(0.2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationDiagonalYawingOutwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationDiagonalYawingOutwardPushInSwing(0.13);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationLongBackwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationLongBackwardPushInSwing(0.15);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationLongForwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationLongForwardPushInSwing(0.07);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationNoPush() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationNoPush();
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationOutwardPushInSlowSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationOutwardPushInSlowSwing(0.11);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationOutwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationOutwardPushInSwing(0.25);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 150000)
   public void testPushICPOptimizationRandomPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationRandomPushInSwing(0.18);
   }

   public static void main(String[] args)
   {
      AtlasICPOptimizationPushRecoveryTestB test = new AtlasICPOptimizationPushRecoveryTestB();
      try
      {
         test.testPushICPOptimizationNoPush();
      }
      catch (SimulationExceededMaximumTimeException e)
      {

      }
   }
}
