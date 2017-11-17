package us.ihmc.atlas.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasContinuousCMPPlannerParameters;
import us.ihmc.atlas.parameters.AtlasICPOptimizationParameters;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.AvatarICPOptimizationPushRecoveryTest;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.SLOW})
public class AtlasICPOptimizationPushRecoveryTest extends AvatarICPOptimizationPushRecoveryTest
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
                  return new AtlasICPOptimizationParameters(false);
               }
            };
         }

         @Override
         public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
         {
            return new AtlasContinuousCMPPlannerParameters(new AtlasPhysicalProperties());
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
   @Test(timeout = 90000)
   public void testPushICPOptimizationBackwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationBackwardPushInSwing(0.2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationDiagonalOutwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationDiagonalOutwardPushInSwing(0.2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationDiagonalYawingOutwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationDiagonalYawingOutwardPushInSwing(0.13);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationForwardPushInSlowSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationForwardPushInSlowSwing(0.2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationForwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationForwardPushInSwing(0.29);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationInwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationInwardPushInSwing(0.18);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationLongBackwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationLongBackwardPushInSwing(0.15);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationLongForwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationLongForwardPushInSwing(0.07);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationLongInwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationLongInwardPushInSwing(0.05);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationNoPush() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationNoPush();
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationOutwardPushInSlowSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationOutwardPushInSlowSwing(0.11);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationOutwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationOutwardPushInSwing(0.25);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationOutwardPushInTransfer() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationOutwardPushInTransfer(0.12);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationOutwardPushOnEachStep() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationOutwardPushOnEachStep(0.12);
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 90000)
   public void testPushICPOptimizationRandomPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationRandomPushInSwing(0.18);
   }

   public static void main(String[] args)
   {
      AtlasICPOptimizationPushRecoveryTest test = new AtlasICPOptimizationPushRecoveryTest();
      try
      {
         test.testPushICPOptimizationOutwardPushInSwing();
      }
      catch(SimulationExceededMaximumTimeException e)
      {

      }
   }
}
