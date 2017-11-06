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
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationBackwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationBackwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationDiagonalOutwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationDiagonalOutwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationDiagonalYawingOutwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationDiagonalYawingOutwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationForwardPushInSlowSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationForwardPushInSlowSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationForwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationForwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationInwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationInwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationLongBackwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationLongBackwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationLongForwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationLongForwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationLongInwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationLongInwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationNoPush() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationNoPush();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationOutwardPushInSlowSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationOutwardPushInSlowSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationOutwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationOutwardPushInSwing();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationOutwardPushInTransfer() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationOutwardPushInTransfer();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationOutwardPushOnEachStep() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationOutwardPushOnEachStep();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 60000)
   public void testPushICPOptimizationRandomPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testPushICPOptimizationRandomPushInSwing();
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
