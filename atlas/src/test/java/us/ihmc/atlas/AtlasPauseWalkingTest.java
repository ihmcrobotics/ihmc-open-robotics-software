package us.ihmc.atlas;

import org.junit.Test;

import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.AvatarPauseWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasPauseWalkingTest extends AvatarPauseWalkingTest
{
   private final RobotTarget target = RobotTarget.SCS;

   private final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, target, false)
   {
      @Override
      public WalkingControllerParameters getWalkingControllerParameters()
      {
         return new AtlasWalkingControllerParameters(target, getJointMap(), getContactPointParameters())
         {
            @Override
            public double getDefaultFinalTransferTime()
            {
               return getFinalTransferDuration();
            }
         };
      }
   };

   @Override
   public double getMaxICPPlanError()
   {
      return 0.02;
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Override
   public double getSwingTime()
   {
      return 1.2;
   }

   @Override
   public double getTransferTime()
   {
      return 0.8;
   }

   @Override
   public double getFinalTransferDuration()
   {
      return 1.5;
   }

   @Override
   public double getStepLength()
   {
      return 0.3;
   }

   @Override
   public double getStepWidth()
   {
      return 0.25;
   }

   @Override
   public double getTimeForPausing()
   {
      return 2.55;
   }

   @Override
   public double getTimeForResuming()
   {
      return 2.0;
   }

   @Override
   public int getNumberOfFootsteps()
   {
      return 5;
   }

   @ContinuousIntegrationTest(estimatedDuration = 76.4, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 380000)
   @Override
   public void testPauseWalking() throws SimulationExceededMaximumTimeException
   {
      super.testPauseWalking();
   }

   @ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 380000)
   @Override
   public void testTwoIndependentSteps()throws SimulationExceededMaximumTimeException
   {
      super.testTwoIndependentSteps();
   }

   @ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 380000)
   @Override
   public void testStartSecondStepWhileTransitioningToStand()throws SimulationExceededMaximumTimeException
   {
      super.testStartSecondStepWhileTransitioningToStand();
   }

   @ContinuousIntegrationTest(estimatedDuration = 77.3)
   @Test(timeout = 390000)
   @Override
   public void testPauseWalkingForward() throws SimulationExceededMaximumTimeException
   {
      super.testPauseWalkingForward();
   }

   @ContinuousIntegrationTest(estimatedDuration = 78.9)
   @Test(timeout = 390000)
   @Override
   public void testPauseWalkingInitialTransfer() throws SimulationExceededMaximumTimeException
   {
      super.testPauseWalkingInitialTransfer();
   }

   @ContinuousIntegrationTest(estimatedDuration = 73.9, categoriesOverride = IntegrationCategory.SLOW)
   @Test(timeout = 370000)
   @Override
   public void testPauseWalkingForwardInitialTransfer() throws SimulationExceededMaximumTimeException
   {
      super.testPauseWalkingForwardInitialTransfer();
   }

   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 100000)
   public void testPauseWalkingInitialTransferOneStep() throws SimulationExceededMaximumTimeException
   {
      super.testPauseWalkingInitialTransferOneStep();
   }
}
