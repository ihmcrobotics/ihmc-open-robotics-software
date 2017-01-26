package us.ihmc.atlas.pushRecovery;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.DRCPushRecoveryWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.robotics.controllers.ControllerFailureException;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.SLOW, IntegrationCategory.VIDEO})
public class AtlasPushRecoveryWalkingTest extends DRCPushRecoveryWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, DRCRobotModel.RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
   
   // cropped to 1.5 - 6.3 seconds
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 50.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 300000)
   public void testForVideo() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testForVideo();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 26.7)
   @Test(timeout = 130000)
   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushLeftEarlySwing();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.2)
   @Test(timeout = 150000)
   public void testPushLeftInitialTransferState() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushLeftInitialTransferState();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 44.5)
   @Test(timeout = 220000)
   public void testPushRightInitialTransferState() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushRightInitialTransferState();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 44.5)
   @Test(timeout = 220000)
   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushRightLateSwing();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 44.5)
   @Test(timeout = 220000)
   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushRightThenLeftMidSwing();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 31.2)
   @Test(timeout = 160000)
   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushRightTransferState();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 29.3)
   @Test(timeout = 150000)
   public void testPushTowardsTheBack() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushTowardsTheBack();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 29.0)
   @Test(timeout = 150000)
   public void testPushTowardsTheFront() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushTowardsTheFront();
   }
}
