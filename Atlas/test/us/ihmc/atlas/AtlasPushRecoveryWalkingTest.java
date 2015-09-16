package us.ihmc.atlas;

import org.junit.Test;
import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;

@DeployableTestClass(targets = {TestPlanTarget.Slow, TestPlanTarget.VideoB})
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

   @Override
   @DeployableTestMethod(estimatedDuration = 35.7)
   @Test(timeout = 180000)
   public void testForVideo() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testForVideo();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 37.2)
   @Test(timeout = 190000)
   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushLeftEarlySwing();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 56.4)
   @Test(timeout = 280000)
   public void testPushLeftInitialTransferState() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushLeftInitialTransferState();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 60.0)
   @Test(timeout = 300000)
   public void testPushRightInitialTransferState() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushRightInitialTransferState();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 36.5)
   @Test(timeout = 180000)
   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushRightLateSwing();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 54.3)
   @Test(timeout = 270000)
   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushRightThenLeftMidSwing();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 40.4)
   @Test(timeout = 200000)
   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushRightTransferState();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 35.8)
   @Test(timeout = 180000)
   public void testPushTowardsTheBack() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushTowardsTheBack();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 35.9)
   @Test(timeout = 180000)
   public void testPushTowardsTheFront() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushTowardsTheFront();
   }
}
