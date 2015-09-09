package us.ihmc.atlas;

import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.DRCPushRecoveryWalkingTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;

@DeployableTestClass(planType = {BambooPlanType.Slow, BambooPlanType.VideoB})
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
	@DeployableTestMethod(duration = 32.7)
   @Test(timeout = 160000)
   public void testForVideo() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testForVideo();
   }
   
   @Override
	@DeployableTestMethod(duration = 34.1)
   @Test(timeout = 170000)
   public void testPushLeftEarlySwing() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushLeftEarlySwing();
   }
   
   @Override
	@DeployableTestMethod(duration = 71.6)
   @Test(timeout = 360000)
   public void testPushLeftInitialTransferState() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushLeftInitialTransferState();
   }
   
   @Override
	@DeployableTestMethod(duration = 52.0)
   @Test(timeout = 260000)
   public void testPushRightInitialTransferState() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushRightInitialTransferState();
   }
   
   @Override
	@DeployableTestMethod(duration = 35.5)
   @Test(timeout = 180000)
   public void testPushRightLateSwing() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushRightLateSwing();
   }
   
   @Override
	@DeployableTestMethod(duration = 58.0)
   @Test(timeout = 290000)
   public void testPushRightThenLeftMidSwing() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushRightThenLeftMidSwing();
   }
   
   @Override
	@DeployableTestMethod(duration = 35.5)
   @Test(timeout = 180000)
   public void testPushRightTransferState() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushRightTransferState();
   }
   
   @Override
	@DeployableTestMethod(duration = 31.2)
   @Test(timeout = 160000)
   public void testPushTowardsTheBack() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushTowardsTheBack();
   }
   
   @Override
	@DeployableTestMethod(duration = 30.9)
   @Test(timeout = 150000)
   public void testPushTowardsTheFront() throws SimulationExceededMaximumTimeException, InterruptedException, ControllerFailureException
   {
      super.testPushTowardsTheFront();
   }
}
