package us.ihmc.valkyrie.controllerAPI;

import org.junit.Test;

import us.ihmc.avatar.controllerAPI.EndToEndFootTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieEndToEndFootTrajectoryMessageTest extends EndToEndFootTrajectoryMessageTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @ContinuousIntegrationTest(estimatedDuration = 40.0)
   @Test(timeout = 400000)
   @Override
   public void testCustomControlPoint() throws SimulationExceededMaximumTimeException
   {
      super.testCustomControlPoint();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 400000)
   @Override
   public void testSingleWaypoint() throws SimulationExceededMaximumTimeException
   {
      super.testSingleWaypoint();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 80.5)
   @Test(timeout = 400000)
   @Override
   public void testMultipleTrajectoryPoints() throws SimulationExceededMaximumTimeException
   {
      super.testMultipleTrajectoryPoints();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 90.0)
   @Test(timeout = 400000)
   @Override
   public void testQueuedMessages() throws SimulationExceededMaximumTimeException
   {
      super.testQueuedMessages();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 70.0)
   @Test(timeout = 400000)
   @Override
   public void testQueueStoppedWithOverrideMessage() throws SimulationExceededMaximumTimeException
   {
      super.testQueueStoppedWithOverrideMessage();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 400000)
   @Override
   public void testQueueWithWrongPreviousId() throws SimulationExceededMaximumTimeException
   {
      super.testQueueWithWrongPreviousId();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }
}
