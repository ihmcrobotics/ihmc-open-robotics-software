package us.ihmc.atlas.controllerAPI;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndFootTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasEndToEndFootTrajectoryMessageTest extends EndToEndFootTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @ContinuousIntegrationTest(estimatedDuration = 24.4)
   @Test(timeout = 120000)
   @Override
   public void testCustomControlPoint() throws SimulationExceededMaximumTimeException
   {
      super.testCustomControlPoint();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 78.1)
   @Test(timeout = 390000)
   @Override
   public void testSingleWaypoint() throws SimulationExceededMaximumTimeException
   {
      super.testSingleWaypoint();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 85.4)
   @Test(timeout = 430000)
   @Override
   public void testMultipleTrajectoryPoints() throws SimulationExceededMaximumTimeException
   {
      super.testMultipleTrajectoryPoints();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 96.2)
   @Test(timeout = 480000)
   @Override
   public void testQueuedMessages() throws SimulationExceededMaximumTimeException
   {
      super.testQueuedMessages();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 72.6)
   @Test(timeout = 360000)
   @Override
   public void testQueueStoppedWithOverrideMessage() throws SimulationExceededMaximumTimeException
   {
      super.testQueueStoppedWithOverrideMessage();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 58.5)
   @Test(timeout = 290000)
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
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
