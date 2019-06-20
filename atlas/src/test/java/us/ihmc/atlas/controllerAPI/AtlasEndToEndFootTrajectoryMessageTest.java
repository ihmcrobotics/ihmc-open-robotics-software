package us.ihmc.atlas.controllerAPI;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndFootTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasEndToEndFootTrajectoryMessageTest extends EndToEndFootTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Test
   @Override
   public void testCustomControlPoint() throws SimulationExceededMaximumTimeException
   {
      super.testCustomControlPoint();
   }
   
   @Test
   @Override
   public void testSingleWaypoint() throws SimulationExceededMaximumTimeException
   {
      super.testSingleWaypoint();
   }
   
   @Test
   @Override
   public void testMultipleTrajectoryPoints() throws SimulationExceededMaximumTimeException
   {
      super.testMultipleTrajectoryPoints();
   }
   
   @Test
   @Override
   public void testQueuedMessages() throws SimulationExceededMaximumTimeException
   {
      super.testQueuedMessages();
   }
   
   @Test
   @Override
   public void testQueueStoppedWithOverrideMessage() throws SimulationExceededMaximumTimeException
   {
      super.testQueueStoppedWithOverrideMessage();
   }
   
   @Test
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
