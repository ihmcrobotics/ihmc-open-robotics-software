package us.ihmc.valkyrie.controllerAPI;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.controllerAPI.EndToEndPelvisHeightTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieEndToEndPelvisHeightTrajectoryMessageTest extends EndToEndPelvisHeightTrajectoryMessageTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

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

   @Tag("controller-api-2")
   @Test
   @Override
   public void testSingleWaypoint() throws Exception
   {
      super.testSingleWaypoint();
   }

   @Tag("controller-api-2")
   @Test
   @Override
   public void testSingleWaypointInUserMode() throws Exception
   {
      super.testSingleWaypointInUserMode();
   }

   @Tag("controller-api-slow-2")
   @Test
   @Override
   public void testSingleWaypointThenManualChange() throws Exception
   {
      super.testSingleWaypointThenManualChange();
   }

   /*
    * FIXME This test wasn't running for a very long time and failed when re-enabled. Need to fix it
    * and re-enbable it.
    */
   @Disabled
   @Tag("controller-api-slow-2")
   @Test
   @Override
   public void testSingleWaypointWithControlFrame() throws SimulationExceededMaximumTimeException
   {
      super.testSingleWaypointWithControlFrame();
   }

   @Tag("controller-api-slow-2")
   @Test
   @Override
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }

   @Tag("controller-api-2")
   @Test
   @Override
   public void testStreaming() throws Exception
   {
      super.testStreaming();
   }
}
