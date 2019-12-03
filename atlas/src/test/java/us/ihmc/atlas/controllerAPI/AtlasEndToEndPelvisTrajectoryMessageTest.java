package us.ihmc.atlas.controllerAPI;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndPelvisTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;

public class AtlasEndToEndPelvisTrajectoryMessageTest extends EndToEndPelvisTrajectoryMessageTest
{

   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

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

   @Tag("controller-api-slow-2")
   @Test
   @Override
   public void testHeightModeSwitchWhileWalking() throws Exception
   {
      super.testHeightModeSwitchWhileWalking();
   }

   /*
    * FIXME This test wasn't running for a very long time and failed when re-enabled. Need to fix it
    * and re-enbable it.
    */
   @Disabled
   @Tag("controller-api-slow-2")
   @Test
   @Override
   public void testHeightUsingMultipleWaypoints() throws Exception
   {
      super.testHeightUsingMultipleWaypoints();
   }

   @Tag("controller-api-slow-2")
   @Test
   @Override
   public void testHeightUsingMultipleWaypointsWhileWalking() throws Exception
   {
      super.testHeightUsingMultipleWaypointsWhileWalking();
   }

   @Tag("controller-api-2")
   @Test
   @Override
   public void testMultipleWaypoints() throws Exception
   {
      super.testMultipleWaypoints();
   }

   @Tag("controller-api-2")
   @Test
   @Override
   public void testSingleWaypoint() throws Exception
   {
      super.testSingleWaypoint();
   }

   @Tag("controller-api-slow-2")
   @Test
   @Override
   public void testSingleWaypointAndAbort() throws Exception
   {
      super.testSingleWaypointAndAbort();
   }

   @Tag("controller-api-slow-2")
   @Test
   @Override
   public void testSingleWaypointAndWalk() throws Exception
   {
      super.testSingleWaypointAndWalk();
   }

   @Tag("controller-api-slow-2")
   @Test
   @Override
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }

   @Tag("controller-api-slow-2")
   @Test
   @Override
   public void testSixDoFMovementsOfPelvis() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      super.testSixDoFMovementsOfPelvis();
   }

   @Test
   @Override
   @Disabled
   public void testSingleWaypointThenManualChange() throws Exception
   {
      super.testSingleWaypointThenManualChange();
   }

   @Tag("controller-api-2")
   @Test
   @Override
   public void testStreaming() throws Exception
   {
      super.testStreaming();
   }
}
