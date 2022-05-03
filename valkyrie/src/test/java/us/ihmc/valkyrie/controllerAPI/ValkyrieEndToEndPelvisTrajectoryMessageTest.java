package us.ihmc.valkyrie.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.controllerAPI.EndToEndPelvisTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieEndToEndPelvisTrajectoryMessageTest extends EndToEndPelvisTrajectoryMessageTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);

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

   @Tag("controller-api-slow-2")
   @Test
   @Override
   public void testHeightModeSwitchWhileWalking() throws Exception
   {
      super.testHeightModeSwitchWhileWalking();
   }

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
   public void testSixDoFMovementsOfPelvis()
   {
      super.testSixDoFMovementsOfPelvis();
   }

   @Tag("controller-api-slow-2")
   @Test
   @Override
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
