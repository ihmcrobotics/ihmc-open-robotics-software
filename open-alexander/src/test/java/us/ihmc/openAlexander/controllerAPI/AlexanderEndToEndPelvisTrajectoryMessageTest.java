package us.ihmc.openAlexander.controllerAPI;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndPelvisTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;

public class AlexanderEndToEndPelvisTrajectoryMessageTest extends EndToEndPelvisTrajectoryMessageTest
{

   private final DRCRobotModel robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(SimpleRobotNameKeys.ALEXANDER);
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
   public void testSixDoFMovementsOfPelvis()
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
