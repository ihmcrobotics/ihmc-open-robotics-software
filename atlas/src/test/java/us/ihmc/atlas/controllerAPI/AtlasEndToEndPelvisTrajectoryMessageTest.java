package us.ihmc.atlas.controllerAPI;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndPelvisTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

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

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 61.1)
   @Test(timeout = 310000)
   public void testHeightModeSwitchWhileWalking() throws Exception
   {
      super.testHeightModeSwitchWhileWalking();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 61.1)
   @Test(timeout = 310000)
   public void testHeightUsingMultipleWaypoints() throws Exception
   {
      super.testHeightUsingMultipleWaypoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 72.0)
   @Test(timeout = 360000)
   public void testHeightUsingMultipleWaypointsWhileWalking() throws Exception
   {
      super.testHeightUsingMultipleWaypointsWhileWalking();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 22.1)
   @Test(timeout = 110000)
   public void testMultipleWaypoints() throws Exception
   {
      super.testMultipleWaypoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.1)
   @Test(timeout = 150000)
   public void testSingleWaypoint() throws Exception
   {
      super.testSingleWaypoint();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 29.1)
   @Test(timeout = 150000)
   public void testSingleWaypointAndAbort() throws Exception
   {
      super.testSingleWaypointAndAbort();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 75.8)
   @Test(timeout = 380000)
   public void testSingleWaypointAndWalk() throws Exception
   {
      super.testSingleWaypointAndWalk();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 29.3)
   @Test(timeout = 150000)
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }
}
