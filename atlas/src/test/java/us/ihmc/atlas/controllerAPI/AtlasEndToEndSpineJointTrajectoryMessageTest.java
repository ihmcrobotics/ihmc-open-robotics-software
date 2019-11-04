package us.ihmc.atlas.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndSpineJointTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasEndToEndSpineJointTrajectoryMessageTest extends EndToEndSpineJointTrajectoryMessageTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSingleWaypoint() throws SimulationExceededMaximumTimeException
   {
      super.testSingleWaypoint();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testSwitchingBetweenControlModes() throws SimulationExceededMaximumTimeException
   {
      super.testSwitchingBetweenControlModes();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testDesiredsAreContinuous() throws SimulationExceededMaximumTimeException
   {
      super.testDesiredsAreContinuous();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testMultipleWaypoints() throws SimulationExceededMaximumTimeException
   {
      super.testMultipleWaypoints();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testLongMessage() throws SimulationExceededMaximumTimeException
   {
      super.testLongMessage();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testMessageQueuing() throws SimulationExceededMaximumTimeException
   {
      super.testMessageQueuing();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testMessageWithDifferentTrajectoryLengthsPerJoint() throws SimulationExceededMaximumTimeException
   {
      super.testMessageWithDifferentTrajectoryLengthsPerJoint();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testStreaming() throws Exception
   {
      super.testStreaming();
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
