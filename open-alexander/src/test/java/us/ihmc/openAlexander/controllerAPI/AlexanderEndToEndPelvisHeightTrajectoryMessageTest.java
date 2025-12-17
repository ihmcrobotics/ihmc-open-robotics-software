package us.ihmc.openAlexander.controllerAPI;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.controllerAPI.EndToEndPelvisHeightTrajectoryMessageTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.tools.CITools.SimpleRobotNameKeys;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AlexanderEndToEndPelvisHeightTrajectoryMessageTest extends EndToEndPelvisHeightTrajectoryMessageTest
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

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSingleWaypoint() throws Exception
   {
      super.testSingleWaypoint();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSingleWaypointInUserMode() throws Exception
   {
      super.testSingleWaypointInUserMode();
   }

   /*
    * FIXME This test wasn't running for a very long time and failed when re-enabled. Need to fix it
    * and re-enbable it.
    */
   @Disabled
   @Tag("controller-api-slow-2")
   @Override
   @Test
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
   @Override
   @Test
   public void testSingleWaypointWithControlFrame() throws SimulationExceededMaximumTimeException
   {
      super.testSingleWaypointWithControlFrame();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testStopAllTrajectory() throws Exception
   {
      super.testStopAllTrajectory();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testStreaming() throws Exception
   {
      super.testStreaming();
   }
}
