package us.ihmc.atlas.controllerAPI;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.controllerAPI.EndToEndPelvisOrientationTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasEndToEndPelvisOrientationTest extends EndToEndPelvisOrientationTest
{
   private DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Tag("controller-api-2")
   @Override
   @Test
   public void testGoHome() throws SimulationExceededMaximumTimeException
   {
      super.testGoHome();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSingleTrajectoryPoint() throws SimulationExceededMaximumTimeException
   {
      super.testSingleTrajectoryPoint();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testQueue() throws SimulationExceededMaximumTimeException
   {
      super.testQueue();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testWalking() throws SimulationExceededMaximumTimeException
   {
      super.testWalking();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testWalkingAfterTrajectory() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingAfterTrajectory();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testMultipleTrajectoryPoints() throws SimulationExceededMaximumTimeException
   {
      super.testMultipleTrajectoryPoints();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testWalkingWithUserControl() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingWithUserControl();
   }

   @Tag("controller-api-slow-2")
   @Override
   @Test
   public void testCustomControlFrame() throws SimulationExceededMaximumTimeException
   {
      super.testCustomControlFrame();
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
