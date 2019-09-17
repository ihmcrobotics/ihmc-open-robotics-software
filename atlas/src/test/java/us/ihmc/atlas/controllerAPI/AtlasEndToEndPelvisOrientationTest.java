package us.ihmc.atlas.controllerAPI;

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

   @Override
   @Test
   public void testGoHome() throws SimulationExceededMaximumTimeException
   {
      super.testGoHome();
   }

   @Override
   @Test
   public void testSingleTrajectoryPoint() throws SimulationExceededMaximumTimeException
   {
      super.testSingleTrajectoryPoint();
   }

   @Override
   @Test
   public void testQueue() throws SimulationExceededMaximumTimeException
   {
      super.testQueue();
   }

   @Override
   @Test
   public void testWalking() throws SimulationExceededMaximumTimeException
   {
      super.testWalking();
   }

   @Override
   @Test
   public void testWalkingAfterTrajectory() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingAfterTrajectory();
   }

   @Override
   @Test
   public void testMultipleTrajectoryPoints() throws SimulationExceededMaximumTimeException
   {
      super.testMultipleTrajectoryPoints();
   }

   @Override
   @Test
   public void testWalkingWithUserControl() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingWithUserControl();
   }

   @Override
   @Test
   public void testCustomControlFrame() throws SimulationExceededMaximumTimeException
   {
      super.testCustomControlFrame();
   }

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
