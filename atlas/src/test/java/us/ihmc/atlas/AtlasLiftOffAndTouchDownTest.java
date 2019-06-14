package us.ihmc.atlas;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.avatar.AvatarLiftOffAndTouchDownTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.robotics.Assert;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasLiftOffAndTouchDownTest
{
   private final double footLength = new AtlasPhysicalProperties().getFootLengthForControl();

   @Test
   public void testForwardStepRotated() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
      DRCSimulationTestHelper testHelper = AvatarLiftOffAndTouchDownTest.setupTest(robotModel, Math.toRadians(90.0));

      double stepLength = 0.4;
      double startPitch = Math.toRadians(20.0);
      double finalPitch = Math.toRadians(-20.0);

      boolean success = AvatarLiftOffAndTouchDownTest.doStep(robotModel, testHelper, stepLength, startPitch, finalPitch, footLength);

      testHelper.destroySimulation();

      Assert.assertTrue("Foot pitch in test did not match expected at checkpoints.", success);
   }

   @Test
   public void testForwardStepWithAdjustment() throws SimulationExceededMaximumTimeException
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
      DRCSimulationTestHelper testHelper = AvatarLiftOffAndTouchDownTest.setupTest(robotModel);

      double stepLength = 0.4;
      double startPitch = Math.toRadians(20.0);
      double finalPitch = Math.toRadians(-20.0);

      boolean success = AvatarLiftOffAndTouchDownTest.doStep(robotModel, testHelper, stepLength, startPitch, finalPitch, footLength, 0.1);

      testHelper.destroySimulation();

      Assert.assertTrue("Foot pitch in test did not match expected at checkpoints.", success);
   }
}
