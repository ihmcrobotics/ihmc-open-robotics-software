package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseWobblyFootTest;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.WobblySimulationContactPoints;
import us.ihmc.yoVariables.variable.YoDouble;

public class AtlasObstacleCourseWobblyFootTest extends DRCObstacleCourseWobblyFootTest
{
   private static final double footZWobbleForTests = 0.01;

   @Override
   public DRCRobotModel getRobotModel()
   {
      final AtlasRobotVersion atlasVersion = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
      FootContactPoints<RobotSide> simulationContactPoints = new WobblySimulationContactPoints(footZWobbleForTests);
      return new AtlasRobotModel(atlasVersion, RobotTarget.SCS, false, simulationContactPoints);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @Test
   public void testStandingForACoupleSecondsWithWobblyFeet() throws SimulationExceededMaximumTimeException
   {
      super.testStandingForACoupleSecondsWithWobblyFeet();
   }

   @Override
   @Disabled
   @Test
   public void testTurningInPlaceAndPassingPIWithWobblyFeet() throws SimulationExceededMaximumTimeException
   {
      super.testTurningInPlaceAndPassingPIWithWobblyFeet();
   }

   @Override
   @Disabled
   @Test
   public void testWalkingUpToRampWithShortStepsWithWobblyFeet() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpToRampWithShortStepsWithWobblyFeet();
   }

   @Override
   protected YoDouble getPelvisOrientationErrorVariableName(SimulationConstructionSet scs)
   {
      return (YoDouble) scs.getVariable("root.atlas.DRCSimulation.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WholeBodyControllerCore.WholeBodyFeedbackController.pelvisOrientationFBController.pelvisAxisAngleOrientationController",
            "pelvisRotationErrorInBodyZ");
   }
}
