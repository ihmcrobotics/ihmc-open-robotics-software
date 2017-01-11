package us.ihmc.atlas.ObstacleCourseTests;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.obstacleCourseTests.DRCObstacleCourseWobblyFootTest;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.wholeBodyController.SimulationFootContactPoints;
import us.ihmc.wholeBodyController.WobblySimulationContactPoints;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.SLOW, IntegrationCategory.VIDEO})
public class AtlasObstacleCourseWobblyFootTest extends DRCObstacleCourseWobblyFootTest
{
   private static final double footZWobbleForTests = 0.01;

   @Override
   public DRCRobotModel getRobotModel()
   {
      final AtlasRobotVersion atlasVersion = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
      SimulationFootContactPoints simulationContactPoints = new WobblySimulationContactPoints(footZWobbleForTests);
      return new AtlasRobotModel(atlasVersion, DRCRobotModel.RobotTarget.SCS, false, simulationContactPoints);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   protected DoubleYoVariable getPelvisOrientationErrorVariableName(SimulationConstructionSet scs)
   {
      return (DoubleYoVariable) scs.getVariable("root.atlas.DRCSimulation.DRCControllerThread.DRCMomentumBasedController.HighLevelHumanoidControllerManager.MomentumBasedControllerFactory.WholeBodyControllerCore.WholeBodyFeedbackController.pelvisOrientationFBController.pelvisAxisAngleOrientationController",
            "pelvisRotationErrorInBodyZ");
   }
}
