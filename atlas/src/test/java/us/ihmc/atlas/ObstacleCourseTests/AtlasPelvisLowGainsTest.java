package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCPelvisLowGainsTest;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerToolbox;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.variable.YoDouble;

public class AtlasPelvisLowGainsTest extends DRCPelvisLowGainsTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false)
   {
      // Disable joint damping to make sure that damping isn't causing the problem.
      private static final boolean enableJointDamping = false;

      @Override
      public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
      {
         return createHumanoidFloatingRootJointRobot(createCollisionMeshes, enableJointDamping);
      }
   };

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
   @Test
   public void testStandingWithLowPelvisOrientationGains() throws SimulationExceededMaximumTimeException
   {
      super.testStandingWithLowPelvisOrientationGains();
   }

   @Override
   protected YoDouble getPelvisOrientationErrorVariableName(SimulationConstructionSet scs)
   {

      return (YoDouble) scs.getVariable(FeedbackControllerToolbox.class.getSimpleName(), "pelvisErrorRotationVectorZ");
   }
}
