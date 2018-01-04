package us.ihmc.valkyrie.simulation;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCPelvisLowGainsTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculatorListener;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.yoVariables.variable.YoDouble;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.EXCLUDE})
public class ValkyriePelvisLowGainsTest extends DRCPelvisLowGainsTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false)
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
   @ContinuousIntegrationTest(estimatedDuration = 38.0)
   @Test(timeout = 30000)
   public void testStandingWithLowPelvisOrientationGains() throws SimulationExceededMaximumTimeException
   {
      super.testStandingWithLowPelvisOrientationGains();
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

   @Override
   protected YoDouble getPelvisOrientationErrorVariableName(SimulationConstructionSet scs)
   {
      return (YoDouble) scs.getVariable("HighLevelHumanoidControllerFactory.PelvisOrientationManager.RootJointAngularAccelerationControlModule.v1PelvisAxisAngleOrientationController",
                                                "v1PelvisOrientationErrorMagnitude");
   }

   @Override
   public InverseDynamicsCalculatorListener getInverseDynamicsCalculatorListener(FullRobotModel controllersFullRobotModel, FloatingRootJointRobot robot)
   {
      return null;
   }
}
