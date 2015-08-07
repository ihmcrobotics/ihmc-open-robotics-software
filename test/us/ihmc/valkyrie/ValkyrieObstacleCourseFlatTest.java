package us.ihmc.valkyrie;

import javax.vecmath.Vector3d;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseFlatTest;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.agileTesting.BambooAnnotations.QuarantinedTest;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

@BambooPlan(planType = {BambooPlanType.Slow, BambooPlanType.VideoB})
public class ValkyrieObstacleCourseFlatTest extends DRCObstacleCourseFlatTest
{

   private final DRCRobotModel robotModel = new ValkyrieRobotModel(false, false);
   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   protected Vector3d getFootSlipVector()
   {
      return new Vector3d(0.02, -0.02, 0.0);
   }
   @Override
   protected DoubleYoVariable getPelvisOrientationErrorVariableName(SimulationConstructionSet scs)
   {
      return (DoubleYoVariable) scs.getVariable("MomentumBasedControllerFactory.PelvisOrientationManager.RootJointAngularAccelerationControlModule.v1PelvisAxisAngleOrientationController",
                                                "v1PelvisOrientationErrorMagnitude");
   }

   @Override
   protected double getFootSlipTimeDeltaAfterTouchdown()
   {
      return 0.1;
   }
   
   
   @Ignore
   @QuarantinedTest("Doesn't work with Valkyrie yet. Need to get it working some day")
   @Override
   @EstimatedDuration
   @Test(timeout = 300000)
   public void testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees() throws SimulationExceededMaximumTimeException
   {
      super.testWalkingUpToRampWithLongStepsAndOccasionallyStraightKnees();
   }

}
