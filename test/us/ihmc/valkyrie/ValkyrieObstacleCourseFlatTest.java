package us.ihmc.valkyrie;

import javax.vecmath.Vector3d;

import us.ihmc.bambooTools.BambooTools;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseFlatTest;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class ValkyrieObstacleCourseFlatTest extends DRCObstacleCourseFlatTest
{

   private DRCRobotModel robotModel = new ValkyrieRobotModel(false, false);
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

}
