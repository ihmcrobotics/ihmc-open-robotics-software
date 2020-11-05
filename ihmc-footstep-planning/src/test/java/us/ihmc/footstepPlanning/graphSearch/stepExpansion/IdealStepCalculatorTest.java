package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepCheckerInterface;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Arrays;
import java.util.Random;

public class IdealStepCalculatorTest
{
   @Test
   public void testStanceWidthAndLength()
   {
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      // make sure it lines up with the lattice for testing
      footstepPlannerParameters.setIdealFootstepWidth(0.3);
      footstepPlannerParameters.setIdealFootstepLength(0.4);

      FootstepCheckerInterface checker = (touchdown, stance, startOfSwing) -> true;
      WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder = new WaypointDefinedBodyPathPlanHolder();

      double pathLength = 20.0;
      Pose3D startPose = new Pose3D(-0.5 * pathLength, 0.0, 0.0, 0.0, 0.0, 0.0);
      Pose3D goalPose = new Pose3D(0.5 * pathLength, 0.0, 0.0, 0.0, 0.0, 0.0);
      bodyPathPlanHolder.setPoseWaypoints(Arrays.asList(startPose, goalPose));

      IdealStepCalculator idealStepCalculator = new IdealStepCalculator(footstepPlannerParameters,
                                                                        checker,
                                                                        bodyPathPlanHolder,
                                                                        new YoRegistry("testRegistry"));

      SideDependentList<Pose3D> goalStepPoses = PlannerTools.createSquaredUpFootsteps(goalPose, footstepPlannerParameters.getIdealFootstepWidth());
      SideDependentList<DiscreteFootstep> goalPoses = new SideDependentList<>(side ->
                                                                          {
                                                                             Pose3D goalStepPose = goalStepPoses.get(side);
                                                                             return new DiscreteFootstep(goalStepPose.getX(),
                                                                                                         goalStepPose.getY(),
                                                                                                         goalStepPose.getYaw(),
                                                                                                         side);
                                                                          });
      idealStepCalculator.initialize(goalPoses, 0.0);

      double idealStepLength = footstepPlannerParameters.getIdealFootstepLength();
      double idealStepWidth = footstepPlannerParameters.getIdealFootstepWidth();

      int numberOfTests = 10;
      Random random = new Random(10);
      for (int i = 0; i < numberOfTests; i++)
      {
         RobotSide side = RobotSide.generateRandomRobotSide(random);
         double startX = EuclidCoreRandomTools.nextDouble(random, 0.5 * pathLength - 1.0);

         DiscreteFootstep testStanceNode = new DiscreteFootstep(startX, 0.5 * side.negateIfRightSide(idealStepWidth), 0.0, side);
         DiscreteFootstep testSwingNode = new DiscreteFootstep(0.0, 0.0, 0.0, side.getOppositeSide());

         DiscreteFootstep idealStep = idealStepCalculator.computeIdealStep(testStanceNode, testSwingNode);

         double stepLength = idealStep.getX() - testStanceNode.getX();
         double stepWidth = side.negateIfLeftSide(idealStep.getY() - testStanceNode.getY());

         Assertions.assertTrue(MathTools.epsilonEquals(stepLength, idealStepLength, 1e-10));
         Assertions.assertTrue(MathTools.epsilonEquals(stepWidth, idealStepWidth, 1e-10));
      }
   }
}
