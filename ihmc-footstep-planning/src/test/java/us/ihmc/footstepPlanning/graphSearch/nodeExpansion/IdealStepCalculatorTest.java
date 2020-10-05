package us.ihmc.footstepPlanning.graphSearch.nodeExpansion;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.footstepPlanning.FootstepPlanHeading;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Arrays;
import java.util.Random;
import java.util.function.BiPredicate;

public class IdealStepCalculatorTest
{
   @Test
   public void testStanceWidthAndLength()
   {
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      // make sure it lines up with the lattice for testing
      footstepPlannerParameters.setIdealFootstepWidth(0.3);
      footstepPlannerParameters.setIdealFootstepLength(0.4);

      BiPredicate<FootstepNode, FootstepNode> checker = (touchdown, stance) -> true;
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
      SideDependentList<FootstepNode> goalPoses = new SideDependentList<>(side ->
                                                                          {
                                                                             Pose3D goalStepPose = goalStepPoses.get(side);
                                                                             return new FootstepNode(goalStepPose.getX(),
                                                                                                     goalStepPose.getY(),
                                                                                                     goalStepPose.getYaw(),
                                                                                                     side);
                                                                          });
      idealStepCalculator.initialize(goalPoses, FootstepPlanHeading.FORWARD);

      double idealStepLength = footstepPlannerParameters.getIdealFootstepLength();
      double idealStepWidth = footstepPlannerParameters.getIdealFootstepWidth();

      int numberOfTests = 10;
      Random random = new Random(10);
      for (int i = 0; i < numberOfTests; i++)
      {
         RobotSide side = RobotSide.generateRandomRobotSide(random);
         double startX = EuclidCoreRandomTools.nextDouble(random, 0.5 * pathLength - 1.0);

         FootstepNode testNode = new FootstepNode(startX, 0.5 * side.negateIfRightSide(idealStepWidth), 0.0, side);
         FootstepNode idealStep = idealStepCalculator.computeIdealStep(testNode);

         double stepLength = idealStep.getX() - testNode.getX();
         double stepWidth = side.negateIfLeftSide(idealStep.getY() - testNode.getY());

         Assertions.assertTrue(MathTools.epsilonEquals(stepLength, idealStepLength, 1e-10));
         Assertions.assertTrue(MathTools.epsilonEquals(stepWidth, idealStepWidth, 1e-10));
      }
   }
}
