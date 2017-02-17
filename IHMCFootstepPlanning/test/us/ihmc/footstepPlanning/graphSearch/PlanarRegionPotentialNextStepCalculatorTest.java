package us.ihmc.footstepPlanning.graphSearch;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.testTools.PlanningTestTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.thread.ThreadTools;

public class PlanarRegionPotentialNextStepCalculatorTest
{
   private final boolean visualize = false;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAndVizOnFlat()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestCalculator");

      SimulationConstructionSet scs = null;

      if (visualize)
      {
         scs = new SimulationConstructionSet(new Robot("TestRobot"));
         scs.setDT(0.01, 1);
         //         scs.setGroundVisible(false);
         scs.startOnAThread();
      }

      BipedalFootstepPlannerParameters parameters = createPlannerParameters(registry);

      PlanarRegionPotentialNextStepCalculator calculator = new PlanarRegionPotentialNextStepCalculator(parameters, registry, null);

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      SideDependentList<ConvexPolygon2d> footPolygonsInSoleFrame = PlanningTestTools.createDefaultFootPolygons();
      PlanarRegionBipedalFootstepPlannerVisualizer listener = new PlanarRegionBipedalFootstepPlannerVisualizer(100, footPolygonsInSoleFrame, registry,
                                                                                                               graphicsListRegistry);

//      calculator.setBipedalFootstepPlannerListener(listener);

      if (visualize)
      {
         scs.addYoGraphicsListRegistry(graphicsListRegistry);
         scs.addYoVariableRegistry(registry);
         listener.setTickAndUpdatable(scs);
      }

      Point3D goalPosition = new Point3D(5.0, 0.0, 0.0);
      AxisAngle goalOrientation = new AxisAngle(new Vector3D(0.0, 0.0, 1.0), 0.0);
      int maxNumberOfExpectedFootsteps = 20;

      testAndVisualizeToGoal(goalPosition, goalOrientation, calculator, listener, maxNumberOfExpectedFootsteps);

      goalPosition = new Point3D(0.0, 5.0, 0.0);
      goalOrientation = new AxisAngle(new Vector3D(0.0, 0.0, 1.0), 0.0);
      maxNumberOfExpectedFootsteps = 40;

      testAndVisualizeToGoal(goalPosition, goalOrientation, calculator, listener, maxNumberOfExpectedFootsteps);

      goalPosition = new Point3D(5.0, 0.0, 0.0);
      goalOrientation = new AxisAngle(new Vector3D(0.0, 0.0, 1.0), Math.PI);
      maxNumberOfExpectedFootsteps = 40;

      testAndVisualizeToGoal(goalPosition, goalOrientation, calculator, listener, maxNumberOfExpectedFootsteps);

      if (visualize)
      {
         scs.cropBuffer();
         scs.gotoInPoint();

         ThreadTools.sleepForever();
      }
   }

   private void testAndVisualizeToGoal(Point3D goalPosition, AxisAngle goalOrientation, PlanarRegionPotentialNextStepCalculator calculator, PlanarRegionBipedalFootstepPlannerVisualizer listener, int maxNumberOfExpectedFootsteps)
   {
      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      FramePose goalPoseBetweenFeet = new FramePose(ReferenceFrame.getWorldFrame(), goalPosition, goalOrientation);
      goal.setGoalPoseBetweenFeet(goalPoseBetweenFeet);
      calculator.setGoal(goal);

      RobotSide footstepSide = RobotSide.LEFT;
      RigidBodyTransform soleTransform = new RigidBodyTransform();

      ArrayList<BipedalFootstepPlannerNode> nodes = new ArrayList<>();

      BipedalFootstepPlannerNode node = new BipedalFootstepPlannerNode(footstepSide, soleTransform);
      calculator.setStartNode(node);
      nodes.add(node);

      BipedalFootstepPlannerNode endNode = node;

      while ((node != null) && (!node.isAtGoal()))
      {
         ArrayList<BipedalFootstepPlannerNode> childrenNodes = calculator.computeChildrenNodes(node, Double.POSITIVE_INFINITY);

         if (childrenNodes.size() > 0)
         {
            BipedalFootstepPlannerNode parentNode = node;
            node = childrenNodes.get(childrenNodes.size() - 1);
            node.setParentNode(parentNode);
            nodes.add(node);
            endNode = node;
         }
         else
            node = null;
      }

      FootstepPlan footstepPlan = new FootstepPlan(endNode);
//      System.out.println(footstepPlan.getNumberOfSteps());
      listener.solutionWasFound(footstepPlan);

      assertTrue(footstepPlan.getNumberOfSteps() <= maxNumberOfExpectedFootsteps);
   }

   private BipedalFootstepPlannerParameters createPlannerParameters(YoVariableRegistry registry)
   {
      BipedalFootstepPlannerParameters parameters = new BipedalFootstepPlannerParameters(registry);

      parameters.setMaximumStepReach(0.55);
      parameters.setMaximumStepZ(0.25);

      parameters.setMaximumStepXWhenForwardAndDown(0.2);
      parameters.setMaximumStepZWhenForwardAndDown(0.10);

      parameters.setMaximumStepYaw(0.15);
      parameters.setMaximumStepWidth(0.4);
      parameters.setMinimumStepWidth(0.15);
      parameters.setMinimumFootholdPercent(0.95);

      parameters.setWiggleInsideDelta(0.08);
      parameters.setMaximumXYWiggleDistance(1.0);
      parameters.setMaximumYawWiggle(0.1);

      double idealFootstepLength = 0.3;
      double idealFootstepWidth = 0.2;
      parameters.setIdealFootstep(idealFootstepLength, idealFootstepWidth);

      return parameters;
   }

}
