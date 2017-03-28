package us.ihmc.footstepPlanning.aStar;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationTools;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.footstepPlanning.aStar.implementations.EuclidianBasedCost;
import us.ihmc.footstepPlanning.aStar.implementations.EuclidianDistanceHeuristics;
import us.ihmc.footstepPlanning.aStar.implementations.SimpleGridResolutionBasedExpansion;
import us.ihmc.footstepPlanning.aStar.implementations.SimpleNodeChecker;
import us.ihmc.footstepPlanning.aStar.implementations.SimpleSideBasedExpansion;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class AStarPlanarRegionsPlannerTest
{
   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testFootstepGraph()
   {
      FootstepNode startNode = new FootstepNode(0.0, 0.0);
      FootstepNode goalNode = new FootstepNode(4.0, 0.0);
      FootstepGraph graph = new FootstepGraph(startNode);
      double transitionCost = 1.0;

      // assemble simple graph structure
      graph.checkAndSetEdge(new FootstepNode(0.0, 0.0), new FootstepNode(1.0, 1.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(0.0, 0.0), new FootstepNode(1.0, 0.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(0.0, 0.0), new FootstepNode(1.0, -1.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(1.0, 1.0), new FootstepNode(2.0, 1.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(1.0, -1.0), new FootstepNode(2.0, -1.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(2.0, 1.0), new FootstepNode(3.0, 1.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(2.0, -1.0), new FootstepNode(3.0, 1.0), transitionCost);
      graph.checkAndSetEdge(new FootstepNode(3.0, 1.0), goalNode, transitionCost);
      assertEquals(graph.getCostFromStart(goalNode), 4.0 * transitionCost, 1.0e-10);

      // add new edge that makes goal cheaper and check goal cost
      graph.checkAndSetEdge(new FootstepNode(1.0, 0.0), new FootstepNode(3.0, 1.0), transitionCost);
      assertEquals(graph.getCostFromStart(goalNode), 3.0 * transitionCost, 1.0e-10);

      // add new edge that should have no effect
      graph.checkAndSetEdge(new FootstepNode(2.0, -1.0), goalNode, transitionCost);
      assertEquals(graph.getCostFromStart(goalNode), 3.0 * transitionCost, 1.0e-10);

      // change goal node, add edge, and check cost
      FootstepNode newGoalNode = new FootstepNode(5.0, 0.0);
      graph.checkAndSetEdge(goalNode, newGoalNode, transitionCost);
      assertEquals(graph.getCostFromStart(newGoalNode), 4.0 * transitionCost, 1.0e-10);

      // update edge cost to be negative and make sure path to goal was updated
      graph.checkAndSetEdge(startNode, new FootstepNode(2.0, 1.0), -2.0);
      assertEquals(graph.getCostFromStart(newGoalNode), 1.0 * transitionCost, 1.0e-10);

      // check that goal path matches expected
      List<FootstepNode> pathToGoal = graph.getPathFromStart(newGoalNode);
      List<FootstepNode> expectedPathToGoal = new ArrayList<>();
      expectedPathToGoal.add(startNode);
      expectedPathToGoal.add(new FootstepNode(2.0, 1.0));
      expectedPathToGoal.add(new FootstepNode(3.0, 1.0));
      expectedPathToGoal.add(goalNode);
      expectedPathToGoal.add(newGoalNode);
      assertEquals(pathToGoal.size(), expectedPathToGoal.size());
      for (int i = 0; i < expectedPathToGoal.size(); i++)
      {
         FootstepNode node = pathToGoal.get(i);
         FootstepNode expectedNode = expectedPathToGoal.get(i);
         assertEquals(node, expectedNode);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testFootstepNode()
   {
      double gridX = FootstepNode.gridSizeX;
      double gridY = FootstepNode.gridSizeY;
      FootstepNode node;

      node = new FootstepNode(gridX * 0.3, 0.0);
      assertEquals(0.0, node.getX(), 1.0e-10);
      assertEquals(0.0, node.getY(), 1.0e-10);
      int hash1 = node.hashCode();

      node = new FootstepNode(gridX * 0.1, -gridY * 0.2);
      assertEquals(0.0, node.getX(), 1.0e-10);
      assertEquals(0.0, node.getY(), 1.0e-10);
      int hash2 = node.hashCode();

      assertEquals(hash1, hash2);

      node = new FootstepNode(gridX * 0.8, 0.0);
      assertEquals(gridX, node.getX(), 1.0e-10);
      assertEquals(0.0, node.getY(), 1.0e-10);

      node = new FootstepNode(gridX * 3.8, -gridY * 8.1);
      assertEquals(4.0 * gridX, node.getX(), 1.0e-10);
      assertEquals(-8.0 * gridY, node.getY(), 1.0e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testNodeExpansion()
   {
      FootstepNode node = new FootstepNode(0.0, 0.0, Math.PI, RobotSide.RIGHT);

      SimpleSideBasedExpansion expansion = new SimpleSideBasedExpansion();
      HashSet<FootstepNode> neighbors = expansion.expandNode(node);

      YoVariableRegistry registry = new YoVariableRegistry("Test");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));

      YoFramePose originPose = new YoFramePose("OrgionPose", ReferenceFrame.getWorldFrame(), registry);
      originPose.setYawPitchRoll(node.getYaw(), 0.0, 0.0);
      originPose.setXYZ(node.getX(), node.getY(), 0.0);
      YoGraphicCoordinateSystem originNode = new YoGraphicCoordinateSystem("OrginNode", originPose, 0.4);
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), originNode);

      int count = 0;
      for (FootstepNode neighbor : neighbors)
      {
         YoFramePose pose = new YoFramePose("NeighborPose" + count, ReferenceFrame.getWorldFrame(), registry);
         pose.setYawPitchRoll(neighbor.getYaw(), 0.0, 0.0);
         pose.setXYZ(neighbor.getX(), neighbor.getY(), 0.0);
         YoGraphicCoordinateSystem neighborNode = new YoGraphicCoordinateSystem("NeighborNode" + count, pose, 0.1);
         graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), neighborNode);
         count++;
      }

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      if (visualize)
      {
         scs.startOnAThread();
         ThreadTools.sleepForever();
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 300000)
   public void testSimpleExpansion()
   {
      // make planar regions
      PlanarRegionsListGenerator generator = new PlanarRegionsListGenerator();
      generator.translate(0.0, 0.0, 0.0001);
      generator.addRectangle(0.4, 0.4);
      generator.translate(0.5, 0.4, 0.0);
      generator.addRectangle(1.4, 0.4);
      generator.translate(0.5, -0.4, 0.0);
      generator.addRectangle(0.4, 0.4);
      PlanarRegionsList planarRegionsList = generator.getPlanarRegionsList();

      // make goal and initial conditions
      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      FramePose goalPose = new FramePose(ReferenceFrame.getWorldFrame());
      goalPose.setX(1.0);
      goal.setGoalPoseBetweenFeet(goalPose);
      FramePose startPose = new FramePose();
      RobotSide startSide = RobotSide.LEFT;

      // create planner
      SimpleNodeChecker nodeChecker = new SimpleNodeChecker();
      EuclidianDistanceHeuristics heuristics = new EuclidianDistanceHeuristics();
      SimpleGridResolutionBasedExpansion expansion = new SimpleGridResolutionBasedExpansion();
      EuclidianBasedCost stepCostCalculator = new EuclidianBasedCost();
      FootstepNodeVisualization viz = null;
      if (visualize)
         viz = new FootstepNodeVisualization(1000, 0.04, planarRegionsList);
      AStarFootstepPlanner planner = new AStarFootstepPlanner(nodeChecker, heuristics, expansion, stepCostCalculator, viz);

      // plan
      planner.setPlanarRegions(planarRegionsList);
      planner.setGoal(goal);
      planner.setInitialStanceFoot(startPose, startSide);

      if (!visualize)
      {
         assertEquals(FootstepPlanningResult.OPTIMAL_SOLUTION, planner.plan());
         FootstepPlan plan = planner.getPlan();
         SimpleFootstep lastStep = plan.getFootstep(plan.getNumberOfSteps() - 1);
         FramePose achievedGoalPose = new FramePose();
         lastStep.getSoleFramePose(achievedGoalPose);
         assertTrue(goalPose.epsilonEquals(achievedGoalPose, FootstepNode.gridSizeX));

         planner.setWeight(5.0);
         assertEquals(FootstepPlanningResult.SUB_OPTIMAL_SOLUTION, planner.plan());

         planner.setTimeout(1.0e-10);
         assertEquals(FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION, planner.plan());

         planner.setTimeout(Double.POSITIVE_INFINITY);
         planner.setPlanarRegions(new PlanarRegionsList());
         assertEquals(FootstepPlanningResult.NO_PATH_EXISTS, planner.plan());
      }
      else
      {
         planner.plan();
         viz.showAndSleep(true);
      }
   }
}
