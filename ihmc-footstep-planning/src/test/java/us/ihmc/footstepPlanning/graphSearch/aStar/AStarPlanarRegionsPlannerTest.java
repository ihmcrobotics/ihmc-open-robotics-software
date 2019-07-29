package us.ihmc.footstepPlanning.graphSearch.aStar;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;

import com.google.common.util.concurrent.AtomicDouble;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FlatGroundFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.heuristics.EuclideanDistanceHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SimpleNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.SimpleGridResolutionBasedExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.SimpleSideBasedExpansion;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.EuclideanBasedCost;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;

public class AStarPlanarRegionsPlannerTest
{
//   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   private static final boolean visualize = false;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testFootstepGraph()
   {
      FootstepNode startNode = new FootstepNode(0.0, 0.0);
      FootstepNode goalNode = new FootstepNode(4.0, 0.0);
      FootstepGraph graph = new FootstepGraph();
      graph.initialize(startNode);
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

   @Test
   public void testFootstepNode()
   {
      double gridX = LatticeNode.gridSizeXY;
      double gridY = LatticeNode.gridSizeXY;
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

   @Test
   public void testNodeExpansion()
   {
      if (!visualize)
         return;

      FootstepNode node = new FootstepNode(0.0, 0.0, Math.PI, RobotSide.RIGHT);

      FootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters();
      SimpleSideBasedExpansion expansion = new SimpleSideBasedExpansion(parameters);
      HashSet<FootstepNode> neighbors = expansion.expandNode(node);

      YoVariableRegistry registry = new YoVariableRegistry("Test");
      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));

      YoFramePoseUsingYawPitchRoll originPose = new YoFramePoseUsingYawPitchRoll("OrgionPose", ReferenceFrame.getWorldFrame(), registry);
      originPose.setYawPitchRoll(node.getYaw(), 0.0, 0.0);
      originPose.setXYZ(node.getX(), node.getY(), 0.0);
      YoGraphicCoordinateSystem originNode = new YoGraphicCoordinateSystem("OrginNode", originPose, 0.4);
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), originNode);

      int count = 0;
      for (FootstepNode neighbor : neighbors)
      {
         YoFramePoseUsingYawPitchRoll pose = new YoFramePoseUsingYawPitchRoll("NeighborPose" + count, ReferenceFrame.getWorldFrame(), registry);
         pose.setYawPitchRoll(neighbor.getYaw(), 0.0, 0.0);
         pose.setXYZ(neighbor.getX(), neighbor.getY(), 0.0);
         YoGraphicCoordinateSystem neighborNode = new YoGraphicCoordinateSystem("NeighborNode" + count, pose, 0.1);
         graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), neighborNode);
         count++;
      }

      scs.addYoVariableRegistry(registry);
      scs.addYoGraphicsListRegistry(graphicsListRegistry);
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   @Test
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
      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame());
      Point3D goalPosition = new Point3D(1.0, 0.0, 0.0);
      goalPose.setPosition(goalPosition);
      goal.setGoalPoseBetweenFeet(goalPose);
      FramePose3D startPose = new FramePose3D();
      RobotSide startSide = RobotSide.LEFT;

      // create planner
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      FootstepPlannerParametersReadOnly parameters = new DefaultFootstepPlannerParameters()
      {
         @Override
         public FootstepPlannerCostParameters getCostParameters()
         {
            return new DefaultFootstepPlannerCostParameters()
            {
               @Override
               public double getCostPerStep()
               {
                  return 0.0;
               }
            };
         }
      };
      FootstepNodeChecker nodeChecker = new SimpleNodeChecker();

      final AtomicDouble heuristicCost = new AtomicDouble(1.0);
      DoubleProvider heuristicCostProvider = () -> heuristicCost.get();
      EuclideanDistanceHeuristics heuristics = new EuclideanDistanceHeuristics(heuristicCostProvider);
      SimpleGridResolutionBasedExpansion expansion = new SimpleGridResolutionBasedExpansion();
      EuclideanBasedCost stepCostCalculator = new EuclideanBasedCost(parameters);
      FlatGroundFootstepNodeSnapper snapper = new FlatGroundFootstepNodeSnapper();
      FootstepNodeVisualization viz = null;
      if (visualize)
         viz = new FootstepNodeVisualization(1000, 0.04, planarRegionsList);
      AStarFootstepPlanner planner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, expansion, stepCostCalculator, snapper, viz, null, registry);

      // plan
      planner.setPlanarRegions(planarRegionsList);
      planner.setGoal(goal);
      planner.setInitialStanceFoot(startPose, startSide);

      if (!visualize)
      {
         assertEquals(FootstepPlanningResult.OPTIMAL_SOLUTION, planner.plan());
         FootstepPlan plan = planner.getPlan();
         SimpleFootstep lastStep = plan.getFootstep(plan.getNumberOfSteps() - 1);
         FramePose3D achievedGoalPose = new FramePose3D();
         lastStep.getSoleFramePose(achievedGoalPose);

         goalPose.setY(-parameters.getIdealFootstepWidth() / 2.0);
         assertTrue(goalPose.epsilonEquals(achievedGoalPose, LatticeNode.gridSizeXY));

         heuristicCost.set(5.0);

         assertEquals(FootstepPlanningResult.SUB_OPTIMAL_SOLUTION, planner.plan());

         planner.setTimeout(1.0e-10);
         assertEquals(FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION, planner.plan());

         planner.setTimeout(Double.POSITIVE_INFINITY);
         generator = new PlanarRegionsListGenerator();
         generator.addRectangle(0.05, 0.05);
         generator.translate(1.0, 0.0, 0.0);
         generator.addRectangle(0.05, 0.5);
         planner.setPlanarRegions(generator.getPlanarRegionsList());
         assertEquals(FootstepPlanningResult.NO_PATH_EXISTS, planner.plan());
      }
      else
      {
         planner.plan();
         viz.showAndSleep(true);
      }
   }
}
