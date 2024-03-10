package us.ihmc.footstepPlanning.monteCarloPlanning;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

public class MonteCarloFootstepPlanner
{
   private MonteCarloFootstepPlannerStatistics statistics;
   private MonteCarloFootstepPlannerParameters parameters;
   private TerrainPlanningDebugger debugger;
   private MonteCarloFootstepNode root;

   private MonteCarloFootstepPlannerRequest request;
   private final HashMap<MonteCarloFootstepNode, MonteCarloFootstepNode> visitedNodes = new HashMap<>();
   private final Random random = new Random();
   private SideDependentList<ConvexPolygon2D> footPolygons;

   private double startTime = 0;
   private double timeSpentPlanningSoFar = 0;

   private boolean planning = false;
   private int uniqueNodeId = 0;
   private int cellsPerMeter = 50;

   public MonteCarloFootstepPlanner(MonteCarloFootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons, TerrainPlanningDebugger debugger)
   {
      this.parameters = parameters;
      this.footPolygons = footPolygons;
      this.statistics = new MonteCarloFootstepPlannerStatistics();
      this.debugger = debugger;
   }

   public FootstepPlan generateFootstepPlan(MonteCarloFootstepPlannerRequest request)
   {
      this.startTime = System.nanoTime() / 1e9;
      this.request = request;
      planning = true;

      // Debug Only
      debugger.setRequest(request);
      debugger.refresh(request.getTerrainMapData());
      statistics.startTotalTime();

      // Initialize Root
      if (root == null)
      {
         Point2D position = new Point2D(request.getStartFootPoses().get(request.getRequestedInitialStanceSide()).getPosition().getX() * cellsPerMeter,
                                        request.getStartFootPoses().get(request.getRequestedInitialStanceSide()).getPosition().getY() * cellsPerMeter);
         float yaw = (float) request.getStartFootPoses().get(request.getRequestedInitialStanceSide()).getYaw();
         Point3D state = new Point3D(position.getX(), position.getY(), yaw);
         root = new MonteCarloFootstepNode(state, null, request.getRequestedInitialStanceSide(), uniqueNodeId++);
      }

      // Perform Monte-Carlo Tree Search
      for (int i = 0; i < parameters.getNumberOfIterations(); i++)
      {
         updateTree(root, request);
         debugger.printScoreStats(root, request, parameters, i);
      }

      // Compute plan from maximum value path in the tree so far
      FootstepPlan plan = MonteCarloPlannerTools.getFootstepPlanFromTree(root, request, footPolygons);

      // Debug Only

      statistics.stopTotalTime();
      statistics.setNumberOfNodesVisited(visitedNodes.size());
      statistics.setLayerCountsString(MonteCarloPlannerTools.getLayerCountsString(root));
      statistics.logToFile(true, true);

      if (timeSpentPlanningSoFar > request.getTimeout())
         LogTools.warn("[Monte-Carlo Footstep Planner] Timeout Reached: Spent:{} / Allowed:{}", timeSpentPlanningSoFar, request.getTimeout());

      planning = false;
      return plan;
   }

   public void updateTree(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      LogTools.debug("Update: Level: {}", node.getLevel());
      timeSpentPlanningSoFar = System.nanoTime() / 1e9 - startTime;

      //if (timeSpentPlanningSoFar > request.getTimeout())
      //{
      //   LogTools.debug("Timeout Reached: {}", timeSpentPlanningSoFar);
      //   return;
      //}

      // Pruning and Sorting
      statistics.startPruningTime();
      node.sortChildren();
      node.prune(parameters.getMaxNumberOfChildNodes());
      statistics.stopPruningTime();

      LogTools.debug("Update: Children: {}", node.getChildren().size());
      if (node.getChildren().isEmpty())
      {
         MonteCarloFootstepNode childNode = node;
         if (node.getLevel() < parameters.getMaxTreeDepth())
         {
            LogTools.debug("Expanding: {}", node.getLevel());
            // Expansion and Random Selection
            statistics.startExpansionTime();
            childNode = expand(node, request);
            statistics.stopExpansionTime();
         }
         else
         {
            LogTools.debug("Max Depth Reached: {}", node.getLevel());
         }

         if (childNode != null)
         {
            LogTools.debug("Simulating: {}", childNode.getLevel());
            // Simulation
            statistics.startSimulationTime();
            double score = simulate(childNode, request);
            statistics.stopSimulationTime();

            LogTools.debug("Back Propagating: {}", childNode.getLevel());
            // Back Propagation
            statistics.startPropagationTime();
            childNode.setValue((float) score);
            backPropagate(node, (float) score, 0);
            statistics.stopPropagationTime();
         }
      }
      else
      {
         // Maximum UCB Search
         statistics.startSearchTime();
         float bestScore = 0;
         MonteCarloFootstepNode bestNode = null;
         for (MonteCarloTreeNode child : node.getChildren())
         {
            child.updateUpperConfidenceBound(parameters.getExplorationConstant());
            if (child.getUpperConfidenceBound() >= bestScore)
            {
               bestScore = child.getUpperConfidenceBound();
               bestNode = (MonteCarloFootstepNode) child;
            }
         }
         statistics.stopSearchTime();

         LogTools.debug("Recursing: {}", bestNode.getLevel());
         // Recursion into highest UCB node
         updateTree(bestNode, request);
      }
   }

   public MonteCarloFootstepNode expand(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      ArrayList<?> availableStates = node.getAvailableStates(request, parameters);
      LogTools.debug("Total Available States: {} at Level: {}", availableStates.size(), node.level);
      for (Object newStateObj : availableStates)
      {
         MonteCarloFootstepNode newState = (MonteCarloFootstepNode) newStateObj;
         double score = MonteCarloPlannerTools.scoreFootstepNode(node, newState, request, parameters, false);
         if (score > parameters.getInitialValueCutoff())
         {
            statistics.incrementNumberOfNodesExpanded();
            // Create node if not previously visited, or pull from visited node map
            if (visitedNodes.getOrDefault(newState, null) != null)
            {
               visitedNodes.get(newState).setValue(Math.max((float) score, visitedNodes.get(newState).getValue()));
            }
            else
            {
               MonteCarloFootstepNode postNode = new MonteCarloFootstepNode(newState.getState(), node, newState.getRobotSide(), uniqueNodeId++);
               postNode.setValue((float) score);
               visitedNodes.put(newState, postNode);
               node.addChild(postNode);
            }
         }
      }

      if (node.getChildren().isEmpty())
      {
         LogTools.debug("No Children");
         return null;
      }

      if (node.getChildren().size() > 1)
         return (MonteCarloFootstepNode) node.getChildren().get(random.nextInt(0, node.getChildren().size() - 1));
      else
         return null;
   }

   public double simulate(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      double score = 0;

      MonteCarloFootstepNode simulationState = new MonteCarloFootstepNode(node.getState(), null, node.getRobotSide().getOppositeSide(), 0);

      for (int i = 0; i < parameters.getNumberOfSimulations(); i++)
      {
         ArrayList<MonteCarloFootstepNode> nextStates = simulationState.getAvailableStates(request, parameters);
         if (nextStates.isEmpty())
            break;

         int actionIndex = random.nextInt(0, nextStates.size());
         simulationState = nextStates.get(actionIndex);

         score += MonteCarloPlannerTools.scoreFootstepNode(node, simulationState, request, parameters, false);
      }

      return score;
   }

   public void backPropagate(MonteCarloFootstepNode node, float score, int callLevel)
   {
      if (timeSpentPlanningSoFar > request.getTimeout())
         return;

      //LogTools.debug("Back Propagate: {} {} {}", node.getLevel(), callLevel, score);

      node.setValue(Math.max( (float) (score * parameters.getDecayFactor()), node.getValue()));
      node.incrementVisits();

      if (!node.getParents().isEmpty() && node.getLevel() > 0 && callLevel < parameters.getMaxTreeDepth())
      {
         for (MonteCarloTreeNode parent : node.getParents())
         {
            backPropagate((MonteCarloFootstepNode) parent, score, callLevel + 1);
         }
      }
   }

   public void reset(MonteCarloFootstepPlannerRequest request)
   {
      random.setSeed(100);
      uniqueNodeId = 0;
      visitedNodes.clear();

      if (request == null)
         root = new MonteCarloFootstepNode(new Point3D(), null, RobotSide.LEFT, uniqueNodeId++);
      else
         root = new MonteCarloFootstepNode(new Point3D(request.getStartFootPoses().get(request.getRequestedInitialStanceSide()).getPosition().getX() * cellsPerMeter,
                                                       request.getStartFootPoses().get(request.getRequestedInitialStanceSide()).getPosition().getY() * cellsPerMeter,
                                                       request.getStartFootPoses().get(request.getRequestedInitialStanceSide()).getYaw()),
                                           null,
                                           request.getRequestedInitialStanceSide(),
                                           uniqueNodeId++);
   }

   public Vector3D transitionToOptimal()
   {
      if (root.getChildren().isEmpty())
      {
         reset(request);
         return new Vector3D();
      }

      for (MonteCarloTreeNode child : root.getChildren())
      {
         child.getParents().clear();
      }
      MonteCarloFootstepNode maxNode = (MonteCarloFootstepNode) root.getMaxQueueNode();

      root = maxNode;
      MonteCarloPlannerTools.pruneTree(root, parameters.getMaxNumberOfChildNodes());
      MonteCarloPlannerTools.resetNodeLevels(root, 0);

      Vector3D action = new Vector3D(maxNode.getState());
      action.sub(root.getState());
      action.scale(1 / 50.0);
      return action;
   }

   public MonteCarloTreeNode getRoot()
   {
      return root;
   }

   public List<MonteCarloFootstepNode> getVisitedNodes()
   {
      return new ArrayList<>(visitedNodes.values());
   }

   public boolean isPlanning()
   {
      return planning;
   }
}
