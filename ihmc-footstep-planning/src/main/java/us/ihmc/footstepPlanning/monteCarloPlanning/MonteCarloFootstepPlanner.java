package us.ihmc.footstepPlanning.monteCarloPlanning;

import us.ihmc.commons.Conversions;
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

/**
 * Monte Carlo Footstep Planner (MCFP)
 * Author(s): Bhavyansh Mishra
 * <p>
 * This class is the main implementation of the Monte Carlo Footstep Planner. It uses a Monte Carlo Tree Search (MCTS) algorithm to generate a footstep plan.
 * This class maintains
 * a Directed Acyclic Bipartite Footstep Graph (DAB-FG) of MonteCarloFootstepNode's and evolves this graph using the following four subroutines:
 * 1. Selection: Selects the best child node to explore next based on the Upper Confidence Bound (UCB) of the child nodes.
 * 2. Expansion: Expands the selected node by generating all possible child nodes from the current node.
 * 3. Simulation: Simulates a random walk from the expanded node to a terminal node.
 * 4. Back Propagation: Propagates the simulation score back up the tree to update the value of the nodes.
 * <p>
 * The graph is allowed to evolve for either desired number of graph update iterations or until a timeout is reached. The final footstep plan is generated by
 * selecting the
 * path from root to the deepest leaf node. The final footstep plan is then returned to the caller. Traversability-based guidance is possible given a
 * traversability graph.
 * Such a body path planning data structure can help improve the quality of the footstep plan by providing a heuristic to guide the search towards more
 * traversable regions of the terrain.
 */
public class MonteCarloFootstepPlanner
{
   private final MonteCarloFootstepPlannerStatistics statistics = new MonteCarloFootstepPlannerStatistics();
   ;
   private final HashMap<MonteCarloFootstepNode, MonteCarloFootstepNode> visitedNodes = new HashMap<>();
   private final Random random = new Random();

   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final MonteCarloFootstepPlannerParameters parameters;
   private MonteCarloFootstepPlannerRequest request;
   private final TerrainPlanningDebugger debugger;
   private MonteCarloFootstepNode root;

   private boolean planning = false;

   private double timeSpentPlanningSoFar = 0;
   private double startTime = 0;
   private int uniqueNodeId = 0;

   public MonteCarloFootstepPlanner(MonteCarloFootstepPlannerParameters parameters,
                                    SideDependentList<ConvexPolygon2D> footPolygons,
                                    TerrainPlanningDebugger debugger)
   {
      this.parameters = parameters;
      this.footPolygons = footPolygons;
      this.debugger = debugger;
   }

   /**
    * Main function to generate a footstep plan using the Monte Carlo Footstep Planner.
    *
    * @param request Monte Carlo Footstep Planner Request
    * @return Footstep Plan
    */
   public FootstepPlan generateFootstepPlan(MonteCarloFootstepPlannerRequest request)
   {
      this.startTime = Conversions.nanosecondsToSeconds(System.nanoTime());
      this.request = request;
      planning = true;

      // Debug Only
      debugger.setRequest(request);
      debugger.refresh(request.getTerrainMapData());
      statistics.startTotalTime();

      // Initialize Root
      if (root == null)
      {
         Point2D position = new Point2D(
               request.getStartFootPoses().get(request.getRequestedInitialStanceSide()).getPosition().getX() * parameters.getNodesPerMeter(),
               request.getStartFootPoses().get(request.getRequestedInitialStanceSide()).getPosition().getY() * parameters.getNodesPerMeter());
         float yaw = (float) request.getStartFootPoses().get(request.getRequestedInitialStanceSide()).getYaw();
         Point3D state = new Point3D(position.getX(), position.getY(), yaw);
         root = new MonteCarloFootstepNode(state, null, request.getRequestedInitialStanceSide(), uniqueNodeId++);
      }

      // Perform Monte-Carlo Tree Search
      for (int i = 0; i < parameters.getNumberOfIterations(); i++)
      {
         updateTree(root, request);
      }

      // Compute plan from maximum value path in the tree so far
      FootstepPlan plan = MonteCarloPlannerTools.getFootstepPlanFromTree(root, request, parameters, footPolygons);

      // Debug Only
      debugger.printScoreStats(root, request, parameters);
      statistics.stopTotalTime();
      statistics.setLayerCountsString(MonteCarloPlannerTools.getLayerCountsString(root));
      statistics.logToFile(true, true);

      if (timeSpentPlanningSoFar > request.getTimeout())
         LogTools.warn("[Monte-Carlo Footstep Planner] Timeout Reached: Spent:{} / Allowed:{}", timeSpentPlanningSoFar, request.getTimeout());

      planning = false;
      return plan;
   }

   /**
    * Single update iteration of the MCFP algorithm. This function recursively updates the DAB-FG using expand(), simulate(), and backPropagate()
    * until a timeout is reached.
    *
    * @param node    Current node to start update from
    * @param request Monte Carlo Footstep Planner Request
    */
   public void updateTree(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      timeSpentPlanningSoFar = Conversions.nanosecondsToSeconds(System.nanoTime()) - startTime;
      if (timeSpentPlanningSoFar > request.getTimeout())
      {
         return;
      }

      if (node == null)
      {
         LogTools.debug("Node is null");
         return;
      }

      // Pruning and Sorting
      statistics.startPruningTime();
      node.sortChildren();
      node.prune(parameters.getMaxNumberOfChildNodes());
      statistics.stopPruningTime();

      if (node.getChildren().isEmpty())
      {
         MonteCarloFootstepNode childNode = node;
         if (node.getLevel() < parameters.getMaxTreeDepth())
         {
            // Expansion and Random Selection
            statistics.startExpansionTime();
            childNode = expand(node, request);
            statistics.stopExpansionTime();
         }

         if (childNode != null)
         {
            // Simulation
            statistics.startSimulationTime();
            double score = simulate(childNode, request);
            statistics.stopSimulationTime();

            // Back Propagation
            statistics.startPropagationTime();
            childNode.setValue((float) score);
            backPropagate(node, (float) score);
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

         // Recursion into highest UCB node
         updateTree(bestNode, request);
      }
   }

   /**
    * Expansion subroutine of the MCFP algorithm. This function generates all possible child nodes from the current node.
    *
    * @param node    Current node to expand
    * @param request Monte Carlo Footstep Planner Request
    * @return Best child node to explore next
    */
   public MonteCarloFootstepNode expand(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      ArrayList<?> availableStates = node.getAvailableStates(request, parameters);
      for (Object newStateObj : availableStates)
      {
         MonteCarloFootstepNode newState = (MonteCarloFootstepNode) newStateObj;
         double score = MonteCarloPlannerTools.scoreFootstepNode(node, newState, request, parameters, false);
         if (score > parameters.getInitialValueCutoff())
         {
            // Create node if not previously visited, or pull from visited node map
            if (visitedNodes.getOrDefault(newState, null) != null)
            {
               MonteCarloFootstepNode existingNode = visitedNodes.get(newState);
               node.addChild(existingNode);
               existingNode.getParents().add(node);
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

   /**
    * Simulation subroutine of the MCFP algorithm. This function simulates a random walk from the expanded node to a terminal node.
    *
    * @param node    Current node to simulate from
    * @param request Monte Carlo Footstep Planner Request
    * @return Score of the simulation
    */
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

   public void backPropagate(MonteCarloFootstepNode node, float score)
   {
      if (timeSpentPlanningSoFar > request.getTimeout())
         return;

      node.setValue(Math.max((float) (score * parameters.getDecayFactor()), node.getValue()));
      node.incrementVisits();

      if (!node.getParents().isEmpty())
      {
         for (MonteCarloTreeNode parent : node.getParents())
         {
            backPropagate((MonteCarloFootstepNode) parent, score);
         }
      }
   }

   /**
    * Resets the planner to start a new planning session.
    *
    * @param request Monte Carlo Footstep Planner Request
    */
   public void reset(MonteCarloFootstepPlannerRequest request)
   {
      random.setSeed(100); // fixed seed for deterministic behavior. Comment this out for stochasticity.
      uniqueNodeId = 0;
      visitedNodes.clear();

      if (request == null)
         root = new MonteCarloFootstepNode(new Point3D(), null, RobotSide.LEFT, uniqueNodeId++);
      else
         root = new MonteCarloFootstepNode(new Point3D(
               request.getStartFootPoses().get(request.getRequestedInitialStanceSide()).getPosition().getX() * parameters.getNodesPerMeter(),
               request.getStartFootPoses().get(request.getRequestedInitialStanceSide()).getPosition().getY() * parameters.getNodesPerMeter(),
               request.getStartFootPoses().get(request.getRequestedInitialStanceSide()).getYaw()),
                                           null,
                                           request.getRequestedInitialStanceSide(),
                                           uniqueNodeId++);
   }

   /**
    * Transitions the root of graph to the optimal footstep node
    *
    * @return Vector from root to optimal footstep node
    */
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
      action.scale(1 / parameters.getNodesPerMeter());
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
