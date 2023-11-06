package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * This class performs the Monte Carlo Tree Search for the Monte Carlo Planning agent. It uses the
 * Monte Carlo Tree Node class to store the state at each node in the tree and uses the Agent and World
 * classes to store the agent and world states for planning purposes.
 */
public class MonteCarloPlanner
{
   private int searchIterations = 15;
   private int simulationIterations = 100;
   private int uniqueNodeId = 0;

   private MonteCarloPlanningWorld world;
   private MonteCarloWaypointAgent agent;
   private MonteCarloWaypointNode root;

   int worldHeight = 200;
   int worldWidth = 200;
   int goalMargin = 5;

   int stepLength = 5;

   private final Point2D agentPos = new Point2D(0, 0);

   public MonteCarloPlanner(int offset)
   {
      agentPos.set(offset, offset);

      this.world = new MonteCarloPlanningWorld(goalMargin, worldHeight, worldWidth);
      this.agent = new MonteCarloWaypointAgent(agentPos);

      root = new MonteCarloWaypointNode(agent.getState(), null, uniqueNodeId++);
   }

   /**
    * Computes the next best action for the agent to take based on the current state of the world
    * and the agent. Updates the Monte Carlo Tree multiple times before selecting the best node
    * based on the Upper Confidence Bound.
    */
   public Point2DReadOnly plan()
   {
      if (root == null)
         return agent.getState();

      // Update the tree multiple times
      for (int i = 0; i < searchIterations; i++)
      {
         updateTree(root);
      }

      float bestScore = 0;
      MonteCarloWaypointNode bestNode = null;

      if (root.getChildren().isEmpty())
         LogTools.warn("No Children Nodes Found");

      // Select the best node based on the Upper Confidence Bound
      for (MonteCarloTreeNode node : root.getChildren())
      {
         node.updateUpperConfidenceBound();
         if (node.getUpperConfidenceBound() > bestScore)
         {
            bestScore = node.getUpperConfidenceBound();
            bestNode = (MonteCarloWaypointNode) node;
         }
      }

      root = bestNode;

      if (bestNode == null)
         return agent.getState();

      return bestNode.getPosition();
   }

   public void updateState(Point2DReadOnly newState)
   {
      updateWorld(newState);
      updateAgent(newState);
   }

   /**
    * Performs the Monte Carlo Tree Search Algorithm on the given sub-tree of the Monte Carlo Tree.
    *    1. If the node has not been visited
    *       1a. expand the node
    *       1b. compute its value by simulating with random actions.
    *       1c. back propagate the value to the root node.
    *    2. If the node has been visited
    *       2a. select and recurse into the child node with the highest Upper Confidence Bound (UCB)
    *       2b. until the node is a leaf node (unvisited node is reached)
    */
   public void updateTree(MonteCarloWaypointNode node)
   {
      if (node == null)
         return;

      if (node.getVisits() == 0)
      {
         MonteCarloWaypointNode childNode = expand(node);
         double score = simulate(childNode);
         childNode.setValue((float) score);
         backPropagate(node, (float) score);
      }
      else
      {
         float bestScore = 0;
         MonteCarloWaypointNode bestNode = null;
         for (MonteCarloTreeNode child : node.getChildren())
         {
            child.updateUpperConfidenceBound();
            if (child.getUpperConfidenceBound() >= bestScore)
            {
               bestScore = child.getUpperConfidenceBound();
               bestNode = (MonteCarloWaypointNode) child;
            }
         }
         updateTree(bestNode);
      }
   }

   public void updateWorld(Point2DReadOnly newState)
   {
      MonteCarloPlannerTools.updateGrid(world, newState, agent.getRangeScanner().getMaxRange());
   }

   public void updateAgent(Point2DReadOnly newState)
   {
      agent.changeStateTo(newState);
   }

   /**
    * Expands the given node by creating a child node for each available action.
    */
   public MonteCarloWaypointNode expand(MonteCarloWaypointNode node)
   {
      ArrayList<Vector2DReadOnly> availableActions = getAvailableActions(node.getPosition(), stepLength);

      for (Vector2DReadOnly action : availableActions)
      {
         Point2D newState = computeActionResult(node.getPosition(), action);
         MonteCarloWaypointNode postNode = new MonteCarloWaypointNode(newState, node, uniqueNodeId++);

         node.getChildren().add(postNode);
      }

      return (MonteCarloWaypointNode) node.getChildren().get((int) (Math.random() * node.getChildren().size()));
   }

   /**
    * Back propagates the given score to the root node.
    */
   public ArrayList<Vector2DReadOnly> getAvailableActions(Point2DReadOnly position, int stepLength)
   {
      ArrayList<Vector2DReadOnly> availableActions = new ArrayList<>();

      Vector2D[] actions = new Vector2D[] {new Vector2D(0, stepLength),
                                           new Vector2D(0, -stepLength),
                                           new Vector2D(stepLength, 0),
                                           new Vector2D(-stepLength, 0)};

      for (Vector2D action : actions)
      {
         if (checkActionObstacles(position, action, world))
         {
            if (checkActionBoundaries(position, action, world.getGridWidth()))
            {
               availableActions.add(action);
            }
         }
      }

      //LogTools.warn("Available Actions: {}", availableActions.size());

      return availableActions;
   }

   public double simulate(MonteCarloWaypointNode node)
   {
      double score = 0;

      Point2DReadOnly randomState = node.getPosition();

      for (int i = 0; i < simulationIterations; i++)
      {
         //LogTools.warn("Simulation: {}", i);

         ArrayList<Vector2DReadOnly> availableActions = getAvailableActions(randomState, stepLength);
         Vector2DReadOnly randomAction = availableActions.get((int) (Math.random() * availableActions.size()));
         randomState = computeActionResult(randomState, randomAction);
         score -= 1;

         if (!MonteCarloPlannerTools.isWithinGridBoundaries(randomState, world.getGridWidth()))
         {
            score -= MonteCarloPlannerConstants.PENALTY_COLLISION_BOUNDARY;
         }

         if (randomState.distanceSquared(world.getGoal()) < world.getGoalMarginSquared())
         {
            score += MonteCarloPlannerConstants.REWARD_GOAL;
         }

         if (world.getGrid().ptr((int) randomState.getX(), (int) randomState.getY()).get() == MonteCarloPlannerConstants.OCCUPIED)
         {
            score -= MonteCarloPlannerConstants.PENALTY_COLLISION_OBSTACLE;
         }

         ArrayList<Point2DReadOnly> scanPoints = agent.getRangeScanner().scan(randomState, world);

         score += agent.getAveragePosition().distanceSquared(randomState) * MonteCarloPlannerConstants.REWARD_DISTANCE_FROM_AVERAGE_POSITION;

         for (Point2DReadOnly point : scanPoints)
         {
            if (MonteCarloPlannerTools.isWithinGridBoundaries(point, world.getGridWidth()))
            {
               if (point.distanceSquared(randomState) < agent.getRangeScanner().getMaxRangeSquared())
               {
                  score -= MonteCarloPlannerConstants.PENALTY_PROXIMITY_OBSTACLE;
               }
               else
               {
                  score += MonteCarloPlannerConstants.REWARD_SAFE_DISTANCE;
               }

               if (world.getGrid().ptr((int) point.getX(), (int) point.getY()).get() == MonteCarloPlannerConstants.OCCUPANCY_UNKNOWN)
               {
                  score += MonteCarloPlannerConstants.REWARD_COVERAGE;
               }
            }
         }
      }

      return score;
   }

   public void backPropagate(MonteCarloWaypointNode node, float score)
   {
      node.addValue(score);
      node.incrementVisits();

      if (node.getParent() != null)
      {
         backPropagate((MonteCarloWaypointNode) node.getParent(), score);
      }
   }

   private static Point2D computeActionResult(Point2DReadOnly state, Vector2DReadOnly action)
   {
      Point2D actionResult = new Point2D();
      actionResult.add(state, action);
      return actionResult;
   }

   public boolean checkActionObstacles(Point2DReadOnly state, Vector2DReadOnly action, MonteCarloPlanningWorld world)
   {
      Point2D position = computeActionResult(state, action);
      return !MonteCarloPlannerTools.isPointOccupied(position, world.getGrid());
   }

   public boolean checkActionBoundaries(Point2DReadOnly state, Vector2DReadOnly action, int gridWidth)
   {
      Point2D position = new Point2D();
      position.add(state, action);
      return MonteCarloPlannerTools.isWithinGridBoundaries(position, gridWidth);
   }

   public void submitMeasurements(List<Point3DReadOnly> measurements)
   {
      ArrayList<Point2DReadOnly> points = new ArrayList<>();

      for (int i = 0; i<measurements.size(); i+=5)
      {
         Point3DReadOnly measurement = measurements.get(i);

         if (measurement.getZ() > MonteCarloPlannerConstants.OCCUPANCY_MIN_THRESHOLD_HEIGHT_IN_METERS)
         {
            points.add(new Point2D(measurement.getX(), measurement.getY()));
         }
      }

      agent.setMeasurements(points);
   }

   public void setParameters(int simulationIterations, int searchIterations, int stepLength)
   {
      this.simulationIterations = simulationIterations;
      this.searchIterations = searchIterations;
      this.stepLength = stepLength;
   }

   public void getOptimalPathFromRoot(List<MonteCarloWaypointNode> path)
   {
      MonteCarloPlannerTools.getOptimalPath(root, path);
   }

   public int getNumberOfNodesInTree()
   {
      return MonteCarloPlannerTools.getTotalNodesInSubTree(root);
   }

   public void printLayerCounts()
   {
      HashMap<Integer, Integer> layerCounts = new HashMap<>();
      MonteCarloPlannerTools.getLayerCounts(root, layerCounts);

      StringBuilder output = new StringBuilder("{");
      for (Integer key : layerCounts.keySet())
      {
         output.append("(").append(key - root.getLevel()).append(":").append(layerCounts.get(key)).append(")");
         output.append(", ");
      }

      LogTools.info("Layer Counts: {}", output.toString());
   }

   public MonteCarloWaypointAgent getAgent()
   {
      return agent;
   }

   public MonteCarloPlanningWorld getWorld()
   {
      return world;
   }

   public void scanWorld()
   {
      agent.measure(world);
   }
}
