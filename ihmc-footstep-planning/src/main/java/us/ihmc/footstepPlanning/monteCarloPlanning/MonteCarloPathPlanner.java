package us.ihmc.footstepPlanning.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
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
public class MonteCarloPathPlanner
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

   public MonteCarloPathPlanner(int offset)
   {
      this();
      agentPos.set(offset, offset);
   }

   public MonteCarloPathPlanner()
   {
      this.world = new MonteCarloPlanningWorld(goalMargin, worldHeight, worldWidth);
      this.agent = new MonteCarloWaypointAgent(agentPos);
      root = new MonteCarloWaypointNode(agent.getState(), null, uniqueNodeId++);
   }

   /**
    * Computes the next best action for the agent to take based on the current state of the world
    * and the agent. Updates the Monte Carlo Tree multiple times before selecting the best node
    * based on the Upper Confidence Bound.
    */
   public MonteCarloTreeNode plan()
   {
      if (root == null)
         return new MonteCarloWaypointNode(agent.getState(), null, 0);

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
         if (node.getValue() > bestScore)
         {
            bestScore = node.getValue();
            bestNode = (MonteCarloWaypointNode) node;
         }
      }

      root = bestNode;

      if (bestNode == null)
         return new MonteCarloWaypointNode(agent.getState(), null, 0);

      return bestNode;
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
            child.updateUpperConfidenceBound(2.0f);
            if (child.getUpperConfidenceBound() >= bestScore)
            {
               bestScore = child.getUpperConfidenceBound();
               bestNode = (MonteCarloWaypointNode) child;
            }
         }
         updateTree(bestNode);
      }
   }

   /**
    * Expands the given node by creating a child node for each available action.
    */
   public MonteCarloWaypointNode expand(MonteCarloWaypointNode node)
   {
      ArrayList<?> availableStates = node.getAvailableStates(world);

      for (Object newStateObj : availableStates)
      {
         MonteCarloWaypointNode newState = (MonteCarloWaypointNode) newStateObj;
         MonteCarloWaypointNode postNode = new MonteCarloWaypointNode(newState.getState(), node, uniqueNodeId++);

         node.addChild(postNode);
      }

      return (MonteCarloWaypointNode) node.getChildren().get((int) (Math.random() * node.getChildren().size()));
   }

   public double simulate(MonteCarloWaypointNode node)
   {
      double score = 0;

      MonteCarloWaypointNode randomState = new MonteCarloWaypointNode(node.getState(), null, 0);

      for (int i = 0; i < simulationIterations; i++)
      {
         //LogTools.warn("Simulation: {}", i);

         ArrayList<MonteCarloWaypointNode> availableActions = randomState.getAvailableStates(world);
         randomState = availableActions.get((int) (Math.random() * availableActions.size()));
         score -= 1;

         if (!MonteCarloPlannerTools.isWithinGridBoundaries(randomState.getState(), world.getGridWidth()))
         {
            score -= MonteCarloPlannerConstants.PENALTY_COLLISION_BOUNDARY;
         }

         if (randomState.getState().distanceSquared(world.getGoal()) < world.getGoalMarginSquared())
         {
            score += MonteCarloPlannerConstants.REWARD_GOAL;
         }

         if (world.getGrid().ptr((int) randomState.getState().getX(), (int) randomState.getState().getY()).get() == MonteCarloPlannerConstants.OCCUPIED)
         {
            score -= MonteCarloPlannerConstants.PENALTY_COLLISION_OBSTACLE;
         }

         ArrayList<Point2DReadOnly> scanPoints = agent.getRangeScanner().scan(randomState.getState(), world);

         score += agent.getAveragePosition().distanceSquared(randomState.getState()) * MonteCarloPlannerConstants.REWARD_DISTANCE_FROM_AVERAGE_POSITION;

         for (Point2DReadOnly point : scanPoints)
         {
            if (MonteCarloPlannerTools.isWithinGridBoundaries(point, world.getGridWidth()))
            {
               if (point.distanceSquared(randomState.getState()) < agent.getRangeScanner().getMaxRangeSquared())
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

      if (!node.getParents().isEmpty())
      {
         for (MonteCarloTreeNode parent : node.getParents())
         {
            backPropagate((MonteCarloWaypointNode) parent, score);
         }
      }
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

   public void updateState(MonteCarloTreeNode newState)
   {
      updateWorld(newState);
      updateAgent(newState);
   }

   public void updateWorld(MonteCarloTreeNode newState)
   {
      MonteCarloPlannerTools.updateGrid(world, (Point2DReadOnly) newState.getState(), agent.getRangeScanner().getMaxRange());
   }

   public void updateAgent(MonteCarloTreeNode newState)
   {
      agent.changeStateTo((Point2DReadOnly) newState.getState());
   }

   public void setParameters(int simulationIterations, int searchIterations, int stepLength)
   {
      this.simulationIterations = simulationIterations;
      this.searchIterations = searchIterations;
      this.stepLength = stepLength;
   }

   public void getOptimalPathFromRoot(List<MonteCarloTreeNode> path)
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

   public MonteCarloTreeNode getRoot()
   {
      return root;
   }
}
