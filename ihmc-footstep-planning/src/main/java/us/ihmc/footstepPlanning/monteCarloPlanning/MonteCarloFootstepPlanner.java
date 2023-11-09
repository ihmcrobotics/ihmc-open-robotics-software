package us.ihmc.footstepPlanning.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class MonteCarloFootstepPlanner
{
   private MonteCarloFootstepPlannerParameters plannerParameters;

   private MonteCarloPlanningWorld world;
   private MonteCarloWaypointAgent agent;
   private MonteCarloFootstepNode root;

   private MonteCarloFootstepPlannerRequest request;

   private int searchIterations = 15;
   private int simulationIterations = 100;
   private int uniqueNodeId = 0;
   private int stepLength = 5;
   private int worldHeight = 200;
   private int worldWidth = 200;
   private int goalMargin = 5;

   public MonteCarloFootstepPlanner(MonteCarloFootstepPlannerParameters plannerParameters)
   {
      this.plannerParameters = plannerParameters;
      this.world = new MonteCarloPlanningWorld(goalMargin, worldHeight, worldWidth);
      root = new MonteCarloFootstepNode(new Point3D(), null, uniqueNodeId++);
   }

   /**
    * Computes the next best action for the agent to take based on the current state of the world
    * and the agent. Updates the Monte Carlo Tree multiple times before selecting the best node
    * based on the Upper Confidence Bound.
    */
   public FootstepPlan generateFootstepPlan(MonteCarloFootstepPlannerRequest request)
   {
      this.request = request;

      // Update the tree multiple times
      for (int i = 0; i < searchIterations; i++)
      {
         updateTree(root);
      }

      FootstepPlan plan = MonteCarloPlannerTools.getFootstepPlanFromTree(root);
      return plan;
   }

   public MonteCarloFootstepNode getBestNode()
   {
      float bestScore = 0;
      MonteCarloFootstepNode bestNode = null;

      if (root.getChildren().isEmpty())
         LogTools.warn("No Children Nodes Found");

      // Select the best node based on the Upper Confidence Bound
      for (MonteCarloTreeNode node : root.getChildren())
      {
         node.updateUpperConfidenceBound();
         if (node.getUpperConfidenceBound() > bestScore)
         {
            bestScore = node.getUpperConfidenceBound();
            bestNode = (MonteCarloFootstepNode) node;
         }
      }

      root = bestNode;

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
   public void updateTree(MonteCarloFootstepNode node)
   {
      if (node == null)
         return;

      if (node.getVisits() == 0)
      {
         MonteCarloFootstepNode childNode = expand(node);
         double score = simulate(childNode);
         childNode.setValue((float) score);
         backPropagate(node, (float) score);
      }
      else
      {
         float bestScore = 0;
         MonteCarloFootstepNode bestNode = null;
         for (MonteCarloTreeNode child : node.getChildren())
         {
            child.updateUpperConfidenceBound();
            if (child.getUpperConfidenceBound() >= bestScore)
            {
               bestScore = child.getUpperConfidenceBound();
               bestNode = (MonteCarloFootstepNode) child;
            }
         }
         updateTree(bestNode);
      }
   }

   /**
    * Expands the given node by creating a child node for each available action.
    */
   public MonteCarloFootstepNode expand(MonteCarloFootstepNode node)
   {
      ArrayList<?> availableStates = node.getAvailableStates(world);

      for (Object newStateObj : availableStates)
      {
         MonteCarloFootstepNode newState = (MonteCarloFootstepNode) newStateObj;
         MonteCarloFootstepNode postNode = new MonteCarloFootstepNode(newState.getPosition(), node, uniqueNodeId++);

         node.getChildren().add(postNode);
      }

      return (MonteCarloFootstepNode) node.getChildren().get((int) (Math.random() * node.getChildren().size()));
   }

   public double simulate(MonteCarloFootstepNode node)
   {
      double score = 0;

      MonteCarloFootstepNode randomState = new MonteCarloFootstepNode(node.getPosition(), null, 0);

      for (int i = 0; i < simulationIterations; i++)
      {
         //LogTools.warn("Simulation: {}", i);

         ArrayList<MonteCarloFootstepNode> availableActions = randomState.getAvailableStates(world);
         randomState = availableActions.get((int) (Math.random() * availableActions.size()));
         score -= 1;

         //if (!MonteCarloPlannerTools.isWithinGridBoundaries(randomState.getPosition(), world.getGridWidth()))
         //{
         //   score -= MonteCarloPlannerConstants.PENALTY_COLLISION_BOUNDARY;
         //}

         if (randomState.getPosition().distanceSquared(request.getGoalFootPoses().get(RobotSide.LEFT).getPosition()) < world.getGoalMarginSquared())
         {
            score += MonteCarloPlannerConstants.REWARD_GOAL;
         }

         if (world.getGrid().ptr((int) randomState.getPosition().getX(), (int) randomState.getPosition().getY()).get() == MonteCarloPlannerConstants.OCCUPIED)
         {
            score -= MonteCarloPlannerConstants.PENALTY_COLLISION_OBSTACLE;
         }
      }

      return score;
   }

   public void backPropagate(MonteCarloFootstepNode node, float score)
   {
      node.addValue(score);
      node.incrementVisits();

      if (node.getParent() != null)
      {
         backPropagate((MonteCarloFootstepNode) node.getParent(), score);
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
      MonteCarloPlannerTools.updateGrid(world, (Point2DReadOnly) newState.getPosition(), agent.getRangeScanner().getMaxRange());
   }

   public void updateAgent(MonteCarloTreeNode newState)
   {
      agent.changeStateTo((Point2DReadOnly) newState.getPosition());
   }

   public void setParameters(int simulationIterations, int searchIterations, int stepLength)
   {
      this.simulationIterations = simulationIterations;
      this.searchIterations = searchIterations;
      this.stepLength = stepLength;
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
