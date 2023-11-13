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
import java.util.HashSet;
import java.util.List;

public class MonteCarloFootstepPlanner
{
   private MonteCarloFootstepPlannerParameters plannerParameters;

   private MonteCarloPlanningWorld world;
   private MonteCarloWaypointAgent agent;
   private MonteCarloFootstepNode root;

   private HashMap<MonteCarloFootstepNode, MonteCarloFootstepNode> visitedNodes = new HashMap<>();

   private int searchIterations = 200;
   private int simulationIterations = 8;
   private int uniqueNodeId = 0;
   private int worldHeight = 200;
   private int worldWidth = 200;
   private int goalMargin = 5;

   public MonteCarloFootstepPlanner(MonteCarloFootstepPlannerParameters plannerParameters)
   {
      this.plannerParameters = plannerParameters;
      this.world = new MonteCarloPlanningWorld(goalMargin, worldHeight, worldWidth);
      root = new MonteCarloFootstepNode(new Point3D(), null, RobotSide.LEFT, uniqueNodeId++);
   }

   /**
    * Computes the next best action for the agent to take based on the current state of the world
    * and the agent. Updates the Monte Carlo Tree multiple times before selecting the best node
    * based on the Upper Confidence Bound.
    */
   public FootstepPlan generateFootstepPlan(MonteCarloFootstepPlannerRequest request)
   {
      // Update the tree multiple times
      for (int i = 0; i < searchIterations; i++)
      {
         updateTree(root, request);
      }

      MonteCarloPlannerTools.printLayerCounts(root);

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
   public void updateTree(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      if (node == null)
         return;

      if (node.getVisits() == 0)
      {
         MonteCarloFootstepNode childNode = expand(node, request);
         double score = simulate(childNode, request);
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
         updateTree(bestNode, request);
      }
   }

   /**
    * Expands the given node by creating a child node for each available action.
    */
   public MonteCarloFootstepNode expand(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      ArrayList<?> availableStates = node.getAvailableStates(world, request);

      for (Object newStateObj : availableStates)
      {
         MonteCarloFootstepNode newState = (MonteCarloFootstepNode) newStateObj;

         if (visitedNodes.getOrDefault(newState, null) != null)
         {
            MonteCarloFootstepNode existingNode = visitedNodes.get(newState);
            node.getChildren().add(existingNode);
         }
         else
         {
            MonteCarloFootstepNode postNode = new MonteCarloFootstepNode(newState.getPosition(), node, newState.getRobotSide(), uniqueNodeId++);
            visitedNodes.put(newState, postNode);
            node.getChildren().add(postNode);
         }
      }

      return (MonteCarloFootstepNode) node.getChildren().get((int) (Math.random() * node.getChildren().size()));
   }

   public double simulate(MonteCarloFootstepNode node, MonteCarloFootstepPlannerRequest request)
   {
      double score = 0;

      MonteCarloFootstepNode randomState = new MonteCarloFootstepNode(node.getPosition(), null, node.getRobotSide().getOppositeSide(), 0);

      for (int i = 0; i < simulationIterations; i++)
      {
         ArrayList<MonteCarloFootstepNode> nextStates = randomState.getAvailableStates(world, request);
         int actionIndex = (int) (Math.random() * nextStates.size());
         randomState = nextStates.get(actionIndex);

         //LogTools.info(String.format("Simulation %d for Node %d, Position: %s, Random State: %s, Actions: %d", i, node.getId(), node.getPosition(), randomState.getPosition(), nextStates.size()));

         //score -= 0.1;
         //if (randomState.getPosition().distanceSquared(request.getGoalFootPoses().get(RobotSide.LEFT).getPosition()) < world.getGoalMarginSquared())
         {
            score += 1.0f / randomState.getPosition().distance(request.getGoalFootPoses().get(RobotSide.LEFT).getPosition());
         }

         //LogTools.info("Action Taken: {}, Score: {}", actionIndex, score);
      }

      return score;
   }

   public void backPropagate(MonteCarloFootstepNode node, float score)
   {
      node.addValue(score);
      node.incrementVisits();

      if (!node.getParents().isEmpty())
      {
         for (MonteCarloTreeNode parent : node.getParents())
         {
            backPropagate((MonteCarloFootstepNode) parent, score);
         }
      }
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

   public MonteCarloPlanningWorld getWorld()
   {
      return world;
   }

   public MonteCarloTreeNode getRoot()
   {
      return root;
   }
}
