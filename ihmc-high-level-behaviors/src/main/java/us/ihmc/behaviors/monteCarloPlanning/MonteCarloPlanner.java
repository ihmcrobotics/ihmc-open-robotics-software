package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;

import java.util.ArrayList;

public class MonteCarloPlanner
{
   private int searchIterations = 10;
   private int simulationIterations = 10;
   private int uniqueNodeId = 0;

   private World world;
   private Agent agent;
   private MonteCarloTreeNode root;

   int worldHeight = 200;
   int worldWidth = 200;
   int goalMargin = 5;

   int stepLength = 8;

   private final Point2D agentPos = new Point2D(0, 0);

   public MonteCarloPlanner(int offset)
   {
      agentPos.set(offset, offset);

      this.world = new World(goalMargin, worldHeight, worldWidth);
      this.agent = new Agent(agentPos);

      root = new MonteCarloTreeNode(agent.getPosition(), null, uniqueNodeId++);
   }

   /**
    * Computes the next best action for the agent to take based on the current state of the world
    * and the agent. Updates the Monte Carlo Tree multiple times before selecting the best node
    * based on the Upper Confidence Bound.
    */
   public Point2D plan()
   {
      if (root == null)
         return agent.getPosition();

      // Update the tree multiple times
      for (int i = 0; i < searchIterations; i++)
      {
         updateTree(root);
      }

      float bestScore = 0;
      MonteCarloTreeNode bestNode = null;

      if (root.getChildren().size() == 0)
         LogTools.warn("No Children Nodes Found");

      // Select the best node based on the Upper Confidence Bound
      for (MonteCarloTreeNode node : root.getChildren())
      {
         node.updateUpperConfidenceBound();
         if (node.getUpperConfidenceBound() > bestScore)
         {
            bestScore = node.getUpperConfidenceBound();
            bestNode = node;
         }
      }

      root = bestNode;

      LogTools.info("Total Nodes in Tree: " + MonteCarloPlannerTools.getTotalNodes(root));

      if (bestNode == null)
         return agent.getPosition();

      return bestNode.getAgentState().getPosition();
   }

   public void execute(Point2D newState)
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
   public void updateTree(MonteCarloTreeNode node)
   {
      if (node == null)
         return;

      if (node.getVisits() == 0)
      {
         MonteCarloTreeNode childNode = expand(node);
         float score = simulate(childNode);
         childNode.setValue(score);
         backPropagate(node, score);
      }
      else
      {
         float bestScore = 0;
         MonteCarloTreeNode bestNode = null;
         for (MonteCarloTreeNode child : node.getChildren())
         {
            child.updateUpperConfidenceBound();
            if (child.getUpperConfidenceBound() >= bestScore)
            {
               bestScore = child.getUpperConfidenceBound();
               bestNode = child;
            }
         }
         updateTree(bestNode);
      }
   }

   public void updateWorld(Point2D newState)
   {
      MonteCarloPlannerTools.updateGrid(world, newState, agent.getRangeScanner().getMaxRange());
   }

   public void updateAgent(Point2D newState)
   {
      agent.updateState(newState);
   }

   /**
    * Expands the given node by creating a child node for each available action.
    */
   public MonteCarloTreeNode expand(MonteCarloTreeNode node)
   {
      ArrayList<Vector2D> availableActions = getAvailableActions(node, stepLength);

      for (Vector2D action : availableActions)
      {
         Point2D newState = computeActionResult(node.getAgentState().getPosition(), action);
         MonteCarloTreeNode postNode = new MonteCarloTreeNode(newState, node, uniqueNodeId++);

         node.getChildren().add(postNode);
      }

      MonteCarloTreeNode newNode = node.getChildren().get((int) (Math.random() * node.getChildren().size()));

      return newNode;
   }

   /**
    * Back propagates the given score to the root node.
    */
   public ArrayList<Vector2D> getAvailableActions(MonteCarloTreeNode node, int stepLength)
   {
      ArrayList<Vector2D> availableActions = new ArrayList<>();

      Vector2D[] actions = new Vector2D[] {new Vector2D(0, stepLength),
                                           new Vector2D(0, -stepLength),
                                           new Vector2D(stepLength, 0),
                                           new Vector2D(-stepLength, 0)};

      for (Vector2D action : actions)
      {
         if (checkActionObstacles(node.getAgentState().getPosition(), action, world))
         {
            if (checkActionBoundaries(node.getAgentState().getPosition(), action, world.getGridWidth()))
            {
               availableActions.add(action);
            }
         }
      }

      return availableActions;
   }

   public float simulate(MonteCarloTreeNode node)
   {
      float score = 0;

      Point2D randomState = node.getAgentState().getPosition();

      for (int i = 0; i < simulationIterations; i++)
      {
         ArrayList<Vector2D> availableActions = getAvailableActions(node, stepLength);
         Vector2D randomAction = availableActions.get((int) (Math.random() * availableActions.size()));
         randomState = computeActionResult(randomState, randomAction);
         score -= 1;

         if (randomState.getX() < 0 || randomState.getX() >= world.getGridWidth() || randomState.getY() < 0 || randomState.getY() >= world.getGridHeight())
         {
            score -= 1000;
         }

         if (randomState.distance(world.getGoal()) < world.getGoalMargin())
         {
            score += 200000;
         }

         if (world.getGrid().ptr((int) randomState.getX(), (int) randomState.getY()).get() == 100)
         {
            score -= 400;
         }

         ArrayList<Point2D> scanPoints = agent.getRangeScanner().scan(randomState, world);

         score += Math.pow(agent.getAveragePosition().distance(randomState), 2);

         for (Point2D point : scanPoints)
         {
            if (point.getX() >= 0 && point.getX() < world.getGridWidth() && point.getY() >= 0 && point.getY() < world.getGridHeight())
            {
               if (point.distance(randomState) < agent.getRangeScanner().getMaxRange())
               {
                  score -= 40;
               }
               else
               {
                  score += 20;
               }

               if (world.getGrid().ptr((int) point.getX(), (int) point.getY()).get() == 0)
               {
                  score += 500;
               }
            }
         }
      }

      return score;
   }

   public void backPropagate(MonteCarloTreeNode node, float score)
   {
      node.setValue(node.getValue() + score);
      node.setVisits(node.getVisits() + 1);

      if (node.getParent() != null)
      {
         backPropagate(node.getParent(), score);
      }
   }

   public Point2D computeActionResult(Point2D state, Vector2D action)
   {
      return new Point2D(state.getX() + action.getX(), state.getY() + action.getY());
   }

   public boolean checkActionObstacles(Point2D state, Vector2D action, World world)
   {
      Point2D position = new Point2D(state.getX() + action.getX(), state.getY() + action.getY());

      return !MonteCarloPlannerTools.isPointOccupied(position, world.getGrid());
   }

   public boolean checkActionBoundaries(Point2D state, Vector2D action, int gridWidth)
   {
      Point2D position = new Point2D(state.getX() + action.getX(), state.getY() + action.getY());

      return position.getX() >= 0 && position.getX() < gridWidth && position.getY() >= 0 && position.getY() < gridWidth;
   }

   public void addMeasurements(ArrayList<Point3D> measurements)
   {
      ArrayList<Point2D> points = new ArrayList<>();

      for (int i = 0; i<measurements.size(); i+=5)
      {
         Point3D measurement = measurements.get(i);

         if (measurement.getZ() > 0.5)
         {
            points.add(new Point2D(measurement.getX(), measurement.getY()));
         }
      }

      agent.addMeasurements(points);
   }

   public Agent getAgent()
   {
      return agent;
   }

   public World getWorld()
   {
      return world;
   }

   public void scanWorld()
   {
      agent.measure(world);
   }
}
