package us.ihmc.behaviors.monteCarloPlanning;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Vector4D32;
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

   int worldHeight = 100;
   int worldWidth = 100;
   int goalMargin = 5;

   private final Point2D agentPos = new Point2D(10, 10);
   private final Point2D goal = new Point2D(10, worldHeight - 10);

   public MonteCarloPlanner()
   {
      this.world = new World(new ArrayList<>(), goal, goalMargin, worldHeight, worldWidth);
      this.agent = new Agent(agentPos);

      root = new MonteCarloTreeNode(agent.getPosition(), null, uniqueNodeId++);
   }

   public Point2D plan()
   {
      if (root == null)
         return agent.getPosition();

      for (int i = 0; i < searchIterations; i++)
      {
         updateTree(root);
      }

      float bestScore = 0;
      MonteCarloTreeNode bestNode = null;

      if (root.getChildren().size() == 0)
         LogTools.warn("No Children Nodes Found");

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

      updateWorld(bestNode.getAgentState().getPosition());
      updateAgent(bestNode.getAgentState().getPosition());

      return bestNode.getAgentState().getPosition();
   }

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
      world.updateGrid(newState, agent.getRangeScanner().getMaxRange());
   }

   public void updateAgent(Point2D newState)
   {
      agent.updateState(newState);
   }

   public MonteCarloTreeNode expand(MonteCarloTreeNode node)
   {
      ArrayList<Vector2D> availableActions = getAvailableActions(node);

      for (Vector2D action : availableActions)
      {
         Point2D newState = computeActionResult(node.getAgentState().getPosition(), action);
         MonteCarloTreeNode postNode = new MonteCarloTreeNode(newState, node, uniqueNodeId++);

         node.getChildren().add(postNode);
      }

      MonteCarloTreeNode newNode = node.getChildren().get((int) (Math.random() * node.getChildren().size()));

      return newNode;
   }

   public ArrayList<Vector2D> getAvailableActions(MonteCarloTreeNode node)
   {
      ArrayList<Vector2D> availableActions = new ArrayList<>();

      for (Vector2D action : new Vector2D[] {new Vector2D(0, 1), new Vector2D(0, -1), new Vector2D(1, 0), new Vector2D(-1, 0)})
      {
         if (checkActionObstacles(node.getAgentState().getPosition(), action, world.getObstacles()))
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
         ArrayList<Vector2D> availableActions = getAvailableActions(node);
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

         if (world.getGrid().ptr((int) randomState.getX(), (int) randomState.getY()).getInt() == 100)
         {
            score -= 400;
         }

         ArrayList<Point2D> scanPoints = agent.getRangeScanner().scan(randomState, world.getObstacles());

         score += Math.pow(agent.getAveragePosition().distance(randomState), 2);

         for (Point2D point : scanPoints)
         {
            if (point.getX() >= 0 && point.getX() < world.getGridWidth() && point.getY() >= 0 && point.getY() < world.getGridHeight())
            {
               if (point.distance(randomState) < agent.getRangeScanner().getMaxRange())
               {
                  score -= 20;
               }
               else
               {
                  score += 20;
               }

               if (world.getGrid().ptr((int) point.getX(), (int) point.getY()).getInt() == 0)
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

   public boolean checkActionObstacles(Point2D state, Vector2D action, ArrayList<Vector4D32> obstacles)
   {
      Point2D position = new Point2D(state.getX() + action.getX(), state.getY() + action.getY());

      boolean collision = false;
      for (Vector4D32 obstacle : obstacles)
      {
         float obstacleSizeX = obstacle.getZ32();
         float obstacleSizeY = obstacle.getS32();
         float obstacleMinX = obstacle.getX32() - obstacleSizeX;
         float obstacleMaxX = obstacle.getX32() + obstacleSizeX;
         float obstacleMinY = obstacle.getY32() - obstacleSizeY;
         float obstacleMaxY = obstacle.getY32() + obstacleSizeY;

         if (position.getX() > obstacleMinX && position.getX() < obstacleMaxX)
         {
            if (position.getY() > obstacleMinY && position.getY() < obstacleMaxY)
            {
               collision = true;
               break;
            }
         }
      }

      return !collision;
   }

   public boolean checkActionBoundaries(Point2D state, Vector2D action, int gridWidth)
   {
      Point2D position = new Point2D(state.getX() + action.getX(), state.getY() + action.getY());

      return position.getX() >= 0 && position.getX() < gridWidth && position.getY() >= 0 && position.getY() < gridWidth;
   }

   public void addObstacles(ArrayList<Vector4D32> obstacles)
   {
      this.world.getObstacles().addAll(obstacles);
   }

   public void collectObservations()
   {
      agent.measure(world.getObstacles());
   }

   public void addMeasurements(ArrayList<Point3D> measurements)
   {
      ArrayList<Point2D> points = new ArrayList<>();

      for (int i = 0; i<measurements.size(); i+=2)
      {
         Point3D measurement = measurements.get(i);
         points.add(new Point2D(measurement.getX(), measurement.getY()));
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


}
