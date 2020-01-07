package us.ihmc.pathPlanning.graph.search;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import java.util.HashSet;
import java.util.List;
import java.util.function.BiPredicate;
import java.util.function.ToDoubleBiFunction;

public class AStarPathPlannerTest
{
   @Test
   public void testSimple2DGridSearch()
   {
      ManhattanDistanceCalculator distanceCalculator = new ManhattanDistanceCalculator();
      AStarPathPlanner<GridNode> planner = new AStarPathPlanner<>(this::getNeighbors, (n1, n2) -> true, (n1, n2) -> 1.0, distanceCalculator::getManhattanDistance);

      GridNode startNode = new GridNode(0, 0);
      GridNode goalNode = new GridNode(5, 0);
      distanceCalculator.setGoalNode(goalNode);
      planner.initialize(startNode);

      for (int i = 0; i < 5; i++)
      {
         AStarIterationData<GridNode> iterationData = planner.doPlanningIteration();
         Assertions.assertTrue(iterationData.getParentNode().x == i);
         Assertions.assertTrue(iterationData.getParentNode().y == 0);
         Assertions.assertTrue(iterationData.getValidChildNodes().size() == 4);
         Assertions.assertTrue(iterationData.getInvalidChildNodes().isEmpty());

         if(i == 4)
         {
            boolean foundGoalNode = false;
            for (int j = 0; j < iterationData.getValidChildNodes().size(); j++)
            {
               if(iterationData.getValidChildNodes().get(j).equals(goalNode))
                  foundGoalNode = true;
            }

            Assertions.assertTrue(foundGoalNode);
         }
      }
   }

   @Test
   public void test2DSearchWithObstacle()
   {
      ManhattanDistanceCalculator distanceCalculator = new ManhattanDistanceCalculator();
      BiPredicate<GridNode, GridNode> edgeChecker = (child, parent) -> !child.equals(new GridNode(0, 4));
      ToDoubleBiFunction<GridNode, GridNode> edgeCost = (n1, n2) -> 1.0 + (n2.x < 0 ? 2.0 : 0.0);
      AStarPathPlanner<GridNode> planner = new AStarPathPlanner<>(this::getNeighbors, edgeChecker, edgeCost, distanceCalculator::getManhattanDistance);

      GridNode startNode = new GridNode(0, 0);
      GridNode goalNode = new GridNode(0, 5);
      distanceCalculator.setGoalNode(goalNode);
      planner.initialize(startNode);

      planningLoop:
      while(true)
      {
         AStarIterationData<GridNode> iterationData = planner.doPlanningIteration();
         List<GridNode> childNodes = iterationData.getValidChildNodes();
         for (int i = 0; i < childNodes.size(); i++)
         {
            if(childNodes.get(i).equals(goalNode))
            {
               break planningLoop;
            }
         }
      }

      Assertions.assertTrue(planner.getGraph().doesNodeExist(goalNode));
      List<GridNode> path = planner.getGraph().getPathFromStart(goalNode);

      Assertions.assertTrue(path.size() == 8);
      Assertions.assertTrue(path.get(0).equals(new GridNode(0, 0)));
      Assertions.assertTrue(path.get(1).equals(new GridNode(0, 1)));
      Assertions.assertTrue(path.get(2).equals(new GridNode(0, 2)));
      Assertions.assertTrue(path.get(3).equals(new GridNode(0, 3)));
      Assertions.assertTrue(path.get(4).equals(new GridNode(1, 3)));
      Assertions.assertTrue(path.get(5).equals(new GridNode(1, 4)));
      Assertions.assertTrue(path.get(6).equals(new GridNode(1, 5)));
      Assertions.assertTrue(path.get(7).equals(new GridNode(0, 5)));
   }

   private class GridNode
   {
      final int x, y;

      public GridNode(int x, int y)
      {
         this.x = x;
         this.y = y;
      }

      @Override
      public int hashCode()
      {
         return 5 * x + 13 * y;
      }

      @Override
      public boolean equals(Object o)
      {
         GridNode gridNode = (GridNode) o;
         return (gridNode.x == x) && (gridNode.y == y);
      }
   }

   private class ManhattanDistanceCalculator
   {
      private GridNode goalNode;

      public void setGoalNode(GridNode goalNode)
      {
         this.goalNode = goalNode;
      }

      double getManhattanDistance(GridNode other)
      {
         return (double) (Math.abs(other.x - goalNode.x) + Math.abs(other.y - goalNode.y));
      }
   }

   private HashSet<GridNode> getNeighbors(GridNode gridNode)
   {
      HashSet<GridNode> neighbors = new HashSet<>();
      neighbors.add(new GridNode(gridNode.x - 1, gridNode.y));
      neighbors.add(new GridNode(gridNode.x + 1, gridNode.y));
      neighbors.add(new GridNode(gridNode.x, gridNode.y- 1));
      neighbors.add(new GridNode(gridNode.x, gridNode.y+ 1));
      return neighbors;
   }
}

