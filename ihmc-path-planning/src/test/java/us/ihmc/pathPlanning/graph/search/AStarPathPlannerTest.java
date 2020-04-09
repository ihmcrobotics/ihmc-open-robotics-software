package us.ihmc.pathPlanning.graph.search;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.pathPlanning.graph.GridNode;

import java.util.ArrayList;
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
         Assertions.assertTrue(iterationData.getParentNode().getX() == i);
         Assertions.assertTrue(iterationData.getParentNode().getY() == 0);
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
      ToDoubleBiFunction<GridNode, GridNode> edgeCost = (n1, n2) -> 1.0 + (n2.getX() < 0 ? 2.0 : 0.0);
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

   private class ManhattanDistanceCalculator
   {
      private GridNode goalNode;

      public void setGoalNode(GridNode goalNode)
      {
         this.goalNode = goalNode;
      }

      double getManhattanDistance(GridNode other)
      {
         return (Math.abs(other.getX() - goalNode.getX()) + Math.abs(other.getY() - goalNode.getY()));
      }
   }

   private List<GridNode> getNeighbors(GridNode gridNode)
   {
      List<GridNode> neighbors = new ArrayList<>();
      neighbors.add(new GridNode(gridNode.getX() - 1, gridNode.getY()));
      neighbors.add(new GridNode(gridNode.getX() + 1, gridNode.getY()));
      neighbors.add(new GridNode(gridNode.getX(), gridNode.getY()- 1));
      neighbors.add(new GridNode(gridNode.getX(), gridNode.getY()+ 1));
      return neighbors;
   }
}

