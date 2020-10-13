package us.ihmc.footstepPlanning.graphSearch;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiPredicate;
import java.util.function.ToDoubleBiFunction;

public class AStarFootstepPlannerIterationConductorTest
{
   @Test
   public void testSimple2DGridSearch()
   {
      ManhattanDistanceCalculator distanceCalculator = new ManhattanDistanceCalculator();
      AStarFootstepPlannerIterationConductor planner = new AStarFootstepPlannerIterationConductor(this::getNeighbors, (n1, n2) -> true, (n1, n2) -> 1.0, distanceCalculator::getManhattanDistance);

      FootstepNode startNode = new FootstepNode(0, 0, 0, RobotSide.LEFT);
      FootstepNode goalNode = new FootstepNode(5, 0, 0, RobotSide.RIGHT);
      distanceCalculator.setGoalNode(goalNode);
      planner.initialize(startNode);

      for (int i = 0; i < 5; i++)
      {
         AStarIterationData<FootstepNode> iterationData = planner.doPlanningIteration(planner.getNextNode(), true);
         Assertions.assertEquals(iterationData.getParentNode().getXIndex(), i);
         Assertions.assertEquals(iterationData.getParentNode().getYIndex(), 0);
         Assertions.assertEquals(iterationData.getValidChildNodes().size(), 4);
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
      BiPredicate<FootstepNode, FootstepNode> edgeChecker = (child, parent) -> !child.getLatticeNode().equals(new LatticeNode(0, 4, 0));
      ToDoubleBiFunction<FootstepNode, FootstepNode> edgeCost = (n1, n2) -> 1.0 + (n2.getXIndex() < 0 ? 2.0 : 0.0);
      AStarFootstepPlannerIterationConductor planner = new AStarFootstepPlannerIterationConductor(this::getNeighbors, edgeChecker, edgeCost, distanceCalculator::getManhattanDistance);

      FootstepNode startNode = new FootstepNode(0, 0, 0, RobotSide.LEFT);
      FootstepNode goalNode = new FootstepNode(0, 5, 0, RobotSide.RIGHT);
      distanceCalculator.setGoalNode(goalNode);
      planner.initialize(startNode);

      planningLoop:
      while (true)
      {
         AStarIterationData<FootstepNode> iterationData = planner.doPlanningIteration(planner.getNextNode(), true);
         List<FootstepNode> childNodes = iterationData.getValidChildNodes();
         for (int i = 0; i < childNodes.size(); i++)
         {
            if(childNodes.get(i).equals(goalNode))
            {
               break planningLoop;
            }
         }
      }

      Assertions.assertTrue(planner.getGraph().doesNodeExist(goalNode));
      List<FootstepNode> path = planner.getGraph().getPathFromStart(goalNode);

      Assertions.assertTrue(path.size() == 8);
      Assertions.assertTrue(path.get(0).equals(new FootstepNode(0, 0, 0, RobotSide.LEFT)));
      Assertions.assertTrue(path.get(1).equals(new FootstepNode(0, 1, 0, RobotSide.RIGHT)));
      Assertions.assertTrue(path.get(2).equals(new FootstepNode(0, 2, 0, RobotSide.LEFT)));
      Assertions.assertTrue(path.get(3).equals(new FootstepNode(0, 3, 0, RobotSide.RIGHT)));
      Assertions.assertTrue(path.get(4).equals(new FootstepNode(1, 3, 0, RobotSide.LEFT)));
      Assertions.assertTrue(path.get(5).equals(new FootstepNode(1, 4, 0, RobotSide.RIGHT)));
      Assertions.assertTrue(path.get(6).equals(new FootstepNode(1, 5, 0, RobotSide.LEFT)));
      Assertions.assertTrue(path.get(7).equals(new FootstepNode(0, 5, 0, RobotSide.RIGHT)));
   }

   private class ManhattanDistanceCalculator
   {
      private FootstepNode goalNode;

      public void setGoalNode(FootstepNode goalNode)
      {
         this.goalNode = goalNode;
      }

      double getManhattanDistance(FootstepNode other)
      {
         return Math.abs(other.getXIndex() - goalNode.getXIndex()) + Math.abs(other.getYIndex() - goalNode.getYIndex());
      }
   }

   private void getNeighbors(FootstepNode node, List<FootstepNode> expansionToPack)
   {
      expansionToPack.clear();
      expansionToPack.add(new FootstepNode(node.getXIndex() - 1, node.getYIndex(), node.getYawIndex(), node.getRobotSide().getOppositeSide()));
      expansionToPack.add(new FootstepNode(node.getXIndex() + 1, node.getYIndex(), node.getYawIndex(), node.getRobotSide().getOppositeSide()));
      expansionToPack.add(new FootstepNode(node.getXIndex(), node.getYIndex() - 1, node.getYawIndex(), node.getRobotSide().getOppositeSide()));
      expansionToPack.add(new FootstepNode(node.getXIndex(), node.getYIndex() + 1, node.getYawIndex(), node.getRobotSide().getOppositeSide()));
   }
}

