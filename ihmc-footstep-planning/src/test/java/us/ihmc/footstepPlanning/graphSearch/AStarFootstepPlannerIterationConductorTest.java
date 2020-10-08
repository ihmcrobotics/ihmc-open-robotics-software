package us.ihmc.footstepPlanning.graphSearch;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

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

      FootstepNode leftStartStep = new FootstepNode(0, 1, 0, RobotSide.LEFT);
      FootstepNode rightStartStep = new FootstepNode(0, -1, 0, RobotSide.RIGHT);
      FootstanceNode startNode = new FootstanceNode(leftStartStep, rightStartStep);

      FootstepNode leftGoalStep = new FootstepNode(3, 1, 0, RobotSide.LEFT);
      FootstepNode rightGoalStep = new FootstepNode(3, -1, 0, RobotSide.RIGHT);
      FootstanceNode goalNode = new FootstanceNode(leftGoalStep, rightGoalStep);

      distanceCalculator.setGoalNode(new SideDependentList<>(leftGoalStep, rightGoalStep));
      planner.initialize(startNode);

      for (int i = 0; i < 4; i++)
      {
         AStarIterationData<FootstanceNode> iterationData = planner.doPlanningIteration(planner.getNextNode(), true);
         Assertions.assertEquals(iterationData.getParentNode().getStanceNode().getXIndex(), i);
         Assertions.assertEquals(iterationData.getParentNode().getStanceNode().getYIndex(), iterationData.getParentNode().getStanceSide() == RobotSide.LEFT ? 1 : -1);
         Assertions.assertEquals(iterationData.getValidChildNodes().size(), 3);
         Assertions.assertTrue(iterationData.getInvalidChildNodes().isEmpty());

         if(i == 3)
         {
            boolean foundGoalNode = false;
            for (int j = 0; j < iterationData.getValidChildNodes().size(); j++)
            {
               if (iterationData.getValidChildNodes().get(j).equals(goalNode))
                  foundGoalNode = true;
            }

            Assertions.assertTrue(foundGoalNode);
         }
      }
   }

   private class ManhattanDistanceCalculator
   {
      private SideDependentList<FootstepNode> goalNodes;

      public void setGoalNode(SideDependentList<FootstepNode> goalNodes)
      {
         this.goalNodes = goalNodes;
      }

      double getManhattanDistance(FootstanceNode other)
      {
         return goalNodes.get(other.getStanceSide()).computeXYManhattanDistance(other.getStanceNode()) + goalNodes.get(other.getSwingSide()).computeXYManhattanDistance(other.getSwingNode());
      }
   }

   private void getNeighbors(FootstanceNode node, List<FootstanceNode> expansionToPack)
   {
      expansionToPack.clear();

      int nextStepY = node.getSwingNode().getYIndex();
      int nextStepYaw = node.getSwingNode().getYawIndex();
      int stanceNodeX = node.getStanceNode().getXIndex();

      expansionToPack.add(new FootstanceNode(node, stanceNodeX - 1, nextStepY, nextStepYaw));
      expansionToPack.add(new FootstanceNode(node, stanceNodeX + 0, nextStepY, nextStepYaw));
      expansionToPack.add(new FootstanceNode(node, stanceNodeX + 1, nextStepY, nextStepYaw));
   }
}

