package us.ihmc.footstepPlanning.graphSearch;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.List;

public class AStarFootstepPlannerIterationConductorTest
{
   @Test
   public void testSimple2DGridSearch()
   {
      ManhattanDistanceCalculator distanceCalculator = new ManhattanDistanceCalculator();
      AStarFootstepPlannerIterationConductor planner = new AStarFootstepPlannerIterationConductor(this::getNeighbors, (n1, n2, n3) -> true, (n1, n2, n3) -> 1.0, distanceCalculator::getManhattanDistance);

      DiscreteFootstep leftStartStep = new DiscreteFootstep(0, 1, 0, RobotSide.LEFT);
      DiscreteFootstep rightStartStep = new DiscreteFootstep(0, -1, 0, RobotSide.RIGHT);
      FootstepGraphNode startNode = new FootstepGraphNode(leftStartStep, rightStartStep);

      DiscreteFootstep leftGoalStep = new DiscreteFootstep(3, 1, 0, RobotSide.LEFT);
      DiscreteFootstep rightGoalStep = new DiscreteFootstep(3, -1, 0, RobotSide.RIGHT);
      FootstepGraphNode goalNode = new FootstepGraphNode(leftGoalStep, rightGoalStep);

      distanceCalculator.setGoalNode(new SideDependentList<>(leftGoalStep, rightGoalStep));
      planner.initialize(startNode);

      for (int i = 0; i < 4; i++)
      {
         AStarIterationData<FootstepGraphNode> iterationData = planner.doPlanningIteration(planner.getNextNode(), true);
         Assertions.assertEquals(iterationData.getParentNode().getEndStep().getXIndex(), i);
         Assertions.assertEquals(iterationData.getParentNode().getEndStep().getYIndex(), iterationData.getParentNode().getEndSide() == RobotSide.LEFT ? 1 : -1);
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
      private SideDependentList<DiscreteFootstep> goalSteps;

      public void setGoalNode(SideDependentList<DiscreteFootstep> goalSteps)
      {
         this.goalSteps = goalSteps;
      }

      double getManhattanDistance(FootstepGraphNode other)
      {
         return goalSteps.get(other.getEndSide()).computeXYManhattanDistance(other.getEndStep()) + goalSteps.get(other.getStartSide()).computeXYManhattanDistance(other.getStartStep());
      }
   }

   private void getNeighbors(FootstepGraphNode node, List<FootstepGraphNode> expansionToPack)
   {
      expansionToPack.clear();

      int nextStepY = node.getStartStep().getYIndex();
      int nextStepYaw = node.getStartStep().getYawIndex();
      int stanceNodeX = node.getEndStep().getXIndex();

      expansionToPack.add(new FootstepGraphNode(new DiscreteFootstep(stanceNodeX - 1, nextStepY, nextStepYaw, node.getStartSide()), node.getEndStep()));
      expansionToPack.add(new FootstepGraphNode(new DiscreteFootstep(stanceNodeX + 0, nextStepY, nextStepYaw, node.getStartSide()), node.getEndStep()));
      expansionToPack.add(new FootstepGraphNode(new DiscreteFootstep(stanceNodeX + 1, nextStepY, nextStepYaw, node.getStartSide()), node.getEndStep()));
   }
}

