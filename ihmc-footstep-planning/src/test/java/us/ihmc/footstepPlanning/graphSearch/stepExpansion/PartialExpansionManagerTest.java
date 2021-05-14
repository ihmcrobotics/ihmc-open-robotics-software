package us.ihmc.footstepPlanning.graphSearch.stepExpansion;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;

public class PartialExpansionManagerTest
{
   @Test
   public void testPartialExpansionWithoutRemainderOfNodes()
   {
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      int maxBranchFactor = 5;
      int numberOfPartialExpansions = 3;
      footstepPlannerParameters.setMaximumBranchFactor(maxBranchFactor);

      PartialExpansionManager partialExpansionManager = new PartialExpansionManager(footstepPlannerParameters);

      List<FootstepGraphNode> allNodes = new ArrayList<>();
      DiscreteFootstep startOfSwingStep = new DiscreteFootstep(0, 0, 0, RobotSide.RIGHT);
      for (int i = 0; i < maxBranchFactor; i++)
      {
         for (int j = 0; j < numberOfPartialExpansions; j++)
         {
            allNodes.add(new FootstepGraphNode(startOfSwingStep, new DiscreteFootstep(i, j, 0, RobotSide.LEFT)));
         }
      }

      partialExpansionManager.initialize(allNodes);
      List<FootstepGraphNode> partialExpansion = new ArrayList<>();

      for (int i = 0; i < numberOfPartialExpansions; i++)
      {
         partialExpansionManager.packPartialExpansion(partialExpansion);
         Assertions.assertEquals(partialExpansion.size(), maxBranchFactor);
         boolean lastExpansion = i == numberOfPartialExpansions - 1;
         Assertions.assertEquals(lastExpansion, partialExpansionManager.finishedExpansion());
      }
   }

   @Test
   public void testPartialExpansionAtBranchFactorOfOne()
   {
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();

      int maxBranchFactor = 1;
      int numberOfNodes = 4;
      footstepPlannerParameters.setMaximumBranchFactor(maxBranchFactor);

      PartialExpansionManager partialExpansionManager = new PartialExpansionManager(footstepPlannerParameters);

      List<FootstepGraphNode> allNodes = new ArrayList<>();
      DiscreteFootstep startOfSwingStep = new DiscreteFootstep(0, 0, 0, RobotSide.RIGHT);
      for (int i = 0; i < numberOfNodes; i++)
      {
         allNodes.add(new FootstepGraphNode(startOfSwingStep, new DiscreteFootstep(i, 0, 0, RobotSide.LEFT)));
      }

      partialExpansionManager.initialize(allNodes);
      List<FootstepGraphNode> partialExpansion = new ArrayList<>();

      for (int i = 0; i < numberOfNodes; i++)
      {
         partialExpansionManager.packPartialExpansion(partialExpansion);
         Assertions.assertEquals(partialExpansion.size(), maxBranchFactor);
         boolean lastExpansion = i == numberOfNodes - 1;
         Assertions.assertEquals(lastExpansion, partialExpansionManager.finishedExpansion());
      }
   }
}
