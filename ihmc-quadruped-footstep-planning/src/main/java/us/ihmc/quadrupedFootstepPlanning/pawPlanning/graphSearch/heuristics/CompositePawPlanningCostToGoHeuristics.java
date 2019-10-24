package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics;

import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;

import java.util.ArrayList;
import java.util.List;

public class CompositePawPlanningCostToGoHeuristics extends PawPlanningCostToGoHeuristics
{
   private final List<PawPlanningCostToGoHeuristics> costToGoHeuristics;

   public CompositePawPlanningCostToGoHeuristics(PawStepPlannerParametersReadOnly parameters)
   {
      super(parameters);

      this.costToGoHeuristics = new ArrayList<>();
   }

   public CompositePawPlanningCostToGoHeuristics(PawStepPlannerParametersReadOnly parameters, List<PawPlanningCostToGoHeuristics> costToGoHeuristics)
   {
      super(parameters);

      this.costToGoHeuristics = costToGoHeuristics;
   }

   public void addCostToGoHeuristic(PawPlanningCostToGoHeuristics costToGoHeuristic)
   {
      costToGoHeuristics.add(costToGoHeuristic);
   }

   @Override
   protected double computeHeuristics(PawNode node)
   {
      double cost = 0.0;
      for (PawPlanningCostToGoHeuristics pawCost : costToGoHeuristics)
         cost += pawCost.computeHeuristics(node);

      return cost;
   }

   @Override
   public void setGoalPose(FramePose3DReadOnly goalPose)
   {
      for (PawPlanningCostToGoHeuristics pawCost : costToGoHeuristics)
         pawCost.setGoalPose(goalPose);
   }
}
