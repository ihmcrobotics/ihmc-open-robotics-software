package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;

public abstract class PawPlanningCostToGoHeuristics
{
   private DoubleProvider heuristicsInflationWeight;
   protected final PawStepPlannerParametersReadOnly parameters;

   protected final FramePose3D goalPose = new FramePose3D();

   public PawPlanningCostToGoHeuristics(PawStepPlannerParametersReadOnly parameters)
   {
      this.parameters = parameters;
   }

   public void setHeuristicsInflationWeight(DoubleProvider heuristicsInflationWeight)
   {
      this.heuristicsInflationWeight = heuristicsInflationWeight;
   }

   public double compute(PawNode node)
   {
      return heuristicsInflationWeight.getValue() * computeHeuristics(node);
   }

   public double getWeight()
   {
      return heuristicsInflationWeight.getValue();
   }

   protected abstract double computeHeuristics(PawNode node);

   public void setGoalPose(FramePose3DReadOnly goalPose)
   {
      this.goalPose.set(goalPose);
   }
}
