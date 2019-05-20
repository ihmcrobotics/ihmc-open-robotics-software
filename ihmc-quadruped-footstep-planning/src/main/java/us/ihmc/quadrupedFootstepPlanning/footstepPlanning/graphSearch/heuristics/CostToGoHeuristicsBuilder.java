package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;

public class CostToGoHeuristicsBuilder
{
   private final RequiredFactoryField<FootstepPlannerParameters> footstepPlannerParameters = new RequiredFactoryField<>("footstepPlannerParameters");
   private final RequiredFactoryField<QuadrupedXGaitSettingsReadOnly> xGaitSettings = new RequiredFactoryField<>("xGaitSettings");
   private final RequiredFactoryField<FootstepNodeSnapper> snapper = new RequiredFactoryField<>("snapper");

   private final OptionalFactoryField<Boolean> useDistanceBasedHeuristics = new OptionalFactoryField<>("useDistanceBasedHeuristics");

   public void setFootstepPlannerParameters(FootstepPlannerParameters footstepPlannerParameters)
   {
      this.footstepPlannerParameters.set(footstepPlannerParameters);
   }

   public void setXGaitSettings(QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.xGaitSettings.set(xGaitSettings);
   }

   public void setSnapper(FootstepNodeSnapper snapper)
   {
      this.snapper.set(snapper);
   }

   public void setUseDistanceBasedHeuristics(boolean useDistanceBasedHeuristics)
   {
      this.useDistanceBasedHeuristics.set(useDistanceBasedHeuristics);
   }

   public CostToGoHeuristics buildHeuristics()
   {
      useDistanceBasedHeuristics.setDefaultValue(false);

      CompositeCostToGoHeuristics costToGoHeuristics = new CompositeCostToGoHeuristics(footstepPlannerParameters.get());

      if (useDistanceBasedHeuristics.get())
      {
         costToGoHeuristics.addCostToGoHeuristic(new DistanceAndYawBasedHeuristics(snapper.get(), footstepPlannerParameters.get()));
         costToGoHeuristics.addCostToGoHeuristic(new SpeedBasedHeuristics(footstepPlannerParameters.get(), xGaitSettings.get()));
      }
      else
      {
         costToGoHeuristics.addCostToGoHeuristic(new SpeedAndYawBasedHeuristics(snapper.get(), footstepPlannerParameters.get(), xGaitSettings.get()));
      }

      FactoryTools.disposeFactory(this);

      return costToGoHeuristics;
   }

}
