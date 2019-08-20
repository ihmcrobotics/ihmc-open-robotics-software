package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParametersReadOnly;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;

public class PawPlaningCostToGoHeuristicsBuilder
{
   private final RequiredFactoryField<PawPlannerParametersReadOnly> pawPlannerParameters = new RequiredFactoryField<>("pawPlannerParameters");
   private final RequiredFactoryField<QuadrupedXGaitSettingsReadOnly> xGaitSettings = new RequiredFactoryField<>("xGaitSettings");
   private final RequiredFactoryField<PawNodeSnapper> snapper = new RequiredFactoryField<>("snapper");

   private final OptionalFactoryField<Boolean> useDistanceBasedHeuristics = new OptionalFactoryField<>("useDistanceBasedHeuristics");

   public void setPawPlannerParameters(PawPlannerParametersReadOnly pawPlannerParameters)
   {
      this.pawPlannerParameters.set(pawPlannerParameters);
   }

   public void setXGaitSettings(QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.xGaitSettings.set(xGaitSettings);
   }

   public void setSnapper(PawNodeSnapper snapper)
   {
      this.snapper.set(snapper);
   }

   public void setUseDistanceBasedHeuristics(boolean useDistanceBasedHeuristics)
   {
      this.useDistanceBasedHeuristics.set(useDistanceBasedHeuristics);
   }

   public PawPlanningCostToGoHeuristics buildHeuristics()
   {
      useDistanceBasedHeuristics.setDefaultValue(false);

      CompositePawPlanningCostToGoHeuristics costToGoHeuristics = new CompositePawPlanningCostToGoHeuristics(pawPlannerParameters.get());

      if (useDistanceBasedHeuristics.get())
      {
         costToGoHeuristics.addCostToGoHeuristic(new PawDistanceAndYawBasedHeuristics(snapper.get(), pawPlannerParameters.get()));
         costToGoHeuristics.addCostToGoHeuristic(new PawSpeedBasedHeuristics(pawPlannerParameters.get(), xGaitSettings.get()));
      }
      else
      {
         costToGoHeuristics.addCostToGoHeuristic(new PawSpeedAndYawBasedHeuristics(snapper.get(), pawPlannerParameters.get(), xGaitSettings.get()));
      }

      FactoryTools.disposeFactory(this);

      return costToGoHeuristics;
   }

}
