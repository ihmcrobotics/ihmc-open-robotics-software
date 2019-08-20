package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost;

import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;

public class PawNodeCostBuilder
{
   private final RequiredFactoryField<PawPlannerParameters> pawPlannerParameters = new RequiredFactoryField<>("pawPlannerParameters");
   private final RequiredFactoryField<QuadrupedXGaitSettingsReadOnly> xGaitSettings = new RequiredFactoryField<>("xGaitSettings");
   private final RequiredFactoryField<PawNodeSnapper> snapper = new RequiredFactoryField<>("snapper");

   private final OptionalFactoryField<Boolean> includeHeightCost = new OptionalFactoryField<>("includeHeightCost");

   public void setPawPlannerParameters(PawPlannerParameters pawPlannerParameters)
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

   public void setIncludeHeightCost(boolean includeHeightCost)
   {
      this.includeHeightCost.set(includeHeightCost);
   }

   public PawNodeCost buildCost()
   {
      includeHeightCost.setDefaultValue(false);

      CompositePawNodeCost compositePawNodeCost = new CompositePawNodeCost();

      if (includeHeightCost.get())
         compositePawNodeCost.addPawNodeCost(new PawHeightCost(pawPlannerParameters.get(), snapper.get()));

      compositePawNodeCost.addPawNodeCost(new PawDistanceAndYawBasedCost(pawPlannerParameters.get(), xGaitSettings.get()));
      compositePawNodeCost.addPawNodeCost(new PawXGaitCost(pawPlannerParameters.get(), xGaitSettings.get()));
      compositePawNodeCost.addPawNodeCost(new PerNodeCost(pawPlannerParameters.get()));

      FactoryTools.disposeFactory(this);

      return compositePawNodeCost;
   }

}
