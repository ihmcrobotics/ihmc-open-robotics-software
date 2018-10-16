package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;

public class FootstepCostBuilder
{
   private final RequiredFactoryField<FootstepPlannerParameters> footstepPlannerParameters = new RequiredFactoryField<>("footstepPlannerParameters");

   private final OptionalFactoryField<Boolean> includeHeightCost = new OptionalFactoryField<>("includeHeightCost");
   private final OptionalFactoryField<Boolean> useQuadraticHeightCost = new OptionalFactoryField<>("useQuadraticHeightCost");
   private final OptionalFactoryField<Boolean> usePitchAndRollCost = new OptionalFactoryField<>("usePitchAndRollCost");

   public void setFootstepPlannerParameters(FootstepPlannerParameters footstepPlannerParameters)
   {
      this.footstepPlannerParameters.set(footstepPlannerParameters);
   }

   public void setIncludeHeightCost(boolean includeHeightCost)
   {
      this.includeHeightCost.set(includeHeightCost);
   }

   public void setUseQuadraticHeightCost(boolean useQuadraticHeightCost)
   {
      this.useQuadraticHeightCost.set(useQuadraticHeightCost);
   }

   public void setUsePitchAndRollCost(boolean usePitchAndRollCost)
   {
      this.usePitchAndRollCost.set(usePitchAndRollCost);
   }

   public FootstepCost buildCost()
   {
      includeHeightCost.setDefaultValue(false);
      useQuadraticHeightCost.setDefaultValue(false);
      usePitchAndRollCost.setDefaultValue(false);

      CompositeFootstepCost compositeFootstepCost = new CompositeFootstepCost();

      if (includeHeightCost.get())
      {
         if (useQuadraticHeightCost.get())
            compositeFootstepCost.addFootstepCost(new QuadraticHeightCost(footstepPlannerParameters.get().getCostParameters()));
         else
            compositeFootstepCost.addFootstepCost(new LinearHeightCost(footstepPlannerParameters.get().getCostParameters()));
      }

      if (usePitchAndRollCost.get())
         compositeFootstepCost.addFootstepCost(new PitchAndRollBasedCost(footstepPlannerParameters.get().getCostParameters()));

      compositeFootstepCost.addFootstepCost(new DistanceAndYawBasedCost(footstepPlannerParameters.get()));

      FactoryTools.disposeFactory(this);

      return compositeFootstepCost;
   }

}
