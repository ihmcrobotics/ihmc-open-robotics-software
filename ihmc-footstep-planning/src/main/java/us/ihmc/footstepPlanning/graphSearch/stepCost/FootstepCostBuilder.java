package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.tools.factories.FactoryTools;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.factories.RequiredFactoryField;

public class FootstepCostBuilder
{
   private final RequiredFactoryField<FootstepPlannerParametersReadOnly> footstepPlannerParameters = new RequiredFactoryField<>("footstepPlannerParameters");
   private final RequiredFactoryField<FootstepNodeSnapperReadOnly> snapper = new RequiredFactoryField<>("snapper");

   private final OptionalFactoryField<Boolean> includeHeightCost = new OptionalFactoryField<>("includeHeightCost");
   private final OptionalFactoryField<Boolean> includePitchAndRollCost = new OptionalFactoryField<>("includePitchAndRollCost");
   private final OptionalFactoryField<Boolean> includeBoundingBoxCost = new OptionalFactoryField<>("includeBoundingBoxCost");
   private final OptionalFactoryField<FootstepNodeBodyCollisionDetector> collisionDetector = new OptionalFactoryField<>("collisionDetector");

   public void setFootstepPlannerParameters(FootstepPlannerParametersReadOnly footstepPlannerParameters)
   {
      this.footstepPlannerParameters.set(footstepPlannerParameters);
   }

   public void setSnapper(FootstepNodeSnapperReadOnly snapper)
   {
      this.snapper.set(snapper);
   }

   public void setIncludeHeightCost(boolean includeHeightCost)
   {
      this.includeHeightCost.set(includeHeightCost);
   }

   public void setIncludePitchAndRollCost(boolean includePitchAndRollCost)
   {
      this.includePitchAndRollCost.set(includePitchAndRollCost);
   }

   public void setCollisionDetector(FootstepNodeBodyCollisionDetector collisionDetector)
   {
      this.collisionDetector.set(collisionDetector);
   }

   public void setIncludeBoundingBoxCost(boolean includeBoundingBoxCost)
   {
      this.includeBoundingBoxCost.set(includeBoundingBoxCost);
   }

   public FootstepCost buildCost()
   {
      includeHeightCost.setDefaultValue(false);
      includePitchAndRollCost.setDefaultValue(false);
      includeBoundingBoxCost.setDefaultValue(false);

      CompositeFootstepCost compositeFootstepCost = new CompositeFootstepCost();

      if (includeHeightCost.get())
         compositeFootstepCost.addFootstepCost(new HeightCost(footstepPlannerParameters.get(), snapper.get()));

      if (includePitchAndRollCost.get())
         compositeFootstepCost.addFootstepCost(new PitchAndRollBasedCost(footstepPlannerParameters.get(), snapper.get()));

      if(includeBoundingBoxCost.get() && collisionDetector.hasValue())
         compositeFootstepCost.addFootstepCost(new BodyCollisionNodeCost(collisionDetector.get(), footstepPlannerParameters.get(), snapper.get()));

      compositeFootstepCost.addFootstepCost(new DistanceAndYawBasedCost(footstepPlannerParameters.get()));

      FactoryTools.disposeFactory(this);

      return compositeFootstepCost;
   }

}
