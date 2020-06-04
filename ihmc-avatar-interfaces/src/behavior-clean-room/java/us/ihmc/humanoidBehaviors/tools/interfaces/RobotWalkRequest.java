package us.ihmc.humanoidBehaviors.tools.interfaces;

import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class RobotWalkRequest
{
   private final FootstepPlan footstepPlan; // TODO: Need ReadOnly FootstepPlan
   private final PlanarRegionsList planarRegions;

   public RobotWalkRequest(FootstepPlan footstepPlan, PlanarRegionsList planarRegions)
   {

      this.footstepPlan = footstepPlan;
      this.planarRegions = planarRegions;
   }

   public FootstepPlan getFootstepPlan()
   {
      return footstepPlan;
   }

   public PlanarRegionsList getPlanarRegions()
   {
      return planarRegions;
   }
}
