package us.ihmc.humanoidBehaviors.tools.footstepPlanner;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepPlanEtcetera extends FootstepPlan
{
   private final PlanarRegionsList planarRegions;
   private final SideDependentList<? extends Pose3DReadOnly> startFootPoses;
   private final SideDependentList<ConvexPolygon2D> startFootholds;
   private final SwingPlannerType swingPlannerType;

   public FootstepPlanEtcetera(FootstepPlan footstepPlan,
                               PlanarRegionsList planarRegions,
                               SideDependentList<? extends Pose3DReadOnly> startFootPoses,
                               SideDependentList<ConvexPolygon2D> startFootholds,
                               SwingPlannerType swingPlannerType)
   {
      super(footstepPlan);
      this.planarRegions = planarRegions;
      this.startFootPoses = startFootPoses;
      this.startFootholds = startFootholds;
      this.swingPlannerType = swingPlannerType;
   }

   public PlanarRegionsList getPlanarRegions()
   {
      return planarRegions;
   }

   public SideDependentList<? extends Pose3DReadOnly> getStartFootPoses()
   {
      return startFootPoses;
   }

   public SideDependentList<ConvexPolygon2D> getStartFootholds()
   {
      return startFootholds;
   }

   public SwingPlannerType getSwingPlannerType()
   {
      return swingPlannerType;
   }
}
