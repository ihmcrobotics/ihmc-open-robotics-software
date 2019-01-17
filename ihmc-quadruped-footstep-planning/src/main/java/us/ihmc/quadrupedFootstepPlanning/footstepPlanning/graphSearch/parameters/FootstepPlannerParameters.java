package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.filters.SteppableRegionFilter;
import us.ihmc.robotics.geometry.PlanarRegion;

public interface FootstepPlannerParameters
{
   double getMaximumStepReach();

   double getMaximumStepWidth();

   double getMaximumStepCycleDistance();

   double getMinimumStepLength();

   double getMinimumStepWidth();

   double getMaximumStepChangeZ();

   double getMaximumStepCycleChangeZ();

   double getBodyGroundClearance();

   double getForwardWeight();

   double getLateralWeight();

   double getYawWeight();

   double getCostPerStep();

   double getStepUpWeight();

   double getStepDownWeight();

   double getHeuristicsWeight();

   /**
    * The planner will ignore candidate footsteps if they are on a planar region with an incline that is higher
    * then the value specified here.
    *
    * <p>
    * More specifically, if a footstep has an associated planar region and that regions surface normal has a
    * z-value less than cos(minimumSurfaceInclineRadians), it will be rejected.
    * </p>
    */
   default double getMinimumSurfaceInclineRadians()
   {
      return Math.toRadians(45.0);
   }

   default SteppableRegionFilter getSteppableRegionFilter()
   {
      return new SteppableRegionFilter()
      {
         private Vector3D vertical = new Vector3D(0.0, 0.0, 1.0);

         @Override
         public boolean isPlanarRegionSteppable(PlanarRegion query)
         {
            double angle = query.getNormal().angle(vertical);

            if (angle > getMinimumSurfaceInclineRadians() + 1e-5)
               return false;

            return true;
         }
      };
   }
}
