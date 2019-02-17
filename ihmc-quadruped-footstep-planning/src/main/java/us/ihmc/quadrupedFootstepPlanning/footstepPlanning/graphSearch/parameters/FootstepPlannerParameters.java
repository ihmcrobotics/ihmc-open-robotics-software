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

   double getMinimumStepYaw();

   double getMaximumStepYaw();

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

   double getMinXClearanceFromFoot();

   double getMinYClearanceFromFoot();

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

   /**
    * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
    * nearby that is cliffHeightToShiftAwayFrom higher than the candidate footstep, it will move away from it
    * until it is minimumDistanceFromCliffBottoms away from it.
    *
    * <p>
    * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
    * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
    * generator is capable of swinging over.
    * </p>
    */
   default double getCliffHeightToAvoid()
   {
      return 0.1;
   }

   /**
    * The planner can be setup to avoid footsteps near the bottom of "cliffs". When the footstep has a planar region
    * nearby that is {@link #getCliffHeightToAvoid} higher than the candidate footstep, it will move away from it
    * until it is minimumDistanceFromCliffBottoms away from it.
    *
    * <p>
    * If these values are set to zero, cliff avoidance will be turned off. This creates a risk that the robot will
    * hit the cliff with its swing foot. Therefore, these parameters should be set according to what the swing trajectory
    * generator is capable of swinging over.
    * </p>
    */
   default double getMinimumDistanceFromCliffBottoms()
   {
      return 0.03;
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
