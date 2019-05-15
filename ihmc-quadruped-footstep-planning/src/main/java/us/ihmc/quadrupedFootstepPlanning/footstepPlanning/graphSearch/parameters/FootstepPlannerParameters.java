package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters;

import controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.filters.SteppableRegionFilter;
import us.ihmc.robotics.geometry.PlanarRegion;

public interface FootstepPlannerParameters
{
   /**
    * The total maximum Euclidean distance length.
    */
   double getMaximumFrontStepReach();

   double getMaximumFrontStepLength();

   double getMinimumFrontStepLength();

   default double getMaximumHindStepReach()
   {
      return getMaximumFrontStepReach();
   }

   default double getMaximumHindStepLength()
   {
      return getMaximumFrontStepLength();
   }

   default double getMinimumHindStepLength()
   {
      return getMinimumFrontStepLength();
   }

   double getMaximumStepWidth();

   double getMinimumStepWidth();

   double getMinimumStepYaw();

   double getMaximumStepYaw();

   double getMaximumStepChangeZ();

   double getBodyGroundClearance();

   double getDistanceHeuristicWeight();

   double getYawWeight();

   double getXGaitWeight();

   double getCostPerStep();

   double getStepUpWeight();

   double getStepDownWeight();

   double getHeuristicsInflationWeight();

   double getMinXClearanceFromFoot();

   double getMinYClearanceFromFoot();

   default double getMaxWalkingSpeedMultiplier()
   {
      return 0.8;
   }

   /**
    * Distance which a foothold is projected into planar region during expansion and node checking. Should be a positive value,
    * e.g. 0.02 means footholds are projected 2cm inside. If this is a non-positive value then no projection is performed.
    */
   double getProjectInsideDistanceForExpansion();

   /**
    * Distance which a foothold is projected into planar region during post processing. Should be a positive value,
    * e.g. 0.02 means footholds are projected 2cm inside. If this is a non-positive value then no projection is performed.
    */
   double getProjectInsideDistanceForPostProcessing();

   boolean getProjectInsideUsingConvexHullDuringExpansion();

   boolean getProjectInsideUsingConvexHullDuringPostProcessing();

   /***
    * Maximum distance that the snap and wiggler is allowed to wiggle the footstep node.
    */
   double getMaximumXYWiggleDistance();

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
      return 0.15;
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
   default double getMinimumDistanceFromCliffTops()
   {
      return 0.01;
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


   default QuadrupedFootstepPlannerParametersPacket getAsPacket()
   {
      QuadrupedFootstepPlannerParametersPacket packet = new QuadrupedFootstepPlannerParametersPacket();
      packet.setMaximumFrontStepReach(getMaximumFrontStepReach());
      packet.setMaximumFrontStepLength(getMaximumFrontStepLength());
      packet.setMinimumFrontStepLength(getMinimumFrontStepLength());
      packet.setMaximumHindStepReach(getMaximumHindStepReach());
      packet.setMaximumHindStepLength(getMaximumHindStepLength());
      packet.setMinimumHindStepLength(getMinimumHindStepLength());
      packet.setMaximumStepWidth(getMaximumStepWidth());
      packet.setMinimumStepWidth(getMinimumStepWidth());
      packet.setMinimumStepYaw(getMinimumStepYaw());
      packet.setMaximumStepYaw(getMaximumStepYaw());
      packet.setMaximumStepChangeZ(getMaximumStepChangeZ());
      packet.setBodyGroundClearance(getBodyGroundClearance());
      packet.setMaxWalkingSpeedMultiplier(getMaxWalkingSpeedMultiplier());
      packet.setDistanceHeuristicWeight(getDistanceHeuristicWeight());
      packet.setYawWeight(getYawWeight());
      packet.setXGaitWeight(getXGaitWeight());
      packet.setCostPerStep(getCostPerStep());
      packet.setStepUpWeight(getStepUpWeight());
      packet.setStepDownWeight(getStepDownWeight());
      packet.setHeuristicsWeight(getHeuristicsInflationWeight());
      packet.setMinXClearanceFromFoot(getMinXClearanceFromFoot());
      packet.setMinYClearanceFromFoot(getMinYClearanceFromFoot());
      packet.setProjectionInsideDistanceForExpansion(getProjectInsideDistanceForExpansion());
      packet.setProjectionInsideDistanceForPostProcessing(getProjectInsideDistanceForPostProcessing());
      packet.setProjectInsideUsingConvexHullDuringExpansion(getProjectInsideUsingConvexHullDuringExpansion());
      packet.setProjectInsideUsingConvexHullDuringPostProcessing(getProjectInsideUsingConvexHullDuringPostProcessing());
      packet.setMaximumXyWiggleDistance(getMaximumXYWiggleDistance());
      packet.setMinimumSurfaceInclineRadians(getMinimumSurfaceInclineRadians());
      packet.setCliffHeightToAvoid(getCliffHeightToAvoid());
      packet.setMinimumDistanceFromCliffBottoms(getMinimumDistanceFromCliffBottoms());
      packet.setMinimumDistanceFromCliffTops(getMinimumDistanceFromCliffTops());

      return packet;
   }
}
