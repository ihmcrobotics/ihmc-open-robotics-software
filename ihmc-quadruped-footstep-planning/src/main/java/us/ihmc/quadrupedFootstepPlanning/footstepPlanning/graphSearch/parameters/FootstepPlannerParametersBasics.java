package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters;

import controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket;

public interface FootstepPlannerParametersBasics extends FootstepPlannerParameters
{
   void setMaximumFrontStepReach(double maximumStepReach);

   void setMaximumFrontStepLength(double maximumStepLength);

   void setMinimumFrontStepLength(double minimumStepLength);

   void setMaximumHindStepReach(double maximumStepReach);

   void setMaximumHindStepLength(double maximumStepLength);

   void setMinimumHindStepLength(double minimumStepLength);

   void setMaximumStepWidth(double maximumStepWidth);

   void setMinimumStepWidth(double minimumStepWidth);

   void setMinimumStepYaw(double minimumStepYaw);

   void setMaximumStepYaw(double maximumStepYaw);

   void setMaximumStepChangeZ(double maximumStepChangeZ);

   void setBodyGroundClearance(double bodyGroundClearance);

   void setDistanceHeuristicWeight(double distanceHeuristicWeight);

   void setYawWeight(double yawWeight);

   void setXGaitWeight(double xGaitWeight);

   void setCostPerStep(double costPerStep);

   void setStepUpWeight(double stepUpWeight);

   void setStepDownWeight(double stepDownWeight);

   void setHeuristicsInflationWeight(double heuristicsInflationWeight);

   void setMinXClearanceFromFoot(double minXClearanceFromFoot);

   void setMinYClearanceFromFoot(double minYClearanceFromFoot);

   void setMaxWalkingSpeedMultiplier(double multiplier);

   void setProjectInsideDistanceForExpansion(double projectionInsideDistanceForExpansion);

   void setProjectInsideDistanceForPostProcessing(double projectionInsideDistanceForPostProcessing);

   void setProjectInsideUsingConvexHull(boolean projectInsideUsingConvexHull);

   void setMaximumXYWiggleDistance(double wiggleDistance);

   void setMinimumSurfaceInclineRadians(double minimumSurfaceIncline);

   void setCliffHeightToAvoid(double cliffHeightToAvoid);

   void setMinimumDistanceFromCliffBottoms(double distance);

   void setMinimumDistanceFromCliffTops(double distance);

   default void set(FootstepPlannerParameters other)
   {
      setMaximumFrontStepReach(other.getMaximumFrontStepReach());
      setMaximumFrontStepLength(other.getMaximumFrontStepLength());
      setMinimumFrontStepLength(other.getMinimumFrontStepLength());
      setMaximumHindStepReach(other.getMaximumHindStepReach());
      setMaximumHindStepLength(other.getMaximumHindStepLength());
      setMinimumHindStepLength(other.getMinimumHindStepLength());
      setMaximumStepWidth(other.getMaximumStepWidth());
      setMinimumStepWidth(other.getMinimumStepWidth());
      setMinimumStepYaw(other.getMinimumStepYaw());
      setMaximumStepYaw(other.getMaximumStepYaw());
      setMaximumStepChangeZ(other.getMaximumStepChangeZ());
      setBodyGroundClearance(other.getBodyGroundClearance());
      setDistanceHeuristicWeight(other.getDistanceHeuristicWeight());
      setYawWeight(other.getYawWeight());
      setXGaitWeight(other.getXGaitWeight());
      setCostPerStep(other.getCostPerStep());
      setStepUpWeight(other.getStepUpWeight());
      setStepDownWeight(other.getStepDownWeight());
      setHeuristicsInflationWeight(other.getHeuristicsInflationWeight());
      setMinXClearanceFromFoot(other.getMinXClearanceFromFoot());
      setMinYClearanceFromFoot(other.getMinYClearanceFromFoot());
      setMaxWalkingSpeedMultiplier(other.getMaxWalkingSpeedMultiplier());
      setProjectInsideDistanceForExpansion(other.getProjectInsideDistanceForExpansion());
      setProjectInsideDistanceForPostProcessing(other.getProjectInsideDistanceForPostProcessing());
      setProjectInsideUsingConvexHull(other.getProjectInsideUsingConvexHull());
      setMaximumXYWiggleDistance(other.getMaximumXYWiggleDistance());
      setMinimumSurfaceInclineRadians(other.getMinimumSurfaceInclineRadians());
      setCliffHeightToAvoid(other.getCliffHeightToAvoid());
      setMinimumDistanceFromCliffBottoms(other.getMinimumDistanceFromCliffBottoms());
      setMinimumDistanceFromCliffTops(other.getMinimumDistanceFromCliffTops());
   }

   default void set(QuadrupedFootstepPlannerParametersPacket other)
   {
      if (other.getMaximumFrontStepReach() != other.NO_VALUE_DOUBLE)
         setMaximumFrontStepReach(other.getMaximumFrontStepReach());
      if (other.getMaximumFrontStepLength() != other.NO_VALUE_DOUBLE)
         setMaximumFrontStepLength(other.getMaximumFrontStepLength());
      if (other.getMinimumFrontStepLength() != other.NO_VALUE_DOUBLE)
         setMinimumFrontStepLength(other.getMinimumFrontStepLength());
      if (other.getMaximumHindStepReach() != other.NO_VALUE_DOUBLE)
         setMaximumHindStepReach(other.getMaximumHindStepReach());
      if (other.getMaximumHindStepLength() != other.NO_VALUE_DOUBLE)
         setMaximumHindStepLength(other.getMaximumHindStepLength());
      if (other.getMinimumHindStepLength() != other.NO_VALUE_DOUBLE)
         setMinimumHindStepLength(other.getMinimumHindStepLength());
      if (other.getMaximumStepWidth() != other.NO_VALUE_DOUBLE)
         setMaximumStepWidth(other.getMaximumStepWidth());
      if (other.getMinimumStepWidth() != other.NO_VALUE_DOUBLE)
         setMinimumStepWidth(other.getMinimumStepWidth());
      if (other.getMinimumStepYaw() != other.NO_VALUE_DOUBLE)
         setMinimumStepYaw(other.getMinimumStepYaw());
      if (other.getMaximumStepYaw() != other.NO_VALUE_DOUBLE)
         setMaximumStepYaw(other.getMaximumStepYaw());
      if (other.getMaximumStepChangeZ() != other.NO_VALUE_DOUBLE)
         setMaximumStepChangeZ(other.getMaximumStepChangeZ());
      if (other.getBodyGroundClearance() != other.NO_VALUE_DOUBLE)
         setBodyGroundClearance(other.getBodyGroundClearance());
      if (other.getMaxWalkingSpeedMultiplier() != other.NO_VALUE_DOUBLE)
         setMaxWalkingSpeedMultiplier(other.getMaxWalkingSpeedMultiplier());
      if (other.getDistanceHeuristicWeight() != other.NO_VALUE_DOUBLE)
         setDistanceHeuristicWeight(other.getDistanceHeuristicWeight());
      if (other.getYawWeight() != other.NO_VALUE_DOUBLE)
         setYawWeight(other.getYawWeight());
      if (other.getXGaitWeight() != other.NO_VALUE_DOUBLE)
         setXGaitWeight(other.getXGaitWeight());
      if (other.getCostPerStep() != other.NO_VALUE_DOUBLE)
         setCostPerStep(other.getCostPerStep());
      if (other.getStepUpWeight() != other.NO_VALUE_DOUBLE)
         setStepUpWeight(other.getStepUpWeight());
      if (other.getStepDownWeight() != other.NO_VALUE_DOUBLE)
         setStepDownWeight(other.getStepDownWeight());
      if (other.getHeuristicsWeight() != other.NO_VALUE_DOUBLE)
         setHeuristicsInflationWeight(other.getHeuristicsWeight());
      if (other.getMinXClearanceFromFoot() != other.NO_VALUE_DOUBLE)
         setMinXClearanceFromFoot(other.getMinXClearanceFromFoot());
      if (other.getMinYClearanceFromFoot() != other.NO_VALUE_DOUBLE)
         setMinYClearanceFromFoot(other.getMinYClearanceFromFoot());
      if (other.getProjectionInsideDistanceForExpansion() != other.NO_VALUE_DOUBLE)
         setProjectInsideDistanceForExpansion(other.getProjectionInsideDistanceForExpansion());
      if (other.getProjectionInsideDistanceForPostProcessing() != other.NO_VALUE_DOUBLE)
         setProjectInsideDistanceForPostProcessing(other.getProjectionInsideDistanceForPostProcessing());
      if (other.getMaximumXyWiggleDistance() != other.NO_VALUE_DOUBLE)
         setMaximumXYWiggleDistance(other.getMaximumXyWiggleDistance());
      if (other.getMinimumSurfaceInclineRadians() != other.NO_VALUE_DOUBLE)
         setMinimumSurfaceInclineRadians(other.getMinimumSurfaceInclineRadians());
      if (other.getCliffHeightToAvoid() != other.NO_VALUE_DOUBLE)
         setCliffHeightToAvoid(other.getCliffHeightToAvoid());
      if (other.getMinimumDistanceFromCliffBottoms() != other.NO_VALUE_DOUBLE)
         setMinimumDistanceFromCliffBottoms(other.getMinimumDistanceFromCliffBottoms());
      if (other.getMinimumDistanceFromCliffTops() != other.NO_VALUE_DOUBLE)
         setMinimumDistanceFromCliffTops(other.getMinimumDistanceFromCliffTops());

      setProjectInsideUsingConvexHull(other.getProjectInsideUsingConvexHull());
   }
}
