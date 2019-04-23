package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters;

import controller_msgs.msg.dds.QuadrupedFootstepPlannerParametersPacket;

public interface FootstepPlannerParametersBasics extends FootstepPlannerParameters
{
   void setMaximumStepReach(double maximumStepReach);

   void setMaximumStepLength(double maximumStepLength);

   void setMinimumStepLength(double minimumStepLength);

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

   void setCrawlSpeed(double crawlSpeed);

   void setTrotSpeed(double trotSpeed);

   void setPaceSpeed(double paceSpeed);

   void setProjectInsideDistance(double projectionInsideDistance);

   void setMinimumSurfaceInclineRadians(double minimumSurfaceIncline);

   void setCliffHeightToAvoid(double cliffHeightToAvoid);

   void setMinimumDistanceFromCliffBottoms(double distance);

   void setMinimumDistanceFromCliffTops(double distance);

   default void set(FootstepPlannerParameters other)
   {
      setMaximumStepReach(other.getMaximumStepReach());
      setMaximumStepLength(other.getMaximumStepLength());
      setMinimumStepLength(other.getMinimumStepLength());
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
      setCrawlSpeed(other.getCrawlSpeed());
      setTrotSpeed(other.getTrotSpeed());
      setPaceSpeed(other.getPaceSpeed());
      setProjectInsideDistance(other.getProjectInsideDistance());
      setMinimumSurfaceInclineRadians(other.getMinimumSurfaceInclineRadians());
      setCliffHeightToAvoid(other.getCliffHeightToAvoid());
      setMinimumDistanceFromCliffBottoms(other.getMinimumDistanceFromCliffBottoms());
      setMinimumDistanceFromCliffTops(other.getMinimumDistanceFromCliffTops());
   }

   default void set(QuadrupedFootstepPlannerParametersPacket other)
   {
      if (other.getMaximumStepReach() != other.NO_VALUE_DOUBLE)
         setMaximumStepReach(other.getMaximumStepReach());
      if (other.getMaximumStepLength() != other.NO_VALUE_DOUBLE)
         setMaximumStepLength(other.getMaximumStepLength());
      if (other.getMinimumStepLength() != other.NO_VALUE_DOUBLE)
         setMinimumStepLength(other.getMinimumStepLength());
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
      if (other.getCrawlSpeed() != other.NO_VALUE_DOUBLE)
         setCrawlSpeed(other.getCrawlSpeed());
      if (other.getTrotSpeed() != other.NO_VALUE_DOUBLE)
         setTrotSpeed(other.getTrotSpeed());
      if (other.getPaceSpeed() != other.NO_VALUE_DOUBLE)
         setPaceSpeed(other.getPaceSpeed());
      if (other.getProjectionInsideDistance() != other.NO_VALUE_DOUBLE)
         setProjectInsideDistance(other.getProjectionInsideDistance());
      if (other.getMinimumSurfaceInclineRadians() != other.NO_VALUE_DOUBLE)
         setMinimumSurfaceInclineRadians(other.getMinimumSurfaceInclineRadians());
      if (other.getCliffHeightToAvoid() != other.NO_VALUE_DOUBLE)
         setCliffHeightToAvoid(other.getCliffHeightToAvoid());
      if (other.getMinimumDistanceFromCliffBottoms() != other.NO_VALUE_DOUBLE)
         setMinimumDistanceFromCliffBottoms(other.getMinimumDistanceFromCliffBottoms());
      if (other.getMinimumDistanceFromCliffTops() != other.NO_VALUE_DOUBLE)
         setMinimumDistanceFromCliffTops(other.getMinimumDistanceFromCliffTops());
   }
}
