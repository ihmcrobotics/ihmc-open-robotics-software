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
      setHeuristicsInflationWeight(other.getHeuristicsWeight());
      setMinXClearanceFromFoot(other.getMinXClearanceFromFoot());
      setMinYClearanceFromFoot(other.getMinYClearanceFromFoot());
      setCrawlSpeed(other.getCrawlSpeed());
      setTrotSpeed(other.getTrotSpeed());
      setPaceSpeed(other.getPaceSpeed());
      setProjectInsideDistance(other.getProjectionInsideDistance());
      setMinimumSurfaceInclineRadians(other.getMinimumSurfaceInclineRadians());
      setCliffHeightToAvoid(other.getCliffHeightToAvoid());
      setMinimumDistanceFromCliffBottoms(other.getMinimumDistanceFromCliffBottoms());
      setMinimumDistanceFromCliffTops(other.getMinimumDistanceFromCliffTops());
   }
}
