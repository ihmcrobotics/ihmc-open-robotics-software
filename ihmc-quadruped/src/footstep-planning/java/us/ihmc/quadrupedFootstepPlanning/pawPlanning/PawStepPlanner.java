package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import ihmc_common_msgs.msg.dds.GroundPlaneMessage;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface PawStepPlanner
{
   void setGroundPlane(GroundPlaneMessage groundPlane);

   void setPlanarRegionsList(PlanarRegionsList planarRegionsList);

   void setStart(PawStepPlannerStart start);

   void setGoal(PawStepPlannerGoal goal);

   void setTimeout(double timeout);

   default void setBestEffortTimeout(double timeout)
   {
   }

   PawStepPlanningResult plan();

   void cancelPlanning();

   PawStepPlan getPlan();

   default void setPlanningHorizonLength(double planningHorizon)
   {
   }

   default double getPlanningHorizonLength()
   {
      return Double.POSITIVE_INFINITY;
   }

   double getPlanningDuration();
}
