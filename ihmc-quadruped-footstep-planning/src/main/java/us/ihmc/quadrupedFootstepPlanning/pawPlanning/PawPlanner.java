package us.ihmc.quadrupedFootstepPlanning.pawPlanning;

import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public interface PawPlanner
{
   void setGroundPlane(QuadrupedGroundPlaneMessage groundPlane);

   void setPlanarRegionsList(PlanarRegionsList planarRegionsList);

   void setStart(PawPlannerStart start);

   void setGoal(PawPlannerGoal goal);

   void setTimeout(double timeout);

   default void setBestEffortTimeout(double timeout)
   {
   }

   PawPlanningResult plan();

   void cancelPlanning();

   PawPlan getPlan();

   default void setPlanningHorizonLength(double planningHorizon)
   {
   }

   default double getPlanningHorizonLength()
   {
      return Double.POSITIVE_INFINITY;
   }

   double getPlanningDuration();
}
