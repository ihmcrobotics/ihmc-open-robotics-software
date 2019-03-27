package us.ihmc.quadrupedFootstepPlanning.footstepPlanning;

import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public interface QuadrupedFootstepPlanner
{
   void setGroundPlane(QuadrupedGroundPlaneMessage groundPlane);

   void setPlanarRegionsList(PlanarRegionsList planarRegionsList);

   void setStart(QuadrupedFootstepPlannerStart start);

   void setGoal(QuadrupedFootstepPlannerGoal goal);

   void setTimeout(double timeout);

   FootstepPlanningResult plan();

   void cancelPlanning();

   FootstepPlan getPlan();

   default void setPlanningHorizonLength(double planningHorizon)
   {
   }

   default double getPlanningHorizonLength()
   {
      return Double.POSITIVE_INFINITY;
   }

   double getPlanningDuration();
}
