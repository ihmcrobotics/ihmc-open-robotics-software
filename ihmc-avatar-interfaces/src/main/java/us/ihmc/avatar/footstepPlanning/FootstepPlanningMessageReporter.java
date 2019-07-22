package us.ihmc.avatar.footstepPlanning;

import controller_msgs.msg.dds.BodyPathPlanMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlannerStatusMessage;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerStatus;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.Optional;

public class FootstepPlanningMessageReporter
{
   private static final boolean debug = false;


   public static FootstepPlannerStatusMessage packStatus(FootstepPlannerStatus status)
   {
      FootstepPlannerStatusMessage message = new FootstepPlannerStatusMessage();
      message.setFootstepPlannerStatus(status.toByte());

      return message;
   }

   public static BodyPathPlanMessage packPathResult(BodyPathPlan bodyPathPlan, FootstepPlanningResult status, PlanarRegionsList planarRegionsList, int planId)
   {
      if (debug)
      {
         PrintTools.info("Finished planning path. Result: " + status);
      }

      BodyPathPlanMessage result = new BodyPathPlanMessage();
      if (bodyPathPlan != null)
      {
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
            result.getBodyPath().add().set(bodyPathPlan.getWaypoint(i));

         result.getPathPlannerStartPose().set(bodyPathPlan.getStartPose());
         result.getPathPlannerGoalPose().set(bodyPathPlan.getGoalPose());
      }

      if(planarRegionsList != null)
         result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
      result.setPlanId(planId);
      result.setFootstepPlanningResult(status.toByte());
      return result;
   }

   public static FootstepPlanningToolboxOutputStatus packStepResult(FootstepPlan footstepPlan, BodyPathPlan bodyPathPlan, FootstepPlanningResult status,
                                                              double timeTaken, PlanarRegionsList planarRegionsList, int planId)
   {
      if (debug)
      {
         PrintTools.info("Finished planning. Result: " + status);
      }

      FootstepPlanningToolboxOutputStatus result = new FootstepPlanningToolboxOutputStatus();
      if (footstepPlan == null)
      {
         result.getFootstepDataList().set(new FootstepDataListMessage());
      }
      else
      {
         result.getFootstepDataList().set(FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan, 0.0, 0.0, ExecutionMode.OVERRIDE));

         if (footstepPlan.hasLowLevelPlanGoal())
         {
            result.getLowLevelPlannerGoal().set(footstepPlan.getLowLevelPlanGoal());
         }
      }

      if (bodyPathPlan != null)
      {
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
            result.getBodyPath().add().set(bodyPathPlan.getWaypoint(i));
      }

      if(planarRegionsList != null)
         result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
      result.setPlanId(planId);
      result.setFootstepPlanningResult(status.toByte());
      result.getFootstepPlanningStatistics().setTimeTaken(timeTaken);
      return result;
   }
}
