package us.ihmc.humanoidBehaviors.tools.footstepPlanner;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class RemoteFootstepPlannerResult
{
   private FootstepPlanningToolboxOutputStatus rawResult;
   private FootstepPlanningResult result;
   private FootstepDataListMessage footstepDataListMessage;
   private FootstepPlan footstepPlan;
   private PlanarRegionsList planarRegionsList;

   public RemoteFootstepPlannerResult()
   {
      // empty constructor
   }

   public RemoteFootstepPlannerResult(FootstepPlanningToolboxOutputStatus rawResult)
   {
      setRawResult(rawResult);
   }

   public boolean isValidForExecution()
   {
      LogTools.debug("Result: {} valid: {}", rawResult.getFootstepPlanningResult(), result.validForExecution());
      return rawResult != null && result.validForExecution();
   }

   public FootstepDataListMessage getFootstepDataListMessage()
   {
      return footstepDataListMessage;
   }

   public FootstepPlan getFootstepPlan()
   {
      return footstepPlan;
   }

   public PlanarRegionsList getPlanarRegionsList()
   {
      return planarRegionsList;
   }

   public FootstepPlanningResult getResult()
   {
      return result;
   }

   public void setRawResult(FootstepPlanningToolboxOutputStatus rawResult)
   {
      this.rawResult = rawResult;
      result = FootstepPlanningResult.fromByte(rawResult.getFootstepPlanningResult());
      footstepDataListMessage = rawResult.getFootstepDataList();
      footstepPlan = FootstepDataMessageConverter.convertToFootstepPlan(rawResult.getFootstepDataList());
      planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(rawResult.getPlanarRegionsList());
   }

   public FootstepPlanningToolboxOutputStatus getMessage()
   {
      return rawResult;
   }
}
