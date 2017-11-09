package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.SettablePacket;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;

public class FootstepPlanningToolboxOutputStatus extends SettablePacket<FootstepPlanningToolboxOutputStatus>
{
   public FootstepDataListMessage footstepDataList = new FootstepDataListMessage();
   public FootstepPlanningResult planningResult;
   public int planId = FootstepPlanningRequestPacket.NO_PLAN_ID;

   public PlanarRegionsListMessage planarRegionsListMessage = null;

   public FootstepPlanningToolboxOutputStatus()
   {
      // empty constructor for serialization
   }

   public FootstepPlanningToolboxOutputStatus(Random random)
   {
      footstepDataList = new FootstepDataListMessage(random);
      int result = random.nextInt(FootstepPlanningResult.values.length);
      planningResult = FootstepPlanningResult.values[result];
      planId = random.nextInt();
   }

   public void setPlanId(int planId)
   {
      this.planId = planId;
   }

   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
   }

   @Override
   public boolean epsilonEquals(FootstepPlanningToolboxOutputStatus other, double epsilon)
   {
      if (!planningResult.equals(other.planningResult))
         return false;
      if(planId != other.planId)
         return false;

      if(planarRegionsListMessage == null && other.planarRegionsListMessage != null)
         return false;
      else if(planarRegionsListMessage != null && other.planarRegionsListMessage == null)
         return false;
      else if(!planarRegionsListMessage.epsilonEquals(other.planarRegionsListMessage, epsilon))
         return false;

      return footstepDataList.epsilonEquals(other.footstepDataList, epsilon);
   }

   @Override
   public void set(FootstepPlanningToolboxOutputStatus other)
   {
      planningResult = other.planningResult;
      footstepDataList.destination = other.footstepDataList.destination;
      footstepDataList.executionMode = other.footstepDataList.executionMode;
      footstepDataList.footstepDataList = new ArrayList<>();
      for (FootstepDataMessage footstepData : other.footstepDataList)
         footstepDataList.footstepDataList.add(new FootstepDataMessage(footstepData));
      if (other.notes != null)
         footstepDataList.notes = new String(other.notes);
      footstepDataList.defaultSwingDuration = other.footstepDataList.defaultSwingDuration;
      footstepDataList.defaultTransferDuration = other.footstepDataList.defaultTransferDuration;
      footstepDataList.uniqueId = other.footstepDataList.uniqueId;
      planarRegionsListMessage = other.planarRegionsListMessage;
   }

}
