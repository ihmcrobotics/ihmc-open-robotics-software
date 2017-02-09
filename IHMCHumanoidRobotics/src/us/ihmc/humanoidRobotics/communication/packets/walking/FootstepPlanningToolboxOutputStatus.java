package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;
import java.util.Random;

import us.ihmc.communication.packets.StatusPacket;
import us.ihmc.footstepPlanning.FootstepPlanningResult;

public class FootstepPlanningToolboxOutputStatus extends StatusPacket<FootstepPlanningToolboxOutputStatus>
{
   public FootstepDataListMessage footstepDataList = new FootstepDataListMessage();
   public FootstepPlanningResult planningResult;

   public FootstepPlanningToolboxOutputStatus()
   {
      // empty constructor for serialization
   }

   public FootstepPlanningToolboxOutputStatus(Random random)
   {
      footstepDataList = new FootstepDataListMessage(random);
      int result = random.nextInt(FootstepPlanningResult.values.length);
      planningResult = FootstepPlanningResult.values[result];
   }

   @Override
   public boolean epsilonEquals(FootstepPlanningToolboxOutputStatus other, double epsilon)
   {
      if (!planningResult.equals(other.planningResult))
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
      footstepDataList.defaultSwingTime = other.footstepDataList.defaultSwingTime;
      footstepDataList.defaultTransferTime = other.footstepDataList.defaultTransferTime;
      footstepDataList.uniqueId = other.footstepDataList.uniqueId;
   }

}
