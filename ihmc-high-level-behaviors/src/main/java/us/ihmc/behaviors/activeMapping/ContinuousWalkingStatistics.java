package us.ihmc.behaviors.activeMapping;

import java.util.HashMap;

public class ContinuousWalkingStatistics
{
   private HashMap<String, Float> statistics = new HashMap<>();

   public ContinuousWalkingStatistics()
   {
      statistics.put("totalContinuousWalkingTime", 0.0f);
      statistics.put("totalPlanningTime", 0.0f);
      statistics.put("totalWaitingTime", 0.0f);
      statistics.put("totalLengthCompleted", 0.0f);
      statistics.put("totalStepsCompleted", 0.0f);
      statistics.put("totalStepsPlanned", 0.0f);
      statistics.put("numberOfInterventions", 0.0f);

      statistics.put("lastContinuousWalkingTime", 0.0f);
      statistics.put("lastPlanningTime", 0.0f);
      statistics.put("lastWaitingTime", 0.0f);
      statistics.put("lastFootstepQueueLength", 0.0f);
   }

   public void setLastContinuousWalkingTime(float lastContinuousWalkingTime)
   {
      statistics.put("lastContinuousWalkingTime", lastContinuousWalkingTime);
      statistics.put("totalContinuousWalkingTime", statistics.get("totalContinuousWalkingTime") + lastContinuousWalkingTime);
   }

   public void setLastPlanningTime(float lastPlanningTime)
   {
      statistics.put("lastPlanningTime", lastPlanningTime);
      statistics.put("totalPlanningTime", statistics.get("totalPlanningTime") + lastPlanningTime);
   }

   public void setLastWaitingTime(float lastWaitingTime)
   {
      statistics.put("lastWaitingTime", lastWaitingTime);
      statistics.put("totalWaitingTime", statistics.get("totalWaitingTime") + lastWaitingTime);
   }
}
