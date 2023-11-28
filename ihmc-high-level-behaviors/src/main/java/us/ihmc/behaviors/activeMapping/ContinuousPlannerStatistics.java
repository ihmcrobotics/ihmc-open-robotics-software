package us.ihmc.behaviors.activeMapping;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;

public class ContinuousPlannerStatistics
{
   private boolean lock = false;

   private File file;
   private HashMap<String, Float> statistics = new HashMap<>();

   public ContinuousPlannerStatistics()
   {
      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      String logFileName = dateFormat.format(new Date()) + "_" + "ContinuousPlannerLog.txt";
      FileTools.ensureDirectoryExists(Paths.get(IHMCCommonPaths.CONTINUOUS_PLANNING_DIRECTORY_NAME), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      String filePath = IHMCCommonPaths.CONTINUOUS_PLANNING_DIRECTORY.resolve(logFileName).toString();

      try
      {
         Files.createFile(Paths.get(filePath));
         file = new File(filePath);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      // convert to snake case
      statistics.put("total_length_completed", 0.0f);
      statistics.put("total_steps_completed", 0.0f);
      statistics.put("total_steps_planned", 0.0f);
      statistics.put("number_of_interventions", 0.0f);

      statistics.put("total_continuous_walking_time", 0.0f);
      statistics.put("total_planning_time", 0.0f);
      statistics.put("total_waiting_time", 0.0f);

      statistics.put("last_continuous_walking_time", 0.0f);
      statistics.put("last_planning_time", 0.0f);
      statistics.put("last_waiting_time", 0.0f);

      statistics.put("last_footstep_queue_length", 0.0f);
   }

   public void logToFile(boolean logToFile, boolean printToConsole)
   {
      if (logToFile || printToConsole)
      {
         if (printToConsole)
            System.out.println(this);

         if (logToFile)
            FileTools.write(file.getAbsoluteFile().toPath(), toString().getBytes(), WriteOption.APPEND, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      }
   }

   public void setLastPlanningTime(float lastPlanningTime)
   {
      statistics.put("last_planning_time", lastPlanningTime);
      statistics.put("total_planning_time", statistics.get("total_planning_time") + lastPlanningTime);
   }

   public void setLastFootstepQueueLength(int lastFootstepQueueLength)
   {
      statistics.put("last_footstep_queue_length", (float) lastFootstepQueueLength);
   }

   public void setLastLengthCompleted(float lastStepLength)
   {
      statistics.put("last_step_length", lastStepLength);
      statistics.put("total_length_completed", statistics.get("total_length_completed") + lastStepLength);
   }

   public void incrementTotalInterventions()
   {
      statistics.put("number_of_interventions", statistics.get("number_of_interventions") + 1.0f);
   }

   public void incrementTotalStepsCompleted()
   {
      statistics.put("total_steps_completed", statistics.get("total_steps_completed") + 1.0f);
   }

   public void startStepTime()
   {
      lock = true;
      statistics.put("last_continuous_walking_time", System.currentTimeMillis() / 1000.0f);
   }

   public void endStepTime()
   {
      float lastStepTime = System.currentTimeMillis() / 1000.0f - statistics.get("last_continuous_walking_time");
      statistics.put("last_continuous_walking_time", lastStepTime);
      statistics.put("total_continuous_walking_time", statistics.get("total_continuous_walking_time") + lastStepTime);
      lock = false;
   }

   public void startWaitingTime()
   {
      lock = true;
      statistics.put("last_waiting_time", System.currentTimeMillis() / 1000.0f);
   }

   public void endWaitingTime()
   {
      float lastWaitingTime = System.currentTimeMillis() / 1000.0f - statistics.get("last_waiting_time");
      statistics.put("last_waiting_time", lastWaitingTime);
      statistics.put("total_waiting_time", statistics.get("total_waiting_time") + lastWaitingTime);
      lock = false;
   }

   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("ContinuousPlannerStatistics: [");
      for (String key : statistics.keySet())
      {
         builder.append(key).append(":").append(statistics.get(key)).append(", ");
      }
      builder.append("]\n");
      return builder.toString();
   }
}
