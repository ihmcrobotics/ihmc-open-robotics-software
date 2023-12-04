package us.ihmc.behaviors.activeMapping;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.log.LogTools;
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
   private final File file;
   private final HashMap<String, Float> statistics = new HashMap<>();

   // These all need to be snake case in order for everything to work
   private static final String TOTAL_STEPS_COMPLETED = "total_steps_completed";
   private static final String TOTAL_DISTANCE_COMPLETED = "total_length_completed";
   private static final String TOTAL_CONTINUOUS_WALKING_TIME = "total_continuous_walking_time";
   private static final String TOTAL_WAITING_TIME = "total_waiting_time";
   private static final String TOTAL_PLANNING_TIME = "total_planning_time";
   private static final String TOTAL_STEPS_PLANNED = "total_steps_planned";
   private static final String NUMBER_OF_USER_INTERRUPTS = "number_of_interventions";

   private static final String LAST_PLANNING_TIME = "last_planning_time";
   private static final String LAST_WAITING_TIME = "last_waiting_time";
   private static final String LAST_FOOTSTEP_QUEUE_SIZE = "last_footstep_queue_size";
   private static final String LAST_CONTINUOUS_WALKING_TIME = "last_continuous_walking_time";

   public ContinuousPlannerStatistics()
   {
      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      String logFileName = dateFormat.format(new Date()) + "_" + "ContinuousPlannerLog.txt";
      FileTools.ensureDirectoryExists(Paths.get(IHMCCommonPaths.CONTINUOUS_PLANNING_DIRECTORY_NAME), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      String filePath = IHMCCommonPaths.CONTINUOUS_PLANNING_DIRECTORY.resolve(logFileName).toString();

      try
      {
         if(!Files.exists(IHMCCommonPaths.CONTINUOUS_PLANNING_DIRECTORY))
         {
            Files.createDirectory(IHMCCommonPaths.CONTINUOUS_PLANNING_DIRECTORY);
         }
         Files.createFile(Paths.get(filePath));
         file = new File(filePath);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

      statistics.put(TOTAL_DISTANCE_COMPLETED, 0.0f);
      statistics.put(TOTAL_STEPS_COMPLETED, 0.0f);
      statistics.put(TOTAL_PLANNING_TIME, 0.0f);
      statistics.put(TOTAL_WAITING_TIME, 0.0f);
      statistics.put(TOTAL_STEPS_PLANNED, 0.0f);
      statistics.put(TOTAL_CONTINUOUS_WALKING_TIME, 0.0f);

      statistics.put(LAST_PLANNING_TIME, 0.0f);
      statistics.put(LAST_WAITING_TIME, 0.0f);
      statistics.put(LAST_FOOTSTEP_QUEUE_SIZE, 0.0f);
      statistics.put(LAST_CONTINUOUS_WALKING_TIME, 0.0f);

      // TODO fix the rest of these or change them
      statistics.put(NUMBER_OF_USER_INTERRUPTS, 0.0f);
   }

   public void setLastAndTotalPlanningTimes(float lastPlanningTime)
   {
      statistics.put(LAST_PLANNING_TIME, lastPlanningTime);
      statistics.put(TOTAL_PLANNING_TIME, statistics.get(TOTAL_PLANNING_TIME) + lastPlanningTime);
   }


   private boolean startedWalking = false;
   private boolean inWaitingTime = false;
   private long startWaitingTime;
   private long startContinuousWalkingTime;


   public void setStartWaitingTime()
   {
      inWaitingTime = true;
      startWaitingTime = System.currentTimeMillis();
   }

   public void setLastAndTotalWaitingTimes()
   {
      if (inWaitingTime)
      {
         float lastWaitingTime = (System.currentTimeMillis() - startWaitingTime) / 1000.0f;
         statistics.put(LAST_WAITING_TIME, lastWaitingTime);
         statistics.put(TOTAL_WAITING_TIME, statistics.get(TOTAL_WAITING_TIME) + lastWaitingTime);
         inWaitingTime = false;
      }
   }

   public void startStepTime()
   {
      startContinuousWalkingTime = System.currentTimeMillis();
      startedWalking = true;
   }

   public void endStepTime()
   {
      if (startedWalking)
      {
         float lastStepTime = (System.currentTimeMillis() - startContinuousWalkingTime) / 1000.0f;
         statistics.put(LAST_CONTINUOUS_WALKING_TIME, lastStepTime);
         statistics.put(TOTAL_CONTINUOUS_WALKING_TIME, statistics.get(TOTAL_CONTINUOUS_WALKING_TIME) + lastStepTime);
         startedWalking = false;
      }
   }

   public void setLastFootstepQueueLength(int lastFootstepQueueLength)
   {
      statistics.put(LAST_FOOTSTEP_QUEUE_SIZE, (float) lastFootstepQueueLength);
   }

   public void setLastLengthCompleted(float lastStepLength)
   {
      statistics.put("last_step_length", lastStepLength);
      statistics.put(TOTAL_DISTANCE_COMPLETED, statistics.get(TOTAL_DISTANCE_COMPLETED) + lastStepLength);
   }

   public void setTotalStepsPlanned(int newPlannedSteps)
   {
      statistics.put(TOTAL_STEPS_PLANNED, statistics.get(TOTAL_STEPS_PLANNED) + newPlannedSteps);
   }

   public void incrementTotalInterventions()
   {
      statistics.put(NUMBER_OF_USER_INTERRUPTS, statistics.get(NUMBER_OF_USER_INTERRUPTS) + 1.0f);
   }

   public void incrementTotalStepsCompleted()
   {
      statistics.put(TOTAL_STEPS_COMPLETED, statistics.get(TOTAL_STEPS_COMPLETED) + 1.0f);
   }

   public void logToFile(boolean logToFile, boolean printToConsole)
   {
      if (logToFile || printToConsole)
      {
         if (printToConsole)
            System.out.println(this);

         if (logToFile)
         {
            LogTools.info("Logging Continuous Walking Statistics: {}", file.getAbsoluteFile().toPath());
            FileTools.write(file.getAbsoluteFile().toPath(), toString().getBytes(), WriteOption.APPEND, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         }
      }
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
