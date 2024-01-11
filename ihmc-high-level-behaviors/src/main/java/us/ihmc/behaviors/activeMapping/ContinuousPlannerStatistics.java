package us.ihmc.behaviors.activeMapping;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.tools.PerceptionLoggingTools;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.time.Instant;
import java.util.Date;
import java.util.HashMap;

public class ContinuousPlannerStatistics
{
   private File file;
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

   private boolean startedWalking = false;
   private boolean inWaitingTime = false;
   private long startWaitingTime;
   private long startContinuousWalkingTime;

   private final PerceptionDataLogger perceptionDataLogger = new PerceptionDataLogger();

   StringBuilder additionalString = new StringBuilder();

   public ContinuousPlannerStatistics()
   {
      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      String logFileName = dateFormat.format(new Date()) + "_" + "ContinuousPlannerLog.txt";
      FileTools.ensureDirectoryExists(Paths.get(IHMCCommonPaths.CONTINUOUS_PLANNING_DIRECTORY_NAME), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      String filePath = IHMCCommonPaths.CONTINUOUS_PLANNING_DIRECTORY.resolve(logFileName).toString();

//      perceptionDataLogger.openLogFile(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve(logFileName).toString());
//      perceptionDataLogger.addLongChannel(PerceptionLoggerConstants.L515_SENSOR_TIME, 1, PerceptionLoggerConstants.DEFAULT_BLOCK_SIZE);
//      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.L515_SENSOR_POSITION, 3, PerceptionLoggerConstants.DEFAULT_BLOCK_SIZE);
//      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, 4, PerceptionLoggerConstants.DEFAULT_BLOCK_SIZE);
//      perceptionDataLogger.addImageChannel(PerceptionLoggerConstants.L515_DEPTH_NAME);

      try
      {
         if(!Files.exists(IHMCCommonPaths.CONTINUOUS_PLANNING_DIRECTORY))
         {
            Files.createDirectory(IHMCCommonPaths.CONTINUOUS_PLANNING_DIRECTORY);
         }
         if (!Files.exists(IHMCCommonPaths.TERRAIN_MAP_DIRECTORY.resolve(logFileName)))
         {
            Files.createFile(Paths.get(filePath));
            file = new File(filePath);
         }
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

         additionalString.setLength(0);
      }
   }

   public void logHeightMap(Mat heightMap, RigidBodyTransform sensorToWorldTransform)
   {
      Instant acquisitionTime = Instant.now();
      long timestamp = Conversions.secondsToNanoseconds(acquisitionTime.getEpochSecond()) + acquisitionTime.getNano();

      perceptionDataLogger.storeLongs(PerceptionLoggerConstants.L515_SENSOR_TIME, timestamp);
      perceptionDataLogger.storeFloats(PerceptionLoggerConstants.L515_SENSOR_POSITION, new Point3D(sensorToWorldTransform.getTranslation()));
      perceptionDataLogger.storeFloats(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, new Quaternion(sensorToWorldTransform.getRotation()));
      PerceptionLoggingTools.logHeightMap(perceptionDataLogger, heightMap, PerceptionLoggerConstants.CROPPED_HEIGHT_MAP_NAME);
   }

   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("ContinuousPlannerStatistics: [");

      builder.append("continuous_walking_speed").append(":").append(statistics.get(TOTAL_DISTANCE_COMPLETED) / statistics.get(TOTAL_CONTINUOUS_WALKING_TIME));
      for (String key : statistics.keySet())
      {
         builder.append(key).append(":").append(statistics.get(key)).append(", ");
      }

      builder.append("]\n");
      LogTools.warn("Additional String: {}", additionalString.toString());
      builder.append(additionalString.toString()).append("\n");

      return builder.toString();
   }

   public void appendString(String string)
   {
      LogTools.warn("Additional String: {}", string);
      additionalString.append(String.format("[%s]: (", new SimpleDateFormat("HH:mm:ss.SSS").format(new Date())));
      additionalString.append(string);
      additionalString.append(")\n");
   }
}
