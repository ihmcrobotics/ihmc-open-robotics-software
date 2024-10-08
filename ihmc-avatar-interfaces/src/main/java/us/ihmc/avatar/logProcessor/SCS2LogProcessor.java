package us.ihmc.avatar.logProcessor;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.thread.MissingThreadTools;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Consumer;

public class SCS2LogProcessor
{
   private Path logPath;
   private Path jsonPath;
   private Path pelvisXYCSVPath;
   private int numberOfEntries = -1;
   private int currentLogPosition;
   private boolean processingLog = false;
   private boolean requestStopProcessing = false;
   private SCS2LogLocomotionData locomotionData;

   private int numberOfWalksStat = -1;
   private int numberOfFallsStat = -1;
   private int numberOfFootstepsStat = -1;
   private int numberOfComsStat = -1;
   private int workingCounterMismatchStat = -1;

   public SCS2LogProcessor()
   {
      this(System.getProperty("log.path") == null ? null : Paths.get(System.getProperty("log.path")));

      if (logPath == null)
      {
         LogTools.error("Must pass -Dlog.path=/path/to/log");
      }
   }

   /** @param logPath Path to the log folder containing robotData.log */
   public SCS2LogProcessor(Path logPath)
   {
      this.logPath = logPath;

      if (logPath == null || !Files.exists(logPath) || !Files.exists(logPath.resolve("robotData.log")))
      {
         LogTools.error("Log path not valid: %s".formatted(logPath));
         this.logPath = null;
         return;
      }

      jsonPath = logPath.resolve("statistics.json");
      pelvisXYCSVPath = logPath.resolve(logPath.getFileName().toString() + "_PelvisXY.csv");

      loadStats();
   }

   private void runLogSession(Consumer<LogSession> logSessionConsumer)
   {
      processingLog = true;

      try
      {
         LogSession logSession = new LogSession(logPath.toFile(), null);

         logSession.startSessionThread();
         MissingThreadTools.sleep(0.1);

         numberOfEntries = logSession.getLogDataReader().getNumberOfEntries();
         LogTools.info("numberOfEntries: %d".formatted(numberOfEntries));

         logSessionConsumer.accept(logSession);

         MissingThreadTools.sleep(0.1);

         logSession.stopSessionThread();
         logSession.shutdownSession();
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      finally
      {
         processingLog = false;
      }
   }

   public void processLogAsync()
   {
      processLogAsync(this::processLog);
   }

   public void gatherStatsAsync()
   {
      processLogAsync(logSession -> writeJSON(true));
   }

   public void processLogAsync(Consumer<LogSession> logSessionConsumer)
   {
      ThreadTools.startAsDaemon(() -> runLogSession(logSessionConsumer), "SCS2LogDataProcessorThread");
   }

   private void processLog(LogSession logSession)
   {
      locomotionData = new SCS2LogLocomotionData();
      locomotionData.setup(logSession);

      requestStopProcessing = false;
      for (int i = 0; i < numberOfEntries && !requestStopProcessing; i++)
      {
         currentLogPosition = logSession.getLogDataReader().getCurrentLogPosition();
         logSession.runTick();
      }

      locomotionData.requestStopProcessing();

      if (!requestStopProcessing)
      {
         writeJSON(false);
         loadStats();
         new SCS2LogOverheadSVGPlot(logPath, locomotionData).drawSVG();

         try (BufferedWriter writer = Files.newBufferedWriter(pelvisXYCSVPath))
         {
            writer.write("Time,X,Y"); // header
            writer.newLine();
            for (SCS2LogWalk logWalk : locomotionData.getLogWalks())
            {
               for (int i = 0; i < logWalk.getTimes().size(); i++)
               {
                  writer.write("%s,%s,%s".formatted(logWalk.getTimes().get(i),
                                                    logWalk.getPelvisPoses().get(i).getX(),
                                                    logWalk.getPelvisPoses().get(i).getY()));
                  writer.newLine();
               }
            }
         }
         catch (IOException e)
         {
            LogTools.error("Failed to write to CSV file.", e);
         }
      }

      locomotionData = null;
   }

   private void writeJSON(boolean numEntriesOnly)
   {
      LogTools.info("Saving JSON to {}", jsonPath);
      JSONFileTools.save(jsonPath, rootNode ->
      {
         rootNode.put("numberOfEntries", numberOfEntries);
         if (!numEntriesOnly)
         {
            locomotionData.writeJSON(rootNode);
         }
      });
   }

   private void loadStats()
   {
      LogTools.info("Loading JSON stats from {}", jsonPath);
      if (Files.exists(jsonPath))
      {
         JSONFileTools.load(jsonPath, rootNode ->
         {
            numberOfEntries = rootNode.get("numberOfEntries").intValue();
            if (rootNode.has("numberOfWalks"))
               numberOfWalksStat = rootNode.get("numberOfWalks").intValue();
            if (rootNode.has("numberOfFalls"))
               numberOfFallsStat = rootNode.get("numberOfFalls").intValue();
            if (rootNode.has("numberOfFootsteps"))
               numberOfFootstepsStat = rootNode.get("numberOfFootsteps").intValue();
            if (rootNode.has("numberOfComs"))
               numberOfComsStat = rootNode.get("numberOfComs").intValue();
            if (rootNode.has("workingCounterMismatch"))
               workingCounterMismatchStat = rootNode.get("workingCounterMismatch").intValue();
         });
      }
   }
   
   public void stopProcessing()
   {
      requestStopProcessing = true;
   }

   public int getLogCurrentTick()
   {
      return currentLogPosition;
   }

   public int getNumberOfEntries()
   {
      return numberOfEntries;
   }

   public boolean isLogValid()
   {
      return logPath != null;
   }

   public boolean isProcessingLog()
   {
      return processingLog;
   }

   public int getNumberOfWalksStat()
   {
      return locomotionData == null ? numberOfWalksStat : locomotionData.getLogWalks().size();
   }

   public int getNumberOfFallsStat()
   {
      return locomotionData == null ? numberOfFallsStat : locomotionData.getFalls();
   }

   public int getNumberOfFootstepsStat()
   {
      if (locomotionData == null)
         return numberOfFootstepsStat;
      else
         return locomotionData.getNumberOfFootsteps();
   }

   public int getNumberOfComsStat()
   {
      return locomotionData == null ? numberOfComsStat : locomotionData.getNumberOfComs();
   }

   public int getWorkingCounterMismatchStat()
   {
      return locomotionData == null ? workingCounterMismatchStat : locomotionData.getWorkingCounterMismatch();
   }

   public static void main(String[] args)
   {
      SCS2LogProcessor logDataProcessor = new SCS2LogProcessor();
      logDataProcessor.runLogSession(logDataProcessor::processLog);
   }
}
