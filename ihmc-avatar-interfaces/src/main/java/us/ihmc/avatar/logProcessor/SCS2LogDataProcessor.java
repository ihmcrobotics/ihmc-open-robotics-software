package us.ihmc.avatar.logProcessor;

import org.jfree.svg.SVGGraphics2D;
import org.jfree.svg.SVGUnits;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.thread.MissingThreadTools;

import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Consumer;

public class SCS2LogDataProcessor
{
   /** Square document size in meters. */
   private static final double DOCUMENT_SIZE = Double.parseDouble(System.getProperty("document.size", "15.0"));

   private Path logPath;
   private Path jsonPath;
   private Path svgPath;
   private SVGGraphics2D svgGraphics2D;
   private int numberOfEntries = -1;
   private int currentLogPosition;
   private boolean processingLog = false;
   private boolean requestStopProcessing = false;
   private SCS2LogLocomotionData locomotionData;

   private int numberOfWalksStat = -1;
   private int numberOfFootstepsStat = -1;
   private int numberOfComsStat = -1;
   private int workingCounterMismatchStat = -1;

   public SCS2LogDataProcessor()
   {
      this(System.getProperty("log.path") == null ? null : Paths.get(System.getProperty("log.path")));

      if (logPath == null)
      {
         LogTools.error("Must pass -Dlog.path=/path/to/log");
      }
   }

   /** @param logPath Path to the log folder containing robotData.log */
   public SCS2LogDataProcessor(Path logPath)
   {
      this.logPath = logPath;

      if (logPath == null || !Files.exists(logPath) || !Files.exists(logPath.resolve("robotData.log")))
      {
         LogTools.error("Log path not valid: %s".formatted(logPath));
         this.logPath = null;
         return;
      }

      jsonPath = logPath.resolve("statistics.json");
      svgPath = logPath.resolve(logPath.getFileName().toString() + "_OverheadPlot.svg");

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

      writeJSON(false);
      drawSVG();

      locomotionData = null;
   }

   private void writeJSON(boolean statsOnly)
   {
      LogTools.info("Saving JSON to {}", jsonPath);
      JSONFileTools.save(jsonPath, rootNode ->
      {
         rootNode.put("numberOfEntries", numberOfEntries);
         if (!statsOnly)
         {
            numberOfWalksStat = locomotionData.getWalks();
            rootNode.put("numberOfWalks", numberOfWalksStat);
            numberOfFootstepsStat = 0;
            for (RobotSide side : RobotSide.values)
            {
               numberOfFootstepsStat += locomotionData.getFootStates().get(side).getFootsteps().size();
            }
            rootNode.put("numberOfFootsteps", numberOfFootstepsStat);
            numberOfComsStat = locomotionData.getComs().size();
            rootNode.put("numberOfComs", numberOfComsStat);
            workingCounterMismatchStat = locomotionData.getWorkingCounterMismatch();
            rootNode.put("workingCounterMismatch", workingCounterMismatchStat);
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
            numberOfWalksStat = rootNode.get("numberOfWalks").intValue();
            if (rootNode.has("numberOfFootsteps"))
               numberOfFootstepsStat = rootNode.get("numberOfFootsteps").intValue();
            if (rootNode.has("numberOfComs"))
               numberOfComsStat = rootNode.get("numberOfComs").intValue();
            if (rootNode.has("workingCounterMismatch"))
               workingCounterMismatchStat = rootNode.get("workingCounterMismatch").intValue();
         });
      }
   }
   
   private void drawSVG()
   {
      double documentSizeMillimeters = convertToMillimeters(DOCUMENT_SIZE);
      svgGraphics2D = new SVGGraphics2D(documentSizeMillimeters, documentSizeMillimeters, SVGUnits.MM);

      svgGraphics2D.setColor(Color.BLACK);
      svgGraphics2D.setStroke(new BasicStroke(15));

      for (RobotSide side : RobotSide.values)
      {
         for (double[] footstep : locomotionData.getFootStates().get(side).getFootsteps())
         {
            LogTools.info("Drawing step at {} {}", new Point2D(footstep[0], footstep[4]), new Point2D(metersToMMX(footstep[0]), metersToMMY(footstep[4])));
            svgGraphics2D.drawPolygon(new int[] {metersToMMX(footstep[0]),
                                                 metersToMMX(footstep[1]),
                                                 metersToMMX(footstep[2]),
                                                 metersToMMX(footstep[3])},
                                      new int[] {metersToMMY(footstep[4]),
                                                 metersToMMY(footstep[5]),
                                                 metersToMMY(footstep[6]),
                                                 metersToMMY(footstep[7])},
                                      4);
         }
      }

      RecyclingArrayList<Point2D> coms = locomotionData.getComs();
      if (!coms.isEmpty())
      {
         int[] comXs = new int[coms.size()];
         int[] comYs = new int[coms.size()];
         for (int i = 0; i < coms.size(); i++)
         {
            comXs[i] = metersToMMX(coms.get(i).getX());
            comYs[i] = metersToMMY(coms.get(i).getY());
         }
         svgGraphics2D.drawPolyline(comXs, comYs, comXs.length);
      }

      LogTools.info("Saving SVG to {}", svgPath);

      try (FileWriter writer = new FileWriter(svgPath.toFile()))
      {
         String svgDocument = svgGraphics2D.getSVGDocument();
         // Add viewBox attribute to the SVG element, to make it load correctly in Inkscape
         svgDocument = svgDocument.replace("<svg", "<svg viewBox=\"0 0 %f %f\"".formatted(documentSizeMillimeters, documentSizeMillimeters));
         writer.write(svgDocument);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private int metersToMMX(double x)
   {
      double fromStart = x - locomotionData.getRobotStartLocation().getX();
      int halfDocumentMM = (int) convertToMillimeters(DOCUMENT_SIZE / 2.0);
      int mmFromStart = (int) convertToMillimeters(fromStart);
      return mmFromStart + halfDocumentMM;
   }

   private int metersToMMY(double y)
   {
      double fromStart = y - locomotionData.getRobotStartLocation().getY();
      int halfDocumentMM = (int) convertToMillimeters(DOCUMENT_SIZE / 2.0);
      int mmFromStart = (int) -convertToMillimeters(fromStart);
      return mmFromStart + halfDocumentMM;
   }

   private long convertToMillimeters(double meters)
   {
      return (long) (meters * 1000.0);
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
      return numberOfWalksStat;
   }

   public int getNumberOfFootstepsStat()
   {
      if (locomotionData == null || locomotionData.getFootStates().size() < 2)
         return numberOfFootstepsStat;
      else
      {
         int steps = 0;
         for (RobotSide side : RobotSide.values)
         {
            steps += locomotionData.getFootStates().get(side).getFootsteps().size();
         }
         return steps;
      }
   }

   public int getNumberOfComsStat()
   {
      return locomotionData == null ? numberOfComsStat : locomotionData.getComs().size();
   }

   public int getWorkingCounterMismatchStat()
   {
      return locomotionData == null ? workingCounterMismatchStat : locomotionData.getWorkingCounterMismatch();
   }

   public static void main(String[] args)
   {
      SCS2LogDataProcessor logDataProcessor = new SCS2LogDataProcessor();
      logDataProcessor.runLogSession(logDataProcessor::processLog);
   }
}
