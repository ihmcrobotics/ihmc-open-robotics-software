package us.ihmc.avatar.logProcessor;

import org.jfree.svg.SVGGraphics2D;
import org.jfree.svg.SVGHints;
import org.jfree.svg.SVGUnits;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.tools.io.JSONFileTools;
import us.ihmc.tools.thread.MissingThreadTools;

import java.awt.Color;
import java.awt.BasicStroke;
import java.awt.Font;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
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
   private int numberOfFallsStat = -1;
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

      if (!requestStopProcessing)
      {
         writeJSON(false);
         loadStats();
         drawSVG();
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
   
   private void drawSVG()
   {
      double documentSizeMillimeters = convertToMillimeters(DOCUMENT_SIZE);
      svgGraphics2D = new SVGGraphics2D(documentSizeMillimeters, documentSizeMillimeters, SVGUnits.MM);

      svgGraphics2D.setColor(Color.BLACK);
      svgGraphics2D.setStroke(new BasicStroke(5));
      svgGraphics2D.setFont(new Font("Arial", Font.PLAIN, 20));
      svgGraphics2D.setFontSizeUnits(SVGUnits.MM);

      int walk = 1;
      for (SCS2LogWalk logWalk : locomotionData.getLogWalks())
      {
         String walkName = "Walk %d".formatted(walk);
         svgGraphics2D.setRenderingHint(SVGHints.KEY_BEGIN_GROUP, walkName);
         svgGraphics2D.drawString(walkName, metersToMMX(logWalk.getWalkStart().getX()), metersToMMY(logWalk.getWalkStart().getY() + 0.3));

         String groupName = "Footsteps %d".formatted(walk);
         svgGraphics2D.setRenderingHint(SVGHints.KEY_BEGIN_GROUP, groupName);

         for (SCS2LogDataFootstep footstep : logWalk.getFootsteps())
         {
            Color color = FootstepListVisualizer.defaultFeetColors.get(footstep.getSide());
            svgGraphics2D.setColor(color);

            double[] polygon = footstep.getPolygon();
            LogTools.info("Drawing step at {} {}", new Point2D(polygon[0], polygon[4]), new Point2D(metersToMMX(polygon[0]), metersToMMY(polygon[4])));
            svgGraphics2D.drawPolygon(new int[] {metersToMMX(polygon[0]),
                                                 metersToMMX(polygon[1]),
                                                 metersToMMX(polygon[2]),
                                                 metersToMMX(polygon[3])},
                                      new int[] {metersToMMY(polygon[4]),
                                                 metersToMMY(polygon[5]),
                                                 metersToMMY(polygon[6]),
                                                 metersToMMY(polygon[7])},
                                      4);
         }

         svgGraphics2D.setRenderingHint(SVGHints.KEY_END_GROUP, groupName);

         svgGraphics2D.setStroke(new BasicStroke(2.5f));
         svgGraphics2D.setColor(Color.BLACK);
         plot(logWalk.getComs(), "Coms");
         svgGraphics2D.setColor(Color.BLUE);
         
         svgGraphics2D.setStroke(new BasicStroke(2.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10.0f, new float[] {10.0f}, 0.0f));
         plot(logWalk.getIcps(), "ICPs");

         svgGraphics2D.setColor(Color.BLACK);
         if (logWalk.isEndedWithFall())
         {
            Point2D fallLocation;
            if (logWalk.getFootsteps().isEmpty())
            {
               fallLocation = logWalk.getWalkStart();
            }
            else
            {
               double[] lastStepPolygon = logWalk.getFootsteps().get(logWalk.getFootsteps().size() - 1).getPolygon();
               fallLocation = new Point2D(lastStepPolygon[0], lastStepPolygon[4]);
            }

            svgGraphics2D.drawString("Fall %d".formatted(walk), metersToMMX(fallLocation.getX()), metersToMMY(fallLocation.getY() + 0.3));
         }

         svgGraphics2D.setRenderingHint(SVGHints.KEY_END_GROUP, walkName);

         ++walk;
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

   private void plot(List<Point2D> points, String name)
   {
      if (!points.isEmpty())
      {
         int[] xs = new int[points.size()];
         int[] ys = new int[points.size()];
         for (int i = 0; i < points.size(); i++)
         {
            xs[i] = metersToMMX(points.get(i).getX());
            ys[i] = metersToMMY(points.get(i).getY());
         }
         svgGraphics2D.setRenderingHint(SVGHints.KEY_BEGIN_GROUP, name);
         svgGraphics2D.drawPolyline(xs, ys, xs.length);
         svgGraphics2D.setRenderingHint(SVGHints.KEY_END_GROUP, name);
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
      SCS2LogDataProcessor logDataProcessor = new SCS2LogDataProcessor();
      logDataProcessor.runLogSession(logDataProcessor::processLog);
   }
}
