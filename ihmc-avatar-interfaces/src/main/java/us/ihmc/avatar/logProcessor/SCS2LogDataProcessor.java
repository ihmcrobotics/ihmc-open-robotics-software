package us.ihmc.avatar.logProcessor;

import org.jfree.svg.SVGGraphics2D;
import org.jfree.svg.SVGUnits;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;

public class SCS2LogDataProcessor
{
   /** Square document size in meters. */
   private static final double DOCUMENT_SIZE = Double.parseDouble(System.getProperty("document.size", "15.0"));

   private Path logPath;
   private LogSession logSession;
   private YoRegistry rootRegistry;
   private SVGGraphics2D svgGraphics2D;
   private long tick = 0;
   private int numberOfEntries;
   private boolean processingLog = false;

   private final Point2D robotStartToDocumentCenter = new Point2D(DOCUMENT_SIZE / 2.0, DOCUMENT_SIZE / 2.0);

   private YoVariable yoCurrentNumberOfFootsteps;
   private YoVariable leftFootState;
   private YoVariable rightFootState;
   private ConstraintType leftLastFootState = ConstraintType.SWING;
   private ConstraintType rightLastFootState = ConstraintType.SWING;
   private Double leftFullSupportTime = Double.NaN;
   private Double rightFullSupportTime = Double.NaN;
   private boolean newLeftStep = false;
   private boolean newRightStep = false;
   private YoVariable leftFootPolygon_0_x ;
   private YoVariable leftFootPolygon_0_y ;
   private YoVariable leftFootPolygon_1_x ;
   private YoVariable leftFootPolygon_1_y ;
   private YoVariable leftFootPolygon_2_x ;
   private YoVariable leftFootPolygon_2_y ;
   private YoVariable leftFootPolygon_3_x ;
   private YoVariable leftFootPolygon_3_y ;
   private YoVariable rightFootPolygon_0_x;
   private YoVariable rightFootPolygon_0_y;
   private YoVariable rightFootPolygon_1_x;
   private YoVariable rightFootPolygon_1_y;
   private YoVariable rightFootPolygon_2_x;
   private YoVariable rightFootPolygon_2_y;
   private YoVariable rightFootPolygon_3_x;
   private YoVariable rightFootPolygon_3_y;
   private final ArrayList<double[]> leftFootsteps = new ArrayList<>();
   private final ArrayList<double[]> rightFootsteps = new ArrayList<>();

   private final Point2D currentCenterOfMass = new Point2D();
   private final Point2D lastCenterOfMass = new Point2D(Double.NaN, Double.NaN);
   private YoVariable yoCenterOfMassX;
   private YoVariable yoCenterOfMassY;
   private final double comPlotProximityToFootsteps = 5.0;
   private final double comPlotResolution = 0.1;
   private double lastCoMPlotTime = Double.NaN;
   private final RecyclingArrayList<Point2D> coms = new RecyclingArrayList<>(Point2D::new);

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
      }
   }

   public void processLogAsync()
   {
      ThreadTools.startAsDaemon(this::processLog, "SCS2LogDataProcessorThread");
   }

   private void processLog()
   {
      processingLog = true;

      try
      {
         logSession = new LogSession(logPath.toFile(), null);
         logSession.addAfterReadCallback(this::afterRead);
         rootRegistry = logSession.getRootRegistry();

         String highLevelController = "root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.";

         String momentumRateControl = highLevelController + "WalkingControllerState.LinearMomentumRateControlModule.";
         yoCenterOfMassX = rootRegistry.findVariable(momentumRateControl + "centerOfMassX");
         yoCenterOfMassY = rootRegistry.findVariable(momentumRateControl + "centerOfMassY");

         String walkingMessageHandler = highLevelController + "HighLevelHumanoidControllerFactory.WalkingMessageHandler.";
         yoCurrentNumberOfFootsteps = rootRegistry.findVariable(walkingMessageHandler + "currentNumberOfFootsteps");
         String feetManager = highLevelController + "HighLevelHumanoidControllerFactory.HighLevelControlManagerFactory.FeetManager.";
         leftFootState = rootRegistry.findVariable(feetManager + "leftFootControlModule.leftFootCurrentState");
         rightFootState = rootRegistry.findVariable(feetManager + "rightFootControlModule.rightFootCurrentState");
         String footPolygonPrefix = highLevelController + "HighLevelHumanoidControllerToolbox.BipedSupportPolygons.";
         leftFootPolygon_0_x  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_0_x");
         leftFootPolygon_0_y  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_0_y");
         leftFootPolygon_1_x  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_1_x");
         leftFootPolygon_1_y  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_1_y");
         leftFootPolygon_2_x  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_2_x");
         leftFootPolygon_2_y  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_2_y");
         leftFootPolygon_3_x  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_3_x");
         leftFootPolygon_3_y  = rootRegistry.findVariable(footPolygonPrefix + "leftFootPolygon_3_y");
         rightFootPolygon_0_x = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_0_x");
         rightFootPolygon_0_y = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_0_y");
         rightFootPolygon_1_x = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_1_x");
         rightFootPolygon_1_y = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_1_y");
         rightFootPolygon_2_x = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_2_x");
         rightFootPolygon_2_y = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_2_y");
         rightFootPolygon_3_x = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_3_x");
         rightFootPolygon_3_y = rootRegistry.findVariable(footPolygonPrefix + "rightFootPolygon_3_y");

         logSession.startSessionThread();
         MissingThreadTools.sleep(0.1);

         numberOfEntries = logSession.getLogDataReader().getNumberOfEntries();
         LogTools.info("numberOfEntries: %d".formatted(numberOfEntries));

         for (int i = 0; i < numberOfEntries; i++)
         {
            logSession.runTick();
         }

         MissingThreadTools.sleep(0.1);

         logSession.stopSessionThread();

         drawSVG();
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

   private void afterRead(double currentTime)
   {
      int currentLogPosition = logSession.getLogDataReader().getCurrentLogPosition();
      if (currentLogPosition % 10000 == 0)
         LogTools.info("(%d/%d)".formatted(currentLogPosition, numberOfEntries));

      if (leftLastFootState != ConstraintType.FULL && leftFootState.getValueAsString().equals(ConstraintType.FULL.name()))
      {
         LogTools.info("(%d/%d) Recording left footstep".formatted(currentLogPosition, numberOfEntries));
         newLeftStep = true;
         leftFullSupportTime = currentTime;
      }
      leftLastFootState = leftFootState.getValueAsString().equals("null") ? null : ConstraintType.valueOf(leftFootState.getValueAsString());

      if (rightLastFootState != ConstraintType.FULL && rightFootState.getValueAsString().equals(ConstraintType.FULL.name()))
      {
         LogTools.info("(%d/%d) Recording left footstep".formatted(currentLogPosition, numberOfEntries));
         newRightStep = true;
         rightFullSupportTime = currentTime;
      }
      rightLastFootState = rightFootState.getValueAsString().equals("null") ? null : ConstraintType.valueOf(rightFootState.getValueAsString());
      


      if (newLeftStep && currentTime - leftFullSupportTime > 0.1)
      {
         leftFootsteps.add(new double[] {leftFootPolygon_0_x.getValueAsDouble(),
                                         leftFootPolygon_1_x.getValueAsDouble(),
                                         leftFootPolygon_2_x.getValueAsDouble(),
                                         leftFootPolygon_3_x.getValueAsDouble(),
                                         leftFootPolygon_0_y.getValueAsDouble(),
                                         leftFootPolygon_1_y.getValueAsDouble(),
                                         leftFootPolygon_2_y.getValueAsDouble(),
                                         leftFootPolygon_3_y.getValueAsDouble()});
         newLeftStep = false;
      }

      if (newRightStep && currentTime - rightFullSupportTime > 0.1)
      {
         rightFootsteps.add(new double[] {rightFootPolygon_0_x.getValueAsDouble(),
                                          rightFootPolygon_1_x.getValueAsDouble(),
                                          rightFootPolygon_2_x.getValueAsDouble(),
                                          rightFootPolygon_3_x.getValueAsDouble(),
                                          rightFootPolygon_0_y.getValueAsDouble(),
                                          rightFootPolygon_1_y.getValueAsDouble(),
                                          rightFootPolygon_2_y.getValueAsDouble(),
                                          rightFootPolygon_3_y.getValueAsDouble()});
         newRightStep = false;
      }

      boolean recentLeftStep = !Double.isNaN(leftFullSupportTime) && currentTime - leftFullSupportTime < comPlotProximityToFootsteps;
      boolean recentRightStep = !Double.isNaN(rightFullSupportTime) && currentTime - rightFullSupportTime < comPlotProximityToFootsteps;

      if (recentLeftStep || recentRightStep)
      {
         if (Double.isNaN(lastCoMPlotTime) || currentTime - lastCoMPlotTime > comPlotResolution)
         {
            currentCenterOfMass.set(yoCenterOfMassX.getValueAsDouble(), yoCenterOfMassY.getValueAsDouble());

            if (coms.isEmpty())
            {
               robotStartToDocumentCenter.add(-currentCenterOfMass.getX(), currentCenterOfMass.getY());
            }

            LogTools.info("(%d/%d) Extracting CoM at %s", currentLogPosition, numberOfEntries, currentCenterOfMass);
            coms.add().set(currentCenterOfMass);

            lastCenterOfMass.set(currentCenterOfMass);
            lastCoMPlotTime = currentTime;
         }
      }

      ++tick;
   }

   private void drawSVG()
   {
      double documentSizeMillimeters = convertToMillimeters(DOCUMENT_SIZE);
      svgGraphics2D = new SVGGraphics2D(documentSizeMillimeters, documentSizeMillimeters, SVGUnits.MM);

      svgGraphics2D.setColor(Color.BLACK);
      svgGraphics2D.setStroke(new BasicStroke(15));

      for (double[] footstep : leftFootsteps)
      {
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
      for (double[] footstep : rightFootsteps)
      {
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

      Path savePath = logPath.resolve(logPath.getFileName().toString() + "_OverheadPlot.svg");
      LogTools.info("Saving to {}", savePath);

      try (FileWriter writer = new FileWriter(savePath.toFile()))
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
      return (int) convertToMillimeters(x + robotStartToDocumentCenter.getX());
   }

   private int metersToMMY(double y)
   {
      return (int) convertToMillimeters(y + robotStartToDocumentCenter.getY());
   }

   private long convertToMillimeters(double meters)
   {
      return (long) (meters * 1000.0);
   }

   public boolean isLogValid()
   {
      return logPath != null;
   }

   public boolean isProcessingLog()
   {
      return processingLog;
   }

   public static void main(String[] args)
   {
      new SCS2LogDataProcessor().processLog();
   }
}
