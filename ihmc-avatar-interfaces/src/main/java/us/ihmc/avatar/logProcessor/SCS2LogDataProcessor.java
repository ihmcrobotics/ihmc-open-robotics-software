package us.ihmc.avatar.logProcessor;

import gnu.trove.list.array.TIntArrayList;
import org.jfree.svg.SVGGraphics2D;
import org.jfree.svg.SVGUnits;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class SCS2LogDataProcessor
{
   /** Path to the log folder containing robotData.log */
   private final String LOG_PATH = System.getProperty("log.path");
   /** Square document size in meters. */
   private final double DOCUMENT_SIZE = Double.parseDouble(System.getProperty("document.size", "15.0"));

   private final LogSession logSession;
   private final YoRegistry rootRegistry;
   private final SVGGraphics2D svgGraphics2D;
   private long tick = 0;
   private final int numberOfEntries;

   private final Point2D robotStartToDocumentCenter = new Point2D(DOCUMENT_SIZE / 2.0, DOCUMENT_SIZE / 2.0);

   private long lastNumberOfFoosteps = 0;
   private final YoVariable yoCurrentNumberOfFootsteps;
   private double lastFootstepTime = Double.NaN;

   private final Point2D currentCenterOfMass = new Point2D();
   private final Point2D lastCenterOfMass = new Point2D(Double.NaN, Double.NaN);
   private final YoVariable yoCenterOfMassX;
   private final YoVariable yoCenterOfMassY;
   private final double comPlotProximityToFootsteps = 5.0;
   private final double comPlotResolution = 0.1;
   private double lastCoMPlotTime = Double.NaN;
   private final TIntArrayList comXs = new TIntArrayList();
   private final TIntArrayList comYs = new TIntArrayList();

   public SCS2LogDataProcessor()
   {
      try
      {
         if (LOG_PATH == null)
            LogTools.error("Must pass -Dlog.path=/path/to/log");

         logSession = new LogSession(Paths.get(LOG_PATH).toFile(), null);
         logSession.addAfterReadCallback(this::afterRead);
         rootRegistry = logSession.getRootRegistry();

         yoCurrentNumberOfFootsteps = rootRegistry.findVariable(
               "root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.HighLevelHumanoidControllerFactory.WalkingMessageHandler.currentNumberOfFootsteps");
         yoCenterOfMassX = rootRegistry.findVariable(
               "root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.WalkingControllerState.LinearMomentumRateControlModule.centerOfMassX");
         yoCenterOfMassY = rootRegistry.findVariable(
               "root.main.DRCControllerThread.DRCMomentumBasedController.HumanoidHighLevelControllerManager.WalkingControllerState.LinearMomentumRateControlModule.centerOfMassY");

         logSession.startSessionThread();
         MissingThreadTools.sleep(0.1);

         numberOfEntries = logSession.getLogDataReader().getNumberOfEntries();
         LogTools.info("numberOfEntries: %d".formatted(numberOfEntries));

         double documentSizeMillimeters = convertToMillimeters(DOCUMENT_SIZE);
         svgGraphics2D = new SVGGraphics2D(documentSizeMillimeters, documentSizeMillimeters, SVGUnits.MM);

         svgGraphics2D.setColor(Color.BLACK);
         svgGraphics2D.setStroke(new BasicStroke(15));

         for (int i = 0; i < numberOfEntries; i++)
         {
            logSession.runTick();
         }

         svgGraphics2D.drawPolyline(comXs.toArray(), comYs.toArray(), comXs.size());

         MissingThreadTools.sleep(0.1);

         logSession.stopSessionThread();

         DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmssSSS");
         Path saveFile = Paths.get(LocalDateTime.now().format(formatter) + "_LogDataDrawing.svg");
         LogTools.info("Saving to {}", saveFile);

         try (FileWriter writer = new FileWriter(saveFile.toFile()))
         {
            String svgDocument = svgGraphics2D.getSVGDocument();
            // Add viewBox attribute to the SVG element, to make it load correctly in Inkscape
            svgDocument = svgDocument.replace("<svg", "<svg viewBox=\"0 0 %f %f\"".formatted(documentSizeMillimeters, documentSizeMillimeters));
            writer.write(svgDocument);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void afterRead(double currentTime)
   {
      long currentNumberOfFootsteps = (long) yoCurrentNumberOfFootsteps.getValueAsDouble();
      if (currentNumberOfFootsteps != lastNumberOfFoosteps)
      {
         LogTools.info("tick: %d/%d  footstep: %d -> %d".formatted(tick, numberOfEntries, lastNumberOfFoosteps, currentNumberOfFootsteps));
         lastFootstepTime = currentTime;
      }
      lastNumberOfFoosteps = currentNumberOfFootsteps;

      if (!Double.isNaN(lastFootstepTime) && currentTime - lastFootstepTime < comPlotProximityToFootsteps)
      {
         if (Double.isNaN(lastCoMPlotTime) || currentTime - lastCoMPlotTime > comPlotResolution)
         {
            currentCenterOfMass.set(yoCenterOfMassX.getValueAsDouble(), yoCenterOfMassY.getValueAsDouble());

            if (comXs.isEmpty())
            {
               robotStartToDocumentCenter.sub(currentCenterOfMass);
            }

            LogTools.info("Drawing CoM at {}", currentCenterOfMass);
            comXs.add((int) convertToMillimeters(currentCenterOfMass.getX() + robotStartToDocumentCenter.getX()));
            comYs.add((int) convertToMillimeters(currentCenterOfMass.getY() + robotStartToDocumentCenter.getY()));

            lastCenterOfMass.set(currentCenterOfMass);
            lastCoMPlotTime = currentTime;
         }
      }

      ++tick;
   }

   private long convertToMillimeters(double meters)
   {
      return (long) (meters * 1000.0);
   }

   public static void main(String[] args)
   {
      new SCS2LogDataProcessor();
   }
}
