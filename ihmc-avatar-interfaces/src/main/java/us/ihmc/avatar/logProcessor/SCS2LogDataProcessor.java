package us.ihmc.avatar.logProcessor;

import org.jfree.svg.SVGGraphics2D;
import org.jfree.svg.SVGUnits;
import org.jfree.svg.SVGUtils;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.awt.*;
import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;

public class SCS2LogDataProcessor
{
   private final LogSession logSession;
   private final YoRegistry rootRegistry;
   private final SVGGraphics2D svgGraphics2D;
   private long tick = 0;
   private int numberOfEntries;

   private long lastNumberOfFoosteps = 0;
   private final YoVariable yoCurrentNumberOfFootsteps;

   private final Point2D currentCenterOfMass = new Point2D();
   private final Point2D lastCenterOfMass = new Point2D(Double.NaN, Double.NaN);
   private final YoVariable yoCenterOfMassX;
   private final YoVariable yoCenterOfMassY;

   public SCS2LogDataProcessor()
   {
      try
      {
         String logPath = System.getProperty("log.path");

         if (logPath == null)
            LogTools.error("Must pass -Dlog.path=/path/to/log");

         logSession = new LogSession(Paths.get(logPath).toFile(), null);
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

         svgGraphics2D = new SVGGraphics2D(50000, 50000, SVGUnits.MM);

         svgGraphics2D.setColor(Color.BLACK);
         svgGraphics2D.setStroke(new BasicStroke(15));

         for (int i = 0; i < numberOfEntries; i++)
         {
            logSession.runTick();
         }

         MissingThreadTools.sleep(0.1);

         logSession.stopSessionThread();

         SVGUtils.writeToSVG(new File("data.svg"), svgGraphics2D.getSVGElement());
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void afterRead(double currentTime)
   {
      //      LogTools.info("tick: %d/%d  currentTime: %f".formatted(tick, numberOfEntries, currentTime));

      long currentNumberOfFootsteps = (long) yoCurrentNumberOfFootsteps.getValueAsDouble();
      if (currentNumberOfFootsteps != lastNumberOfFoosteps)
      {
         LogTools.info("tick: %d/%d  footstep: %d -> %d".formatted(tick, numberOfEntries, lastNumberOfFoosteps, currentNumberOfFootsteps));


         currentCenterOfMass.set(yoCenterOfMassX.getValueAsDouble() * 1000.0 + 20000,
                                 yoCenterOfMassY.getValueAsDouble() * 1000.0 + 20000);


         if (!lastCenterOfMass.containsNaN())
         {



            svgGraphics2D.drawLine((int) lastCenterOfMass.getX(),
                                   (int) lastCenterOfMass.getY(),
                                   (int) currentCenterOfMass.getX(),
                                   (int) currentCenterOfMass.getY());
         }

         lastCenterOfMass.set(currentCenterOfMass);



      }

      lastNumberOfFoosteps = currentNumberOfFootsteps;


      ++tick;
   }

   private long convertToMillimeters(double meters)
   {
      return (long) (meters / 1000.0);
   }

   public static void main(String[] args)
   {
      new SCS2LogDataProcessor();
   }
}
