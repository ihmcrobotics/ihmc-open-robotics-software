package us.ihmc;

import org.apache.logging.log4j.core.util.ExecutorServices;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.sensors.ImageSensorGrabber;
import us.ihmc.sensors.ImageSensorRunner;
import us.ihmc.sensors.ZEDImageGrabber;
import us.ihmc.sensors.ZEDModelData;
import us.ihmc.tools.thread.Throttler;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

public class PerceptionAutonomyManager implements AutoCloseable
{
   private static final double DEMAND_CHECK_PERIOD = 0.5;      // Check the demand status of ROS2DemandGraphNodes every 0.5 seconds
   private static final double SECONDS_BETWEEN_RETRIES = 1.0;  // Wait 1 second between retries for running sensors

   private final ExecutorService executor = Executors.newCachedThreadPool();
   private final AtomicBoolean running = new AtomicBoolean(true);

   public ImageSensorGrabber createZEDGrabber(int zedID, ZEDModelData zedModel, Supplier<ReferenceFrame> zedFrameSupplier, boolean simulate)
   {
      // TODO: Add simulation option
      return new ZEDImageGrabber(zedID, zedModel, zedFrameSupplier);
   }

   public ImageSensorRunner createSensorRunner(ImageSensorGrabber sensorGrabber)
   {
      ImageSensorRunner sensorRunner = new ImageSensorRunner(sensorGrabber);
      executor.submit(() -> tryToStartSensor(sensorRunner, null));
      return sensorRunner;
   }

   public ImageSensorRunner createSensorRunner(ImageSensorGrabber sensorGrabber, ROS2DemandGraphNode sensorDemandNode)
   {
      ImageSensorRunner sensorRunner = new ImageSensorRunner(sensorGrabber);
      executor.submit(() -> startStopSensorOnDemand(sensorRunner, sensorDemandNode));
      return sensorRunner;
   }

   @Override
   public void close() throws Exception
   {
      System.out.println("Closing " + getClass().getSimpleName());

      running.set(false);
      ExecutorServices.shutdown(executor, 2L, TimeUnit.SECONDS, getClass().getSimpleName());

      System.out.println("Closed " + getClass().getSimpleName());
   }

   private void tryToStartSensor(ImageSensorRunner sensorRunner, ROS2DemandGraphNode sensorDemandNode)
   {
      while (!sensorRunner.start() && (sensorDemandNode == null || sensorDemandNode.isDemanded()) && running.get())
      {
         ThreadTools.sleepSeconds(SECONDS_BETWEEN_RETRIES);
         LogTools.info("Retrying to run {}...", sensorRunner.getImageGrabber().getSensorName());
      }

      if (sensorRunner.isRunning())
         LogTools.info("Running {}", sensorRunner.getImageGrabber().getSensorName());
   }

   private void startStopSensorOnDemand(ImageSensorRunner sensorRunner, ROS2DemandGraphNode sensorDemandNode)
   {
      Throttler updateThrottler = new Throttler();
      updateThrottler.setPeriod(DEMAND_CHECK_PERIOD);

      while (running.get())
      {
         if (sensorDemandNode.isDemanded())
         {
            if (!sensorRunner.isRunning())
               tryToStartSensor(sensorRunner, sensorDemandNode);
         }
         else
            sensorRunner.stop();

         updateThrottler.waitAndRun();
      }
   }
}
