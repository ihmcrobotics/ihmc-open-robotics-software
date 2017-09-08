package us.ihmc.wanderer.hardware.controllers;

import java.io.IOException;

import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeMemory;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.wanderer.hardware.sensorReader.WandererSensorReaderFactory;
import us.ihmc.wholeBodyController.concurrent.MultiThreadedRealTimeRobotController;

public class WandererRunner extends RealtimeThread
{
   
   private final WandererSensorReaderFactory wandererSensorReaderFactory;
   private volatile boolean requestStop = false;
   
   private final MultiThreadedRealTimeRobotController robotController;
   
   public WandererRunner(PriorityParameters priorityParameters, WandererSensorReaderFactory sensorReaderFactory, MultiThreadedRealTimeRobotController robotController)
   {
      super(priorityParameters);
      this.wandererSensorReaderFactory = sensorReaderFactory;
      this.robotController = robotController;
   }

   @Override
   public void run()
   {
      try
      {
         wandererSensorReaderFactory.connect();
      }
      catch (IOException e)
      {
         wandererSensorReaderFactory.disconnect();

         throw new RuntimeException(e);
      }

      System.gc();
      System.gc();

      RealtimeMemory.lock();

      while (!requestStop)
      {
         robotController.read();
      }

      wandererSensorReaderFactory.disconnect();
   }
}
