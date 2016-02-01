package us.ihmc.steppr.hardware.controllers;

import java.io.IOException;

import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeMemory;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.steppr.hardware.sensorReader.StepprSensorReaderFactory;
import us.ihmc.wholeBodyController.concurrent.MultiThreadedRealTimeRobotController;

public class StepprRunner extends RealtimeThread
{
   
   private final StepprSensorReaderFactory stepprSensorReaderFactory;
   private volatile boolean requestStop = false;
   
   private final MultiThreadedRealTimeRobotController robotController;
   
   public StepprRunner(PriorityParameters priorityParameters, StepprSensorReaderFactory sensorReaderFactory, MultiThreadedRealTimeRobotController robotController)
   {
      super(priorityParameters);
      this.stepprSensorReaderFactory = sensorReaderFactory;
      this.robotController = robotController;
   }

   @Override
   public void run()
   {
      try
      {
         stepprSensorReaderFactory.connect();
      }
      catch (IOException e)
      {
         stepprSensorReaderFactory.disconnect();

         throw new RuntimeException(e);
      }

      System.gc();
      System.gc();

      RealtimeMemory.lock();

      while (!requestStop)
      {
         robotController.read();
      }

      stepprSensorReaderFactory.disconnect();
   }
}
