package us.ihmc.darpaRoboticsChallenge.controllers.concurrent;

import java.util.ArrayList;

import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.utilities.Pair;

import com.yobotics.simulationconstructionset.robotController.MultiThreadedRobotControlElement;

public class MultiThreadedRealTimeRobotController
{
   private final MultiThreadedRobotControlElement sensorReader;
   private final ArrayList<Pair<MultiThreadedRobotControlElement, PriorityParameters>> robotControllers = new ArrayList<>();
   
   public MultiThreadedRealTimeRobotController(MultiThreadedRobotControlElement sensorReader)
   {
      this.sensorReader = sensorReader;
   }
   
   public void addController(MultiThreadedRobotControlElement robotController, PriorityParameters priorityParameters)
   {
      robotControllers.add(new Pair<MultiThreadedRobotControlElement, PriorityParameters>(robotController, priorityParameters));
   }
   
   public void read(double time)
   {
      long timestamp = RealtimeThread.getCurrentMonotonicClockTime();
      sensorReader.read(time, timestamp);
      sensorReader.run();
      sensorReader.write(System.nanoTime());
   }
   
   public void start()
   {
      for(int i = 0; i < robotControllers.size(); i++)
      {
         Pair<MultiThreadedRobotControlElement, PriorityParameters> controllerAndPriority = robotControllers.get(i);
         MultiThreadedRobotControlElement controller = controllerAndPriority.first();
         PriorityParameters priority = controllerAndPriority.second();
         
         controller.initialize();
         
         MultiThreadedRobotControlElementRunner runner = new MultiThreadedRobotControlElementRunner(controller, priority);
         runner.start();
      }
   }
   
   
   private class MultiThreadedRobotControlElementRunner extends RealtimeThread
   {
      private final MultiThreadedRobotControlElement controller;
      
      public MultiThreadedRobotControlElementRunner(MultiThreadedRobotControlElement controller, PriorityParameters priorityParameters)
      {
         super(priorityParameters);
         this.controller = controller;
      }
      
      @Override
      public void run()
      {
         controller.read(Double.NaN, Long.MAX_VALUE);
         controller.run();
         controller.write(System.nanoTime());
      }
      
      
   }
   
}
