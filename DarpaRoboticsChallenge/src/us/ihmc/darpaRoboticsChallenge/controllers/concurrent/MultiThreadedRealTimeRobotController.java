package us.ihmc.darpaRoboticsChallenge.controllers.concurrent;

import java.util.ArrayList;

import us.ihmc.affinity.Processor;
import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.Tuple;
import us.ihmc.utilities.math.TimeTools;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

public class MultiThreadedRealTimeRobotController
{
   private final MultiThreadedRobotControlElement sensorReader;
   private final ArrayList<Tuple<MultiThreadedRobotControlElement, PriorityParameters, Processor>> robotControllers = new ArrayList<>();
   private final MonotonicTime triggerTime = new MonotonicTime();
   
   private final YoVariableRegistry registry;   //TODO: Make this properly multi-thread safe for logging
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry;
   
   private final MonotonicTime monotonicTime = new MonotonicTime();
   
   public MultiThreadedRealTimeRobotController(MultiThreadedRobotControlElement sensorReader, YoVariableRegistry registry, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
   {
      this.sensorReader = sensorReader;
      this.registry = registry;
      this.dynamicGraphicObjectsListRegistry = dynamicGraphicObjectsListRegistry;
      this.registry.addChild(sensorReader.getYoVariableRegistry());
      this.dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsListRegistry(dynamicGraphicObjectsListRegistry);
   }
   
   public void addController(MultiThreadedRobotControlElement robotController, PriorityParameters priorityParameters, Processor processor)
   {
      robotControllers.add(new Tuple<MultiThreadedRobotControlElement, PriorityParameters, Processor>(robotController, priorityParameters, processor));
      registry.addChild(robotController.getYoVariableRegistry());
      dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsListRegistry(robotController.getDynamicGraphicObjectsListRegistry());
   }
   
   public void read(double time)
   {
      long timestamp = RealtimeThread.getCurrentMonotonicClockTime();
      sensorReader.read(time, timestamp);
      sensorReader.run();
      
      // Magic jitter fix
      monotonicTime.set(0, timestamp + 500000);
      RealtimeThread.getCurrentRealtimeThread().waitUntil(monotonicTime);
      sensorReader.write(RealtimeThread.getCurrentMonotonicClockTime());
   }
   
   public void start()
   {
      for(int i = 0; i < robotControllers.size(); i++)
      {
         Tuple<MultiThreadedRobotControlElement, PriorityParameters, Processor> controllerAndPriority = robotControllers.get(i);
         MultiThreadedRobotControlElement controller = controllerAndPriority.first();
         PriorityParameters priority = controllerAndPriority.second();
         Processor processor = controllerAndPriority.third();
         
         controller.initialize();
         
         MultiThreadedRobotControlElementRunner runner = new MultiThreadedRobotControlElementRunner(controller, priority);
         if(processor != null)
         {
            runner.setAffinity(processor);            
         }
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
         while(true)
         {
            controller.read(Double.NaN, RealtimeThread.getCurrentMonotonicClockTime());
            controller.run();
            controller.write(RealtimeThread.getCurrentMonotonicClockTime());
            
            if(controller.nextWakeupTime() != Long.MIN_VALUE)
            {
               triggerTime.set(0, controller.nextWakeupTime());
               waitUntil(triggerTime);               
            }
         }
      }
      
      
   }
   
}
