package us.ihmc.wholeBodyController.concurrent;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import us.ihmc.affinity.Processor;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;

public class MultiThreadedRealTimeRobotController implements MultiThreadedRobotControlElementCoordinator
{
   private final MultiThreadedRobotControlElement sensorReader;
   private final ArrayList<ImmutableTriple<MultiThreadedRobotControlElement, PriorityParameters, Processor>> robotControllers = new ArrayList<>();
   
   public MultiThreadedRealTimeRobotController(MultiThreadedRobotControlElement sensorReader)
   {
      this.sensorReader = sensorReader;
   }
   
   public void addController(MultiThreadedRobotControlElement robotController, PriorityParameters priorityParameters, Processor processor)
   {
      robotControllers.add(new ImmutableTriple<MultiThreadedRobotControlElement, PriorityParameters, Processor>(robotController, priorityParameters, processor));
   }
   
   @Override
   public void read()
   {
      long timestamp = RealtimeThread.getCurrentMonotonicClockTime();
      sensorReader.read(timestamp);
      sensorReader.run();
      
      // Magic jitter fix
//      monotonicTime.set(0, timestamp + 500000);
//      RealtimeThread.getCurrentRealtimeThread().waitUntil(monotonicTime);
      sensorReader.write(RealtimeThread.getCurrentMonotonicClockTime());
   }
   
   @Override
   public void start()
   {
      for(int i = 0; i < robotControllers.size(); i++)
      {
         ImmutableTriple<MultiThreadedRobotControlElement, PriorityParameters, Processor> controllerAndPriority = robotControllers.get(i);
         MultiThreadedRobotControlElement controller = controllerAndPriority.getLeft();
         PriorityParameters priority = controllerAndPriority.getMiddle();
         Processor processor = controllerAndPriority.getRight();
         
         controller.initialize();
         
         MultiThreadedRobotControlElementRunner runner = new MultiThreadedRobotControlElementRunner(controller, priority);
         if(processor != null)
         {
            runner.setAffinity(processor);            
         }
         runner.start();
      }
   }
}
