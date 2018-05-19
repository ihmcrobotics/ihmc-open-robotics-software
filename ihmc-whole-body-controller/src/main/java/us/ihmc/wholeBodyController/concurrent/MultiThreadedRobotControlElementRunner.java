package us.ihmc.wholeBodyController.concurrent;

import us.ihmc.realtime.MonotonicTime;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;

public class MultiThreadedRobotControlElementRunner extends RealtimeThread
{
   private final MonotonicTime triggerTime = new MonotonicTime();   
   private final MonotonicTime defaultTriggerTime = new MonotonicTime(0, 100000);
   private final MultiThreadedRobotControlElement controller;
   
   public MultiThreadedRobotControlElementRunner(MultiThreadedRobotControlElement controller, PriorityParameters priorityParameters)
   {
      super(priorityParameters, controller.getName());
      this.controller = controller;
   }
   
   public void run()
   {
      while(true)
      {
         controller.read(RealtimeThread.getCurrentMonotonicClockTime());
         controller.run();
         long currentTime = RealtimeThread.getCurrentMonotonicClockTime();
         controller.write(currentTime);
         
         // Checks if there is a non-zero sleep duration. If not, sleep for 0.1ms. This avoids busy waits and preempting kernel tasks.
         if(controller.nextWakeupTime() != Long.MIN_VALUE && controller.nextWakeupTime() >= currentTime)
         {
            triggerTime.set(0, controller.nextWakeupTime());
         }
         else
         {
            triggerTime.setToCurrentTime();
            triggerTime.add(defaultTriggerTime);
         }
         waitUntil(triggerTime);               
      }
   }
}
