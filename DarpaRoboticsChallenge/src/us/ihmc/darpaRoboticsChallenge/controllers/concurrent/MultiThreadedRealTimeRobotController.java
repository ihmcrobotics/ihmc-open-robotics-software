package us.ihmc.darpaRoboticsChallenge.controllers.concurrent;

import java.util.ArrayList;
import java.util.Arrays;

import us.ihmc.realtime.RealtimeThread;
import us.ihmc.util.RealtimeTools;

import com.yobotics.simulationconstructionset.robotController.MultiThreadedRobotControlElement;

public class MultiThreadedRealTimeRobotController
{
   private final MultiThreadedRobotControlElement sensorReader;
   private final ArrayList<MultiThreadedRobotControlElement> robotControllers = new ArrayList<>();
   
   public MultiThreadedRealTimeRobotController(MultiThreadedRobotControlElement sensorReader, MultiThreadedRobotControlElement... robotControllers)
   {
      this.sensorReader = sensorReader;
      this.robotControllers.addAll(Arrays.asList(robotControllers));
   }
   
   public void read(double time)
   {
      long timestamp = RealtimeThread.getCurrentMonotonicClockTime();
      sensorReader.read(time, System.nanoTime());
      sensorReader.run();
      sensorReader.write();
   }
   
   public void start()
   {
      
   }
   
}
