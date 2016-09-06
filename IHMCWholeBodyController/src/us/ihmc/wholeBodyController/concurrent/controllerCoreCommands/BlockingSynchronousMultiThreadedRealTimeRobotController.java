package us.ihmc.wholeBodyController.concurrent.controllerCoreCommands;

import org.apache.commons.lang3.tuple.ImmutableTriple;
import us.ihmc.affinity.Processor;
import us.ihmc.realtime.PriorityParameters;
import us.ihmc.realtime.RealtimeThread;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;

import java.util.ArrayList;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class BlockingSynchronousMultiThreadedRealTimeRobotController
{
   private final MultiThreadedRobotControlElement sensorReader;
   private final ArrayList<ImmutableTriple<MultiThreadedRobotControlElement, PriorityParameters, Processor>> robotControllers = new ArrayList<>();
   //   private final MonotonicTime triggerTime = new MonotonicTime();
   //   private final MonotonicTime monotonicTime = new MonotonicTime();

   public BlockingSynchronousMultiThreadedRealTimeRobotController(MultiThreadedRobotControlElement sensorReader)
   {
      this.sensorReader = sensorReader;
   }

   public void addController(MultiThreadedRobotControlElement robotController, PriorityParameters priorityParameters, Processor processor)
   {
      robotControllers
            .add(new ImmutableTriple<MultiThreadedRobotControlElement, PriorityParameters, Processor>(robotController, priorityParameters, processor));
   }

   public void read()
   {
      long timestamp = RealtimeThread.getCurrentMonotonicClockTime();
      sensorReader.read(timestamp);
      sensorReader.run();
      sensorReader.write(RealtimeThread.getCurrentMonotonicClockTime());
   }

   public void start()
   {
      //TODO this
   }
}
