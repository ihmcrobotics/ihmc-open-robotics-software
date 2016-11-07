package us.ihmc.avatar.drcRobot;

import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.time.TimeTools;

public class SimulatedDRCRobotTimeProvider extends AtomicSettableTimestampProvider implements RobotController
{
   private final long nanoSecondsPerTick;
   
   public SimulatedDRCRobotTimeProvider(double controlDT)
   {
      nanoSecondsPerTick = TimeTools.secondsToNanoSeconds(controlDT);
   }

   public void initialize()
   {
      set(0);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return new YoVariableRegistry(getName());
   }

   public String getName()
   {
      return getClass().getSimpleName();
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
      increment(nanoSecondsPerTick);
   }

}
