package us.ihmc.avatar.drcRobot;

import us.ihmc.commons.Conversions;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimulatedDRCRobotTimeProvider extends AtomicSettableTimestampProvider implements RobotController
{
   private final long nanoSecondsPerTick;
   
   public SimulatedDRCRobotTimeProvider(double controlDT)
   {
      nanoSecondsPerTick = Conversions.secondsToNanoseconds(controlDT);
   }

   public void initialize()
   {
      set(0);
   }

   public YoRegistry getYoRegistry()
   {
      return new YoRegistry(getName());
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
