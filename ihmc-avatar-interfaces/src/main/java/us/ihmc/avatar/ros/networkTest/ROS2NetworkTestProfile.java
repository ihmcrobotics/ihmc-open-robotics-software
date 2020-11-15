package us.ihmc.avatar.ros.networkTest;

import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public abstract class ROS2NetworkTestProfile
{
   private final ROS2NetworkTestMachine LOCAL_MACHINE = ROS2NetworkTestMachine.getLocalMachine();

   public abstract List<ROS2NetworkTestMachine> getMachines();

   public abstract void runExperiment();

   public abstract YoRegistry getYoRegistry();

   public abstract void destroy();

   public abstract List<String[]> getGraphsToSetup();

   public void updateDerivativeVariables(YoRegistry syncedRegistry)
   {
      // not required
   }

   public List<ROS2NetworkTestMachine> getRemoteMachines()
   {
      ArrayList<ROS2NetworkTestMachine> remoteMachines = new ArrayList<>();
      for (ROS2NetworkTestMachine machine : getMachines())
      {
         if (machine != getLocalMachine())
         {
            remoteMachines.add(machine);
         }
      }
      return remoteMachines;
   }

   public ROS2NetworkTestMachine getLocalMachine()
   {
      return LOCAL_MACHINE;
   }

   public String getMachineName()
   {
      return getLocalMachine().getMachineName();
   }
}
