package us.ihmc.simulationconstructionset.robotController;

import us.ihmc.robotics.dataStructures.listener.RewoundListener;
import us.ihmc.simulationconstructionset.PlayCycleListener;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class MultiThreadedRobotController extends AbstractThreadedRobotController implements RewoundListener, PlayCycleListener
{
   private final SimulationConstructionSet scs;

   public MultiThreadedRobotController(String name, Robot simulatedRobot, SimulationConstructionSet scs)
   {
      super(name, simulatedRobot);
      this.scs = scs;
      scs.attachSimulationRewoundListener(this);
      scs.attachPlayCycleListener(this);
   }

   @Override
   public void addController(MultiThreadedRobotControlElement controller, int executionsPerControlTick, boolean skipFirstControlCycle)
   {
      controllers.add(new MultiThreadedRobotControllerExecutor(simulatedRobot, controller, executionsPerControlTick, skipFirstControlCycle, registry));
      if(controller.getYoGraphicsListRegistry() != null)
      {
         scs.addYoGraphicsListRegistry(controller.getYoGraphicsListRegistry(), false);
      }
   }

   @Override
   public void wasRewound()
   {
      updateDynamicGraphicObjectListRegistries();
//      long lcm;
//      if (controllers.size() == 0)
//      {
//         return;
//      }
//      else if (controllers.size() == 1)
//      {
//         lcm = controllers.get(0).getTicksPerSimulationTick();
//      }
//      else
//      {
//
//         long[] ticks = new long[controllers.size()];
//         for (int i = 0; i < controllers.size(); i++)
//         {
//            ticks[i] = controllers.get(i).getTicksPerSimulationTick();
//         }
//
//         lcm = MathTools.lcm(ticks);
//      }
//
//      if (currentControlTick.getLongValue() % lcm != 0)
//      {
//         scs.stepBackwardNow();
//      }
   }


   private void updateDynamicGraphicObjectListRegistries()
   {
      for (int i = 0; i < controllers.size(); i++)
      {
         controllers.get(i).updateDynamicGraphicObjectListRegistry();
      }
   }

   @Override
   public void update(int tick)
   {
      updateDynamicGraphicObjectListRegistries();
   }


}
