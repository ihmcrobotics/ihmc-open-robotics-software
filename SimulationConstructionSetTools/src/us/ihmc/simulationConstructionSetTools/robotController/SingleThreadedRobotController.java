package us.ihmc.simulationConstructionSetTools.robotController;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SingleThreadedRobotController extends AbstractThreadedRobotController
{
   
   private final SimulationConstructionSet scs;
   
   public SingleThreadedRobotController(String name, Robot simulatedRobot, SimulationConstructionSet scs)
   {
      super(name, simulatedRobot);   
      
      this.scs = scs;
   }

   @Override
   public void addController(MultiThreadedRobotControlElement controller, int executionsPerControlTick, boolean skipFirstControlCycle)
   {
      controllers.add(new SingleThreadedRobotControllerExecutor(simulatedRobot, controller, executionsPerControlTick, skipFirstControlCycle, registry));
      
      if(scs != null && controller.getYoGraphicsListRegistry() != null)
      {
         scs.addYoGraphicsListRegistry(controller.getYoGraphicsListRegistry(), true);
      }
   }
   
   @Override
   public void doControl()
   {
      for (int i = 0; i < controllers.size(); i++)
      {
         controllers.get(i).readData(currentControlTick.getLongValue());
         controllers.get(i).executeForSimulationTick(currentControlTick.getLongValue());
         controllers.get(i).waitAndWriteData(currentControlTick.getLongValue());
      }
      currentControlTick.increment();
   }
}
