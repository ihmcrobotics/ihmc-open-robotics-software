package us.ihmc.simulationconstructionset.robotController;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class SingleThreadedRobotController extends AbstractThreadedRobotController
{
   
   private final SimulationConstructionSet scs;
   
   public SingleThreadedRobotController(String name, DoubleYoVariable yoTime, SimulationConstructionSet scs)
   {
      super(name, yoTime);   
      
      this.scs = scs;
   }

   public void addController(MultiThreadedRobotControlElement controller, int executionsPerControlTick, boolean skipFirstControlCycle)
   {
      controllers.add(new SingleThreadedRobotControllerExecutor(controller, executionsPerControlTick, skipFirstControlCycle, registry));
      
      if(scs != null && controller.getDynamicGraphicObjectsListRegistry() != null)
      {
         scs.addYoGraphicsListRegistry(controller.getDynamicGraphicObjectsListRegistry(), true);
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
