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
      
      if(controller.getDynamicGraphicObjectsListRegistry() != null)
      {
         scs.addYoGraphicsListRegistry(controller.getDynamicGraphicObjectsListRegistry(), true);
      }
   }
}
