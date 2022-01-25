package us.ihmc.exampleSimulations.lipmWalker;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class LIPMWalkerController implements RobotController
{

   private YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      // TODO Auto-generated method stub
      return registry;
   }

   @Override
   public void doControl()
   {
      // TODO Auto-generated method stub

   }

}
