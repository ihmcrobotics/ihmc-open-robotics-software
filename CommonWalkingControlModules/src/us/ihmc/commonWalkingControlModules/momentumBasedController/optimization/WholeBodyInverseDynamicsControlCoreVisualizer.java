package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

public class WholeBodyInverseDynamicsControlCoreVisualizer
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public WholeBodyInverseDynamicsControlCoreVisualizer(YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      
      parentRegistry.addChild(registry);
   }

   public void update(MomentumModuleSolution momentumModuleSolution)
   {
      
   }
}
