package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedControlManagerFactory
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedForceControllerToolbox toolbox;

   public QuadrupedControlManagerFactory(QuadrupedForceControllerToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      this.toolbox = toolbox;

      parentRegistry.addChild(registry);
   }
}
