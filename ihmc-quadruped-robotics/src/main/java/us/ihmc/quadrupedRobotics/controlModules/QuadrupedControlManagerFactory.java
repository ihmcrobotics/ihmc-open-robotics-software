package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedControlManagerFactory
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedForceControllerToolbox toolbox;

   private QuadrupedFeetManager feetManager;

   public QuadrupedControlManagerFactory(QuadrupedForceControllerToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      this.toolbox = toolbox;

      parentRegistry.addChild(registry);
   }

   public QuadrupedFeetManager getOrCreateFeetManager()
   {
      if (feetManager != null)
         return feetManager;

      feetManager = new QuadrupedFeetManager(toolbox, registry);
      return feetManager;
   }

   public QuadrupedSolePositionController getOrCreateSolePositionController(RobotQuadrant robotQuadrant)
   {
      return getOrCreateFeetManager().getSolePositionController(robotQuadrant);
   }
}
