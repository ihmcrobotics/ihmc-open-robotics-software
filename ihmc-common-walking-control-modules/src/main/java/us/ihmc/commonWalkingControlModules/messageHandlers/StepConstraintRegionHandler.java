package us.ihmc.commonWalkingControlModules.messageHandlers;

import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StepConstraintRegionCommand;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class StepConstraintRegionHandler
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean hasNewConstraintRegion = new YoBoolean("hasNewConstraintRegion", registry);
   private final StepConstraintRegion stepConstraintRegion = new StepConstraintRegion();

   private final YoBoolean waitingOnNewConstraintRegion = new YoBoolean("waitingOnNewConstraintRegion", registry);

   public StepConstraintRegionHandler(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public void handleStepConstraintRegionCommand(StepConstraintRegionCommand stepConstraintRegionCommand)
   {
      stepConstraintRegionCommand.getStepConstraintRegion(stepConstraintRegion);

      hasNewConstraintRegion.set(true);
      waitingOnNewConstraintRegion.set(false);
   }

   public boolean hasNewStepConstraintRegion()
   {
      return hasNewConstraintRegion.getBooleanValue();
   }

   public StepConstraintRegion pollHasNewStepConstraintRegion()
   {
      if (!hasNewConstraintRegion.getBooleanValue())
         return null;

      hasNewConstraintRegion.set(false);

      return stepConstraintRegion;
   }
}
