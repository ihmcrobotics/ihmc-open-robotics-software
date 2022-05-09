package us.ihmc.commonWalkingControlModules.messageHandlers;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StepConstraintRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StepConstraintsListCommand;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class StepConstraintRegionHandler
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean hasNewConstraintRegion = new YoBoolean("hasNewConstraintRegion", registry);

   private final RecyclingArrayList<StepConstraintRegion> stepConstraintRegions = new RecyclingArrayList<>(StepConstraintRegion::new);

   private final YoBoolean waitingOnNewConstraintRegion = new YoBoolean("waitingOnNewConstraintRegion", registry);

   public StepConstraintRegionHandler(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public void handleStepConstraintRegionCommand(StepConstraintRegionCommand stepConstraintRegionCommand)
   {
      stepConstraintRegions.clear();
      stepConstraintRegionCommand.getStepConstraintRegion(stepConstraintRegions.add());

      hasNewConstraintRegion.set(true);
      waitingOnNewConstraintRegion.set(false);
   }

   public void handleStepConstraintsListCommand(StepConstraintsListCommand stepConstraintsListCommand)
   {
      stepConstraintRegions.clear();
      for (int i = 0; i < stepConstraintsListCommand.getNumberOfConstraints(); i++)
      {
         stepConstraintsListCommand.getStepConstraint(i).getStepConstraintRegion(stepConstraintRegions.add());
      }

      hasNewConstraintRegion.set(true);
      waitingOnNewConstraintRegion.set(false);
   }

   public boolean hasNewStepConstraintRegion()
   {
      stepConstraintRegions.clear();
      {
         return hasNewConstraintRegion.getBooleanValue();
      }
   }

   public StepConstraintRegion pollHasNewStepConstraintRegion()
   {
      if (!hasNewConstraintRegion.getBooleanValue())
         return null;

      hasNewConstraintRegion.set(false);

      return stepConstraintRegions.get(0);
   }
}
