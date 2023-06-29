package us.ihmc.commonWalkingControlModules.messageHandlers;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StepConstraintRegionCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.StepConstraintsListCommand;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;
import java.util.List;

public class StepConstraintRegionHandler
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoBoolean hasNewConstraintRegion = new YoBoolean("hasNewConstraintRegion", registry);

   private final RecyclingArrayList<StepConstraintRegion> stepConstraintRegionPool = new RecyclingArrayList<>(StepConstraintRegion::new);
   private final List<StepConstraintRegion> stepConstraintRegions = new ArrayList<>();

   private final YoBoolean waitingOnNewConstraintRegion = new YoBoolean("waitingOnNewConstraintRegion", registry);

   public StepConstraintRegionHandler(YoRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }

   public void handleStepConstraintRegionCommand(StepConstraintRegionCommand stepConstraintRegionCommand)
   {
      stepConstraintRegions.clear();
      stepConstraintRegionPool.clear();
      StepConstraintRegion stepConstraintRegion = stepConstraintRegionPool.add();
      stepConstraintRegionCommand.getStepConstraintRegion(stepConstraintRegion);
      stepConstraintRegions.add(stepConstraintRegion);

      hasNewConstraintRegion.set(true);
      waitingOnNewConstraintRegion.set(false);
   }

   public void handleStepConstraintsListCommand(StepConstraintsListCommand stepConstraintsListCommand)
   {
      stepConstraintRegionPool.clear();
      stepConstraintRegions.clear();
      for (int i = 0; i < stepConstraintsListCommand.getNumberOfConstraints(); i++)
      {
         StepConstraintRegion constraintRegion = stepConstraintRegionPool.add();
         stepConstraintsListCommand.getStepConstraint(i).getStepConstraintRegion(constraintRegion);
         stepConstraintRegions.add(constraintRegion);
      }

      hasNewConstraintRegion.set(true);
      waitingOnNewConstraintRegion.set(false);
   }

   public void handleStepConstraintsList(List<StepConstraintRegion> stepConstraintRegions)
   {
      stepConstraintRegionPool.clear();
      this.stepConstraintRegions.clear();
      for (int i = 0; i < stepConstraintRegions.size(); i++)
      {
         this.stepConstraintRegions.add(stepConstraintRegions.get(i));
      }

      hasNewConstraintRegion.set(true);
      waitingOnNewConstraintRegion.set(false);
   }


   public boolean hasNewStepConstraintRegion()
   {
      return hasNewConstraintRegion.getBooleanValue();
   }

   public List<StepConstraintRegion> pollHasNewStepConstraintRegions()
   {
      if (!hasNewConstraintRegion.getBooleanValue())
         return null;

      hasNewConstraintRegion.set(false);

      return stepConstraintRegions;
   }
}
