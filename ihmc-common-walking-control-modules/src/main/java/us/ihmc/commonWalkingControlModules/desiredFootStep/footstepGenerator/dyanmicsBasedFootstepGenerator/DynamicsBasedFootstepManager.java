package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.dyanmicsBasedFootstepGenerator;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class DynamicsBasedFootstepManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DynamicsBasedFootstepTouchdownCalculator touchdownCalculator;

   private final FootstepDataListCommand command = new FootstepDataListCommand();
   private final FootstepDataCommand nextFootstep = new FootstepDataCommand();

   private enum FootstepProviderMode {CSG, DB};
   private final YoEnum<FootstepProviderMode> footstepProviderModeYoEnum = new YoEnum<>("footstepProviderMode", registry, FootstepProviderMode.class);

   public DynamicsBasedFootstepManager(YoRegistry parentRegistry)
   {
      touchdownCalculator = new DynamicsBasedFootstepTouchdownCalculator();

      parentRegistry.addChild(registry);
   }

   public FootstepDataListCommand update(FootstepDataListCommand otherCommand)
   {
      command.clear();

      //nextFootstep.set();

      command.addFootstep(nextFootstep);

      for (int i = 1; i < otherCommand.getNumberOfFootsteps(); i++)
      {
         FootstepDataCommand footstep = otherCommand.getFootstep(i);
         command.addFootstep(footstep);
      }

      return command;
   }
}