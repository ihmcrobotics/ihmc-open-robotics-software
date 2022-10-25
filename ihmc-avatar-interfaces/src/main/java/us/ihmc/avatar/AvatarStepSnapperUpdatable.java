package us.ihmc.avatar;

import us.ihmc.avatar.stepAdjustment.PlanarRegionFootstepSnapper;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin.StepGeneratorCommandInputManager;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class AvatarStepSnapperUpdatable extends PlanarRegionFootstepSnapper implements Updatable
{
   private final StepGeneratorCommandInputManager commandInputManager;
   private final YoBoolean shouldSnapToRegions;

   public AvatarStepSnapperUpdatable(SteppingParameters steppingParameters, StepGeneratorCommandInputManager commandInputManager)
   {
      super(steppingParameters);

      shouldSnapToRegions = new YoBoolean("shouldSnapToRegions", registry);
      this.commandInputManager = commandInputManager;
   }

   public void setShouldSnapToRegions(boolean shouldSnapToRegions)
   {
      this.shouldSnapToRegions.set(shouldSnapToRegions);
   }

   @Override
   public void update(double time)
   {
      if (commandInputManager != null)
      {
         if (commandInputManager.getCommandInputManager().isNewCommandAvailable(PlanarRegionsListCommand.class))
         {
            PlanarRegionsListCommand commands = commandInputManager.getCommandInputManager().pollNewestCommand(PlanarRegionsListCommand.class);
            super.setPlanarRegions(commands);
         }

         commandInputManager.getCommandInputManager().clearCommands(PlanarRegionsListCommand.class);
      }
   }

   @Override
   public boolean adjustFootstep(FramePose3DReadOnly stanceFootPose, FramePose2DReadOnly footstepPose, RobotSide footSide, FixedFramePose3DBasics adjustedPoseToPack)
   {
      if (!shouldSnapToRegions.getValue())
         return true;

      return super.adjustFootstep(stanceFootPose, footstepPose, footSide, adjustedPoseToPack);
   }
}
