package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.robotics.controllers.ControllerFailureListener;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.concurrent.atomic.AtomicReference;

public class ControllerFailedTransition implements StateTransitionCondition
{
   private final YoBoolean controllerFailed;
   private final AtomicReference<HighLevelController> fallbackControllerForFailure;

   public ControllerFailedTransition(HighLevelController stateToAttachEnum, HighLevelController constructedFallbackState,
                                     AtomicReference<HighLevelController> fallbackControllerForFailure, HighLevelHumanoidControllerToolbox controllerToolbox,
                                     YoVariableRegistry registry)
   {
      this.fallbackControllerForFailure = fallbackControllerForFailure;

      controllerFailed = new YoBoolean(stateToAttachEnum + "ControllerFailed", registry);
      controllerToolbox.attachControllerFailureListener(new ControllerFailureListener()
      {
         @Override
         public void controllerFailed(FrameVector2D fallingDirection)
         {
            controllerFailed.set(true);
         }
      });
   }

   @Override
   public boolean checkCondition()
   {
      if (fallbackControllerForFailure.get() == null)
         return false;

      return controllerFailed.getBooleanValue();
   }
}
