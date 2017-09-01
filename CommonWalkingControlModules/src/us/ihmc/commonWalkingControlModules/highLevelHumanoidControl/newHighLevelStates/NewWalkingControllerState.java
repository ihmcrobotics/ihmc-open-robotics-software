package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.*;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.*;

public class NewWalkingControllerState extends NewHighLevelControllerState
{
   private final static NewHighLevelControllerStates controllerState = NewHighLevelControllerStates.WALKING_STATE;

   private final WalkingHighLevelHumanoidController walkingController;

   public NewWalkingControllerState(WalkingHighLevelHumanoidController walkingController)
   {
      super(controllerState);

      this.walkingController = walkingController;
   }


   @Override
   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      walkingController.setControllerCoreOutput(controllerCoreOutput);
   }

   public void initialize()
   {
      walkingController.initialize();
   }

   public void initializeDesiredHeightToCurrent()
   {
      walkingController.initializeDesiredHeightToCurrent();
   }


   @Override
   public void doAction()
   {
      walkingController.doAction();
   }

   public void updateFailureDetection()
   {
      walkingController.updateFailureDetection();
   }

   public void updateManagers(WalkingState currentState)
   {
      walkingController.updateManagers(currentState);
   }


   public void reinitializePelvisOrientation(boolean reinitialize)
   {
      walkingController.reinitializePelvisOrientation(reinitialize);
   }

   @Override
   public void doTransitionIntoAction()
   {
      walkingController.doTransitionIntoAction();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      walkingController.doTransitionOutOfAction();
   }

   @Override
   public ControllerCoreCommand getControllerCoreCommand()
   {
      return walkingController.getControllerCoreCommand();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return walkingController.getYoVariableRegistry();
   }

   /**
    * Get defined states for the walking high level humanoid controller
    *
    * Inefficient, use only in construction
    *
    * @param states return list of walking states
    */
   public void getOrderedWalkingStatesForWarmup(List<WalkingStateEnum> states)
   {
      walkingController.getOrderedWalkingStatesForWarmup(states);
   }

   /**
    * Run one set of doTransitionIntoAction, doAction and doTransitionOutOfAction for a given state.
    *
    * The balance manager is updated, but no commands are consumed.
    *
    * This can be used to warmup the JIT compiler.
    *
    *
    * @param state
    */
   public void warmupStateIteration(WalkingStateEnum state)
   {
      walkingController.warmupStateIteration(state);
   }

   /**
    * Returns the currently active walking state. This is used for unit testing.
    * @return WalkingStateEnum
    */
   public WalkingStateEnum getWalkingStateEnum()
   {
      return walkingController.getWalkingStateEnum();
   }
}
