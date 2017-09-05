package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.*;

public class NewWalkingControllerState extends NewHighLevelControllerState
{
   private final static NewHighLevelControllerStates controllerState = NewHighLevelControllerStates.WALKING_STATE;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WholeBodyControllerCore controllerCore;
   private final WalkingHighLevelHumanoidController walkingController;

   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);

   public NewWalkingControllerState(HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters,
                                    WholeBodyControllerCore controllerCore, WalkingHighLevelHumanoidController walkingController)
   {
      super(controllerState);

      this.controllerToolbox = controllerToolbox;
      this.controllerCore = controllerCore;
      this.walkingController = walkingController;

      OneDoFJoint[] controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();
      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         LowLevelJointControlMode jointControlMode = highLevelControllerParameters.getLowLevelJointControlMode(controlledJoint.getName(), controllerState);
      }

      registry.addChild(walkingController.getYoVariableRegistry());
   }


   @Override
   public void setControllerCoreOutput(ControllerCoreOutputReadOnly controllerCoreOutput)
   {
      walkingController.setControllerCoreOutput(controllerCoreOutput);
   }

   public void initialize()
   {
      controllerCore.initialize();
      controllerToolbox.initialize();
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

      ControllerCoreCommand controllerCoreCommand = walkingController.getControllerCoreCommand();
      controllerCoreTimer.startMeasurement();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();
      controllerCoreTimer.stopMeasurement();

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
   public LowLevelOneDoFJointDesiredDataHolderReadOnly getOutputForLowLevelController()
   {
      return controllerCore.getOutputForLowLevelController();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
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
