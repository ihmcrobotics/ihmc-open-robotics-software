package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.EnumMap;

public class FeetLoadedToWalkingStandTransitionFactory implements ControllerStateTransitionFactory<NewHighLevelControllerStates>
{
   private StateTransition<NewHighLevelControllerStates> stateTransition;

   private final NewHighLevelControllerStates stateToAttachEnum;
   private final NewHighLevelControllerStates nextStateEnum;

   private final YoEnum<NewHighLevelControllerStates> requestedState;
   private final SideDependentList<String> feetForceSensors;

   public FeetLoadedToWalkingStandTransitionFactory(NewHighLevelControllerStates stateToAttachEnum, NewHighLevelControllerStates nextStateEnum,
                                                    YoEnum<NewHighLevelControllerStates> requestedState,
                                                    SideDependentList<String> feetForceSensors)
   {
      this.stateToAttachEnum = stateToAttachEnum;
      this.nextStateEnum = nextStateEnum;
      this.requestedState = requestedState;
      this.feetForceSensors = feetForceSensors;
   }

   @Override
   public StateTransition<NewHighLevelControllerStates> getOrCreateStateTransition(EnumMap<NewHighLevelControllerStates, ? extends FinishableState<NewHighLevelControllerStates>> controllerStateMap,
                                                                                   HighLevelControllerFactoryHelper controllerFactoryHelper, ForceSensorDataHolderReadOnly forceSensorDataHolder,
                                                                                   YoVariableRegistry parentRegistry)
   {
      if (stateTransition != null)
         return stateTransition;

      double totalMass = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getFullRobotModel().getTotalMass();
      double gravityZ = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getGravityZ();
      double controlDT = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlDT();
      HighLevelControllerParameters highLevelControllerParameters = controllerFactoryHelper.getHighLevelControllerParameters();

      StateTransitionCondition stateTransitionCondition = new FeetLoadedToWalkingStandTransition(controllerStateMap.get(stateToAttachEnum), nextStateEnum,
                                                                                                 requestedState, forceSensorDataHolder, feetForceSensors,
                                                                                                 controlDT, totalMass, gravityZ, highLevelControllerParameters, parentRegistry);
      stateTransition = new StateTransition<>(nextStateEnum, stateTransitionCondition);

      return stateTransition;
   }

   @Override
   public NewHighLevelControllerStates getStateToAttachEnum()
   {
      return stateToAttachEnum;
   }
}
