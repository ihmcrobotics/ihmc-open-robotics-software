package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.FeetLoadedToWalkingStandTransition;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.EnumMap;

public class FeetLoadedToWalkingStandTransitionFactory implements ControllerStateTransitionFactory<HighLevelControllerName>
{
   private StateTransition<HighLevelControllerName> stateTransition;

   private final HighLevelControllerName stateToAttachEnum;
   private final HighLevelControllerName nextStateEnum;

   private final YoEnum<HighLevelControllerName> requestedState;
   private final SideDependentList<String> feetForceSensors;

   public FeetLoadedToWalkingStandTransitionFactory(HighLevelControllerName stateToAttachEnum, HighLevelControllerName nextStateEnum,
                                                    YoEnum<HighLevelControllerName> requestedState,
                                                    SideDependentList<String> feetForceSensors)
   {
      this.stateToAttachEnum = stateToAttachEnum;
      this.nextStateEnum = nextStateEnum;
      this.requestedState = requestedState;
      this.feetForceSensors = feetForceSensors;
   }

   @Override
   public StateTransition<HighLevelControllerName> getOrCreateStateTransition(EnumMap<HighLevelControllerName, ? extends FinishableState<HighLevelControllerName>> controllerStateMap,
                                                                              HighLevelControllerFactoryHelper controllerFactoryHelper, YoVariableRegistry parentRegistry)
   {
      if (stateTransition != null)
         return stateTransition;

      double totalMass = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getFullRobotModel().getTotalMass();
      double gravityZ = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getGravityZ();
      double controlDT = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlDT();
      ForceSensorDataHolderReadOnly forceSensorDataHolder = controllerFactoryHelper.getForceSensorDataHolder();
      HighLevelControllerParameters highLevelControllerParameters = controllerFactoryHelper.getHighLevelControllerParameters();

      StateTransitionCondition stateTransitionCondition = new FeetLoadedToWalkingStandTransition(controllerStateMap.get(stateToAttachEnum), nextStateEnum,
                                                                                                 requestedState, forceSensorDataHolder, feetForceSensors,
                                                                                                 controlDT, totalMass, gravityZ, highLevelControllerParameters, parentRegistry);
      stateTransition = new StateTransition<>(nextStateEnum, stateTransitionCondition);

      return stateTransition;
   }

   @Override
   public HighLevelControllerName getStateToAttachEnum()
   {
      return stateToAttachEnum;
   }
}
