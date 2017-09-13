package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.FeetLoadedToWalkingStandTransition;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.FinishableState;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.EnumMap;

public class FeetLoadedToWalkingStandTransitionFactory implements ControllerStateTransitionFactory<HighLevelController>
{
   private StateTransition<HighLevelController> stateTransition;

   private final HighLevelController stateToAttachEnum;
   private final HighLevelController nextStateEnum;

   private final YoEnum<HighLevelController> requestedState;
   private final SideDependentList<String> feetForceSensors;

   public FeetLoadedToWalkingStandTransitionFactory(HighLevelController stateToAttachEnum, HighLevelController nextStateEnum,
                                                    YoEnum<HighLevelController> requestedState,
                                                    SideDependentList<String> feetForceSensors)
   {
      this.stateToAttachEnum = stateToAttachEnum;
      this.nextStateEnum = nextStateEnum;
      this.requestedState = requestedState;
      this.feetForceSensors = feetForceSensors;
   }

   @Override
   public StateTransition<HighLevelController> getOrCreateStateTransition(EnumMap<HighLevelController, ? extends FinishableState<HighLevelController>> controllerStateMap,
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
   public HighLevelController getStateToAttachEnum()
   {
      return stateToAttachEnum;
   }
}
