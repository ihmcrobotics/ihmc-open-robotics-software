package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.FeetLoadedToWalkingStandTransition;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class FeetLoadedToWalkingStandTransitionFactory implements ControllerStateTransitionFactory<HighLevelControllerName>
{
   private StateTransition<HighLevelControllerName> stateTransition;

   private final HighLevelControllerName stateToAttachEnum;
   private final HighLevelControllerName nextStateEnum;

   private final YoEnum<HighLevelControllerName> requestedState;
   private final SideDependentList<String> feetForceSensors;

   public FeetLoadedToWalkingStandTransitionFactory(HighLevelControllerName stateToAttachEnum, HighLevelControllerName nextStateEnum,
                                                    YoEnum<HighLevelControllerName> requestedState, SideDependentList<String> feetForceSensors)
   {
      this.stateToAttachEnum = stateToAttachEnum;
      this.nextStateEnum = nextStateEnum;
      this.requestedState = requestedState;
      this.feetForceSensors = feetForceSensors;
   }

   @Override
   public StateTransition<HighLevelControllerName> getOrCreateStateTransition(EnumMap<HighLevelControllerName, ? extends State> controllerStateMap,
                                                                              HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                                              YoRegistry parentRegistry)
   {
      if (stateTransition != null)
         return stateTransition;

      double totalMass = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getFullRobotModel().getTotalMass();
      double gravityZ = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getGravityZ();
      double controlDT = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlDT();
      ForceSensorDataHolderReadOnly forceSensorDataHolder = controllerFactoryHelper.getForceSensorDataHolder();
      HighLevelControllerParameters highLevelControllerParameters = controllerFactoryHelper.getHighLevelControllerParameters();

      StateTransitionCondition stateTransitionCondition = new FeetLoadedToWalkingStandTransition(nextStateEnum, requestedState, forceSensorDataHolder,
                                                                                                 feetForceSensors, controlDT, totalMass, gravityZ,
                                                                                                 highLevelControllerParameters, parentRegistry);
      stateTransition = new StateTransition<>(nextStateEnum, stateTransitionCondition);

      return stateTransition;
   }

   @Override
   public HighLevelControllerName getStateToAttachEnum()
   {
      return stateToAttachEnum;
   }
}
