package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions.FeetLoadedToWalkingStandTransition;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
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
   private final HighLevelControllerParameters highLevelControllerParameters;

   private final YoEnum<HighLevelControllerName> requestedState;
   private final SideDependentList<String> feetForceSensors;

   private final boolean waitForRequestToTransition;

   public FeetLoadedToWalkingStandTransitionFactory(HighLevelControllerName stateToAttachEnum,
                                                    HighLevelControllerName nextStateEnum,
                                                    HighLevelControllerParameters highLevelControllerParameters,
                                                    YoEnum<HighLevelControllerName> requestedState,
                                                    boolean waitForRequestToTransition,
                                                    SideDependentList<String> feetForceSensors)
   {
      this.stateToAttachEnum = stateToAttachEnum;
      this.nextStateEnum = nextStateEnum;
      this.requestedState = requestedState;
      this.highLevelControllerParameters = highLevelControllerParameters;
      this.waitForRequestToTransition = waitForRequestToTransition;
      this.feetForceSensors = feetForceSensors;
   }

   @Override
   public StateTransition<HighLevelControllerName> getOrCreateStateTransition(EnumMap<HighLevelControllerName, ? extends State> controllerStateMap,
                                                                              HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                                              YoRegistry parentRegistry)
   {
      if (stateTransition != null)
         return stateTransition;

      HighLevelHumanoidControllerToolbox controllerToolbox = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox();
      double totalMass = controllerToolbox.getFullRobotModel().getTotalMass();
      double gravityZ = controllerToolbox.getGravityZ();
      HighLevelControllerParameters highLevelControllerParameters = controllerFactoryHelper.getHighLevelControllerParameters();

      StateTransitionCondition stateTransitionCondition = new FeetLoadedToWalkingStandTransition(nextStateEnum,
                                                                                                 requestedState,
                                                                                                 waitForRequestToTransition,
                                                                                                 controllerToolbox.getFootSwitches(),
                                                                                                 highLevelControllerParameters.getControlDT(stateToAttachEnum),
                                                                                                 totalMass,
                                                                                                 gravityZ,
                                                                                                 highLevelControllerParameters,
                                                                                                 parentRegistry);
      stateTransition = new StateTransition<>(nextStateEnum, stateTransitionCondition);

      return stateTransition;
   }

   @Override
   public HighLevelControllerName getStateToAttachEnum()
   {
      return stateToAttachEnum;
   }
}
