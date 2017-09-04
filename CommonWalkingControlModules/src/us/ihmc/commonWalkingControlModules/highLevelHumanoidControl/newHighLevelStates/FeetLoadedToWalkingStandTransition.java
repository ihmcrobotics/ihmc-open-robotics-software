package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class FeetLoadedToWalkingStandTransition extends FeetLoadedTransition
{
   private final NewHighLevelControllerStates nextState;
   private final YoEnum<NewHighLevelControllerStates> requestedState;

   private final YoBoolean waitForRequest;

   public FeetLoadedToWalkingStandTransition(NewHighLevelControllerStates nextState, YoEnum<NewHighLevelControllerStates> requestedState,
                                             ForceSensorDataHolderReadOnly forceSensorDataHolder, SideDependentList<String> feetContactSensors,
                                             HighLevelHumanoidControllerToolbox controllerToolbox, boolean waitForRequest, YoVariableRegistry parentRegistry)
   {
      super(forceSensorDataHolder, feetContactSensors, controllerToolbox, parentRegistry);

      this.nextState = nextState;
      this.requestedState = requestedState;

      this.waitForRequest = new YoBoolean("waitForRequestToTransitionToWalking", registry);
      this.waitForRequest.set(waitForRequest);
   }

   @Override
   public boolean checkCondition()
   {
      if (super.checkCondition())
         return false;

      boolean transitionRequested = nextState.equals(requestedState.getEnumValue());

      if (waitForRequest.getBooleanValue())
         return transitionRequested;
      else
         return true;
   }
}
