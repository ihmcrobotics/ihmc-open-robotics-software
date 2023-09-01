package us.ihmc.quadrupedRobotics.controller.states;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.quadrupedRobotics.estimator.footSwitch.QuadrupedFootSwitchInterface;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuadrupedFeetLoadedToWalkingStandTransition extends QuadrupedFeetLoadedTransition
{
   private final HighLevelControllerName nextStateEnum;
   private final YoEnum<HighLevelControllerName> requestedState;

   private final YoBoolean waitForRequest;

   private final YoDouble minimumTimeInState;

   public QuadrupedFeetLoadedToWalkingStandTransition(HighLevelControllerName nextStateEnum,
                                                      YoEnum<HighLevelControllerName> requestedState,
                                                      QuadrantDependentList<QuadrupedFootSwitchInterface> footSwitches,
                                                      double controlDT,
                                                      double totalMass,
                                                      double gravityZ,
                                                      HighLevelControllerParameters highLevelControllerParameters,
                                                      YoRegistry parentRegistry)
   {
      super(footSwitches, controlDT, totalMass, gravityZ, parentRegistry);

      this.nextStateEnum = nextStateEnum;
      this.requestedState = requestedState;

      minimumTimeInState = new YoDouble("minimumTimeLoadingFeet", registry);
      minimumTimeInState.set(highLevelControllerParameters.getMinimumTimeInStandReady());

      this.waitForRequest = new YoBoolean("waitForRequestToTransitionToWalking", registry);
      this.waitForRequest.set(!highLevelControllerParameters.automaticallyTransitionToWalkingWhenReady());
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      if (!super.testCondition(timeInState))
         return false;

      if (timeInState < minimumTimeInState.getDoubleValue())
         return false;

      boolean transitionRequested = nextStateEnum.equals(requestedState.getEnumValue());

      if (waitForRequest.getBooleanValue())
         return transitionRequested;
      else
         return true;
   }
}
