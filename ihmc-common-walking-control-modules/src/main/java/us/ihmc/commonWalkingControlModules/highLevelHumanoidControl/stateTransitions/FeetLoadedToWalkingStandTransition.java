package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.stateTransitions;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.function.Function;

public class FeetLoadedToWalkingStandTransition extends FeetLoadedTransition
{
   private final HighLevelControllerName nextStateEnum;
   private final YoEnum<HighLevelControllerName> requestedState;

   private final YoBoolean waitForRequest;
   private final YoDouble lastTimeFeetWereUnloaded;

   private final YoDouble minimumTimeInState;

   public FeetLoadedToWalkingStandTransition(HighLevelControllerName nextStateEnum,
                                             YoEnum<HighLevelControllerName> requestedState,
                                             boolean waitForRequestToTransition,
                                             SideDependentList<FootSwitchInterface> footSwitches,
                                             double controlDT,
                                             double totalMass,
                                             double gravityZ,
                                             HighLevelControllerParameters highLevelControllerParameters,
                                             YoRegistry parentRegistry)
   {
      this(nextStateEnum,
           requestedState,
           waitForRequestToTransition,
           (side) -> footSwitches.get(side).getMeasuredWrench(),
           controlDT,
           totalMass,
           gravityZ,
           highLevelControllerParameters,
           parentRegistry);
   }

   public FeetLoadedToWalkingStandTransition(HighLevelControllerName nextStateEnum,
                                             YoEnum<HighLevelControllerName> requestedState,
                                             boolean waitForRequestToTransition,
                                             ForceSensorDataHolderReadOnly forceSensorDataHolder,
                                             SideDependentList<String> feetForceSensors,
                                             double controlDT,
                                             double totalMass,
                                             double gravityZ,
                                             HighLevelControllerParameters highLevelControllerParameters,
                                             YoRegistry parentRegistry)
   {
      this(nextStateEnum,
           requestedState,
           waitForRequestToTransition,
           (side) -> forceSensorDataHolder.getData(feetForceSensors.get(side)).getWrench(),
           controlDT,
           totalMass,
           gravityZ,
           highLevelControllerParameters,
           parentRegistry);
   }

   public FeetLoadedToWalkingStandTransition(HighLevelControllerName nextStateEnum,
                                             YoEnum<HighLevelControllerName> requestedState,
                                             boolean waitForRequestToTransition,
                                             Function<RobotSide, WrenchReadOnly> footSensors,
                                             double controlDT,
                                             double totalMass,
                                             double gravityZ,
                                             HighLevelControllerParameters highLevelControllerParameters,
                                             YoRegistry parentRegistry)
   {
      super(nextStateEnum.toString(), footSensors, controlDT, totalMass, gravityZ, parentRegistry);

      this.nextStateEnum = nextStateEnum;
      this.requestedState = requestedState;

      lastTimeFeetWereUnloaded = new YoDouble("lastTimeFeetWereUnloaded_" + nextStateEnum.toString(), registry);
      minimumTimeInState = new YoDouble("minimumTimeLoadingFeet_" + nextStateEnum.toString(), registry);
      minimumTimeInState.set(highLevelControllerParameters.getMinimumTimeInStandReady());

      this.waitForRequest = new YoBoolean("waitForRequestToTransitionToWalking_" + nextStateEnum.toString(), registry);
      this.waitForRequest.set(waitForRequestToTransition);
   }

   @Override
   public boolean testCondition(double timeInState)
   {
      if (!super.testCondition(timeInState))
      {
         lastTimeFeetWereUnloaded.set(timeInState);
         return false;
      }

      lastTimeFeetWereUnloaded.set(Math.min(lastTimeFeetWereUnloaded.getDoubleValue(), timeInState));

      if (timeInState - lastTimeFeetWereUnloaded.getDoubleValue() < minimumTimeInState.getDoubleValue())
         return false;

      boolean transitionRequested = nextStateEnum.equals(requestedState.getEnumValue());

      if (waitForRequest.getBooleanValue())
         return transitionRequested;
      else
         return true;
   }
}
