package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.FormattingTools;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootLoadBearingCommand;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class WalkingState implements State
{
   protected final YoRegistry registry;
   private final WalkingStateEnum walkingStateEnum;
   private WalkingStateEnum previousWalkingStateEnum = null;

   protected final HighLevelHumanoidControllerToolbox controllerToolbox;
   protected final BalanceManager balanceManager;

   public WalkingState(WalkingStateEnum stateEnum,
                       HighLevelControlManagerFactory managerFactory,
                       HighLevelHumanoidControllerToolbox controllerToolbox,
                       YoRegistry parentRegistry)
   {
      this.walkingStateEnum = stateEnum;
      this.controllerToolbox = controllerToolbox;

      balanceManager = managerFactory.getOrCreateBalanceManager();

      registry = new YoRegistry(FormattingTools.underscoredToCamelCase(stateEnum.toString(), true));
      parentRegistry.addChild(registry);
   }

   public boolean isDoubleSupportState()
   {
      return getStateEnum().isDoubleSupport();
   }

   public boolean isSingleSupportState()
   {
      return getStateEnum().isSingleSupport();
   }

   public RobotSide getSupportSide()
   {
      return getStateEnum().getSupportSide();
   }

   public RobotSide getTransferToSide()
   {
      return getStateEnum().getTransferToSide();
   }

   public void handleChangeInContactState()
   {
      boolean haveContactStatesChanged = false;
      for (RobotSide robotSide : RobotSide.values)
      {
         YoPlaneContactState contactState = controllerToolbox.getFootContactState(robotSide);
         if (contactState.peekContactHasChangedNotification())
            haveContactStatesChanged = true;
      }

      if (!haveContactStatesChanged)
         return;

      controllerToolbox.updateBipedSupportPolygons();
      balanceManager.computeICPPlan();
   }

   public void handleFootLoadBearingCommand(FootLoadBearingCommand command)
   {
      // Override in state that can handle EndEffectorLoadBearingCommand
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return null;
   }

   public boolean isStateSafeToConsumePelvisTrajectoryCommand()
   {
      return false;
   }

   public boolean isStateSafeToConsumeManipulationCommands()
   {
      return false;
   }

   public WalkingStateEnum getStateEnum()
   {
      return walkingStateEnum;
   }

   public void setPreviousWalkingStateEnum(WalkingStateEnum previousWalkingStateEnum)
   {
      this.previousWalkingStateEnum = previousWalkingStateEnum;
   }

   public WalkingStateEnum getPreviousWalkingStateEnum()
   {
      return previousWalkingStateEnum;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

}
