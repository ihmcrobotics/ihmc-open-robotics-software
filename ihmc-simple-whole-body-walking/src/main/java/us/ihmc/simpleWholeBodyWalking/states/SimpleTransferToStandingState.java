package us.ihmc.simpleWholeBodyWalking.states;

import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simpleWholeBodyWalking.SimpleBalanceManager;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.simpleWholeBodyWalking.SimpleFeetManager;
import us.ihmc.simpleWholeBodyWalking.SimplePelvisOrientationManager;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleTransferToStandingState extends SimpleWalkingState
{
   private final YoDouble maxICPErrorToSwitchToStanding = new YoDouble("maxICPErrorToSwitchToStanding", registry);

   private final WalkingMessageHandler walkingMessageHandler;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final SimpleBalanceManager balanceManager;
   private final SimplePelvisOrientationManager pelvisOrientationManager;
   private final SimpleFeetManager feetManager;

   public SimpleTransferToStandingState(WalkingMessageHandler walkingMessageHandler,
                                        HighLevelHumanoidControllerToolbox controllerToolbox,
                                        SimpleControlManagerFactory managerFactory,
                                        WalkingFailureDetectionControlModule failureDetectionControlModule,
                                        YoRegistry parentRegistry)
   {
      super(SimpleWalkingStateEnum.TO_STANDING, parentRegistry);
      maxICPErrorToSwitchToStanding.set(0.025);

      this.walkingMessageHandler = walkingMessageHandler;
      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
   }

   @Override
   public boolean isDone(double timeInState)
   {
      if (!balanceManager.isICPPlanDone())
         return false;

      return balanceManager.getICPErrorMagnitude() < maxICPErrorToSwitchToStanding.getDoubleValue();
   }

   @Override
   public void onEntry()
   {
      balanceManager.clearICPPlan();

      SimpleWalkingStateEnum previousStateEnum = getPreviousWalkingStateEnum();

      // This can happen if walking is paused or aborted while the robot is on its toes already. In that case
      // restore the full foot contact.
      if (previousStateEnum != null && previousStateEnum.isDoubleSupport())
         feetManager.initializeContactStatesForDoubleSupport(null);

      RobotSide previousSupportSide = null;
      if (previousStateEnum != null)
      {
         if (previousStateEnum.getSupportSide() != null)
            previousSupportSide = previousStateEnum.getSupportSide();
         else if (previousStateEnum.getTransferToSide() != null)
            previousSupportSide = previousStateEnum.getTransferToSide();
      }

      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      failureDetectionControlModule.setNextFootstep(null);

      NewTransferToAndNextFootstepsData transferToAndNextFootstepsDataForDoubleSupport = walkingMessageHandler.createTransferToAndNextFootstepDataForDoubleSupport(
            RobotSide.LEFT);

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();

      // Just standing in double support, do nothing
      pelvisOrientationManager.centerInMidFeetZUpFrame(finalTransferTime);
      balanceManager.setICPPlanTransferFromSide(previousSupportSide);
      balanceManager.initializeICPPlanForTransferToStanding(finalTransferTime);
   }
}