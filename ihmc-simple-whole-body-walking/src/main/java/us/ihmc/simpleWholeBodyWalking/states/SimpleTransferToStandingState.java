package us.ihmc.simpleWholeBodyWalking.states;

import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simpleWholeBodyWalking.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleTransferToStandingState extends SimpleWalkingState
{
   private final YoDouble maxICPErrorToSwitchToStanding = new YoDouble("maxICPErrorToSwitchToStanding", registry);

   private final WalkingMessageHandler walkingMessageHandler;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final SimpleCenterOfMassHeightManager comHeightManager;
   private final SimpleBalanceManager balanceManager;
   private final SimplePelvisOrientationManager pelvisOrientationManager;
   private final SimpleFeetManager feetManager;

   public SimpleTransferToStandingState(WalkingMessageHandler walkingMessageHandler,
                                        HighLevelHumanoidControllerToolbox controllerToolbox,
                                        SimpleControlManagerFactory managerFactory,
                                        WalkingFailureDetectionControlModule failureDetectionControlModule,
                                        YoVariableRegistry parentRegistry)
   {
      super(SimpleWalkingStateEnum.TO_STANDING, parentRegistry);
      maxICPErrorToSwitchToStanding.set(0.025);

      this.walkingMessageHandler = walkingMessageHandler;
      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
      comHeightManager.setSupportLeg(RobotSide.LEFT);
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
      double extraToeOffHeight = 0.0;
      comHeightManager.initialize(transferToAndNextFootstepsDataForDoubleSupport, extraToeOffHeight);

      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      double finalTransferSplitFraction = walkingMessageHandler.getFinalTransferSplitFraction();
      double finalTransferWeightDistribution = walkingMessageHandler.getFinalTransferWeightDistribution();

      // Just standing in double support, do nothing
      pelvisOrientationManager.centerInMidFeetZUpFrame(finalTransferTime);
      balanceManager.setFinalTransferSplitFraction(finalTransferSplitFraction);
      balanceManager.setFinalTransferWeightDistribution(finalTransferWeightDistribution);
      balanceManager.setICPPlanTransferFromSide(previousSupportSide);
      balanceManager.initializeICPPlanForTransferToStanding(finalTransferTime);
   }
}