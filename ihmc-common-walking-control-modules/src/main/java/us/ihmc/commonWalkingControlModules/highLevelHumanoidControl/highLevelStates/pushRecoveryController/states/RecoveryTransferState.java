package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states;

import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepPushRecoveryControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.TransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryBalanceManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RecoveryTransferState extends PushRecoveryState
{
   private final YoDouble currentTransferDuration = new YoDouble("CurrentTransferDuration", registry);

   protected final RobotSide transferToSide;

   protected final WalkingMessageHandler walkingMessageHandler;
   protected final HighLevelHumanoidControllerToolbox controllerToolbox;
   protected final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final PushRecoveryControllerParameters pushRecoveryParameters;
   protected final CenterOfMassHeightManager comHeightManager;
   protected final PushRecoveryBalanceManager balanceManager;
   protected final FeetManager feetManager;

   private final FramePoint2D capturePoint2d = new FramePoint2D();

   private final FramePoint2D desiredCoP = new FramePoint2D();

   private final FramePoint2D capturePoint = new FramePoint2D();

   private final MultiStepPushRecoveryControlModule pushRecoveryControlModule;
   private final TransferToAndNextFootstepsData transferToAndNextFootstepsData = new TransferToAndNextFootstepsData();

   private final FootstepTiming stepTiming = new FootstepTiming();

   private final Footstep nextFootstep = new Footstep();

   public RecoveryTransferState(PushRecoveryStateEnum stateEnum,
                                WalkingMessageHandler walkingMessageHandler,
                                HighLevelHumanoidControllerToolbox controllerToolbox,
                                PushRecoveryControllerParameters pushRecoveryParameters,
                                PushRecoveryControlManagerFactory managerFactory,
                                MultiStepPushRecoveryControlModule pushRecoveryControlModule,
                                WalkingFailureDetectionControlModule failureDetectionControlModule,
                                YoRegistry parentRegistry)
   {
      super(stateEnum, parentRegistry);

      this.transferToSide = stateEnum.getTransferToSide();
      this.walkingMessageHandler = walkingMessageHandler;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.controllerToolbox = controllerToolbox;
      this.balanceManager = managerFactory.getOrCreateBalanceManager();
      this.pushRecoveryControlModule = pushRecoveryControlModule;
      this.pushRecoveryParameters = pushRecoveryParameters;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      feetManager = managerFactory.getOrCreateFeetManager();

      transferToAndNextFootstepsData.setTransferToSide(transferToSide);
   }


   @Override
   public RobotSide getTransferToSide()
   {
      return transferToSide;
   }

   private void updateICPPlan()
   {
      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      failureDetectionControlModule.setNextFootstep(null);

      double finalTransferTime = pushRecoveryParameters.getFinalTransferDurationForRecovery();

      currentTransferDuration.set(stepTiming.getTransferTime());
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.initializeICPPlanForTransfer();
   }

   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeICPPlan();

      switchToToeOffIfPossible();

      feetManager.updateContactStatesInDoubleSupport(transferToSide);

      capturePoint.setIncludingFrame(controllerToolbox.getCapturePoint());
      pushRecoveryControlModule.updateForDoubleSupport(capturePoint, controllerToolbox.getOmega0());

      // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
      comHeightManager.setSupportLeg(transferToSide.getOppositeSide());
   }

   @Override
   public void onEntry()
   {
      stepTiming.set(pushRecoveryControlModule.pollRecoveryStepTiming());
      nextFootstep.set(pushRecoveryControlModule.pollRecoveryStep());
      updateICPPlan();

      transferToAndNextFootstepsData.setTransferToPosition(controllerToolbox.getReferenceFrames().getSoleFrame(transferToSide));

      comHeightManager.setSupportLeg(transferToSide);
      comHeightManager.initialize(transferToAndNextFootstepsData, 0.0);
   }

   public boolean isICPPlanDone()
   {
      return balanceManager.isICPPlanDone();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return isICPPlanDone() && (pushRecoveryControlModule.isRobotFallingFromDoubleSupport() == transferToSide.getOppositeSide());
   }

   @Override
   public void onExit(double timeInState)
   {
      feetManager.reset();
   }

   /**
    * @return whether or not it switched
    */
   public boolean switchToToeOffIfPossible()
   {
      RobotSide trailingLeg = transferToSide.getOppositeSide();

      if (feetManager.getCurrentConstraintType(trailingLeg) != FootControlModule.ConstraintType.TOES)
      {
         capturePoint2d.setIncludingFrame(balanceManager.getCapturePoint());

         controllerToolbox.getDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(trailingLeg), desiredCoP);

         FramePoint3DReadOnly trailingFootExitCMP = balanceManager.getFirstExitCMPForToeOff(true);
         feetManager.updateToeOffStatusDoubleSupport(trailingLeg,
                                                     trailingFootExitCMP,
                                                     balanceManager.getDesiredCMP(),
                                                     balanceManager.getDesiredICP(),
                                                     capturePoint2d);

         if (feetManager.okForPointToeOff(false))
         {
            feetManager.requestPointToeOff(trailingLeg, trailingFootExitCMP, desiredCoP);
            return true;
         }
         else if (feetManager.okForLineToeOff(false))
         {
            feetManager.requestLineToeOff(trailingLeg, trailingFootExitCMP, desiredCoP);
            return true;
         }
      }
      // switch to point toe off from line toe off
      else if (!feetManager.isUsingPointContactInToeOff(trailingLeg) && !feetManager.useToeLineContactInTransfer())
      {
         FramePoint3DReadOnly trailingFootExitCMP = balanceManager.getFirstExitCMPForToeOff(true);
         controllerToolbox.getDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(trailingLeg), desiredCoP);
         feetManager.requestPointToeOff(trailingLeg, trailingFootExitCMP, desiredCoP);
         return true;
      }
      return false;
   }

}