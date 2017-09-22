package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.BalanceManager;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class TransferState extends WalkingState
{
   protected final RobotSide transferToSide;

   protected final WalkingMessageHandler walkingMessageHandler;
   protected final HighLevelHumanoidControllerToolbox controllerToolbox;
   protected final WalkingFailureDetectionControlModule failureDetectionControlModule;

   protected final CenterOfMassHeightManager comHeightManager;
   protected final BalanceManager balanceManager;
   protected final PelvisOrientationManager pelvisOrientationManager;
   protected final FeetManager feetManager;

   private final FramePoint2D desiredICPLocal = new FramePoint2D();
   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FramePoint2D desiredCMP = new FramePoint2D();

   private final FramePoint2D filteredDesiredCoP = new FramePoint2D();
   private final FramePoint2D desiredCoP = new FramePoint2D();
   private final FramePoint3D nextExitCMP = new FramePoint3D();

   private final Footstep nextFootstep = new Footstep();

   public TransferState(RobotSide transferToSide, WalkingStateEnum transferStateEnum, WalkingMessageHandler walkingMessageHandler,
                        HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControlManagerFactory managerFactory,
                        WalkingFailureDetectionControlModule failureDetectionControlModule, YoVariableRegistry parentRegistry)
   {
      super(transferStateEnum, parentRegistry);
      this.transferToSide = transferToSide;
      this.walkingMessageHandler = walkingMessageHandler;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.controllerToolbox = controllerToolbox;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
   }

   public RobotSide getTransferToSide()
   {
      return transferToSide;
   }

   @Override
   public void doAction()
   {
      feetManager.updateContactStatesInDoubleSupport(transferToSide);

      switchToToeOffIfPossible();

      // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
      comHeightManager.setSupportLeg(transferToSide);
   }

   @Override
   public boolean isDone()
   {
      if (!balanceManager.isICPPlanDone())
         return false;
      balanceManager.getCapturePoint(capturePoint2d);
      FrameConvexPolygon2d supportPolygonInWorld = controllerToolbox.getBipedSupportPolygons().getSupportPolygonInWorld();
      boolean isICPInsideSupportPolygon = supportPolygonInWorld.isPointInside(capturePoint2d);

      if (!isICPInsideSupportPolygon)
         return true;
      else
         return balanceManager.isTransitionToSingleSupportSafe(transferToSide);
   }

   public void switchToToeOffIfPossible()
   {
      RobotSide trailingLeg = transferToSide.getOppositeSide();

      boolean shouldComputeToeLineContact = feetManager.shouldComputeToeLineContact();
      boolean shouldComputeToePointContact = feetManager.shouldComputeToePointContact();

      if (shouldComputeToeLineContact || shouldComputeToePointContact)
      {
         balanceManager.getDesiredCMP(desiredCMP);
         balanceManager.getDesiredICP(desiredICPLocal);
         balanceManager.getCapturePoint(capturePoint2d);
         balanceManager.getNextExitCMP(nextExitCMP);

         controllerToolbox.getFilteredDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(trailingLeg), filteredDesiredCoP);
         controllerToolbox.getDesiredCenterOfPressure(controllerToolbox.getContactableFeet().get(trailingLeg), desiredCoP);

         feetManager.updateToeOffStatusDoubleSupport(trailingLeg, nextExitCMP, desiredCMP, desiredCoP, desiredICPLocal, capturePoint2d);

         if (feetManager.okForPointToeOff() && shouldComputeToePointContact)
            feetManager.requestPointToeOff(trailingLeg, nextExitCMP, filteredDesiredCoP);
         else if (feetManager.okForLineToeOff() && shouldComputeToeLineContact)
            feetManager.requestLineToeOff(trailingLeg, nextExitCMP, filteredDesiredCoP);
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      balanceManager.clearICPPlan();

      feetManager.initializeContactStatesForDoubleSupport(transferToSide);
      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      if (walkingMessageHandler.hasUpcomingFootsteps())
      {
         walkingMessageHandler.peekFootstep(0, nextFootstep);
         failureDetectionControlModule.setNextFootstep(nextFootstep);
         balanceManager.setUpcomingFootstep(nextFootstep);
      }
      else
      {
         failureDetectionControlModule.setNextFootstep(null);
         balanceManager.setUpcomingFootstep(null);
      }

      balanceManager.resetPushRecovery();

      double transferTime = walkingMessageHandler.getNextTransferTime();
      pelvisOrientationManager.setTrajectoryTime(transferTime);
   }

   public boolean isInitialTransfer()
   {
      return getPreviousState().getStateEnum() == WalkingStateEnum.STANDING;
   }

   @Override
   public void doTransitionOutOfAction()
   {
      feetManager.reset();
   }
}