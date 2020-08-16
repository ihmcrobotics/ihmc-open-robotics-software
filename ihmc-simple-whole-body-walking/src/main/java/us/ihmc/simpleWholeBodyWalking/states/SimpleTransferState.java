package us.ihmc.simpleWholeBodyWalking.states;

import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.desiredFootStep.NewTransferToAndNextFootstepsData;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simpleWholeBodyWalking.SimpleBalanceManager;
import us.ihmc.simpleWholeBodyWalking.SimpleControlManagerFactory;
import us.ihmc.simpleWholeBodyWalking.SimpleFeetManager;
import us.ihmc.simpleWholeBodyWalking.SimplePelvisOrientationManager;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class SimpleTransferState extends SimpleWalkingState
{
   protected final RobotSide transferToSide;

   protected final WalkingMessageHandler walkingMessageHandler;
   protected final HighLevelHumanoidControllerToolbox controllerToolbox;
   protected final WalkingFailureDetectionControlModule failureDetectionControlModule;

   protected final SimpleBalanceManager balanceManager;
   protected final SimplePelvisOrientationManager pelvisOrientationManager;
   protected final SimpleFeetManager feetManager;

   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final FramePoint3D desiredCoM = new FramePoint3D();

   private final FootstepTiming stepTiming = new FootstepTiming();

   private final Footstep nextFootstep = new Footstep();

   public SimpleTransferState(SimpleWalkingStateEnum transferStateEnum,
                              WalkingMessageHandler walkingMessageHandler,
                              HighLevelHumanoidControllerToolbox controllerToolbox,
                              SimpleControlManagerFactory managerFactory,
                              WalkingFailureDetectionControlModule failureDetectionControlModule,
                              YoRegistry parentRegistry)
   {
      super(transferStateEnum, parentRegistry);
      this.transferToSide = transferStateEnum.getTransferToSide();
      this.walkingMessageHandler = walkingMessageHandler;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.controllerToolbox = controllerToolbox;

      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
   }

   @Override
   public RobotSide getTransferToSide()
   {
      return transferToSide;
   }

   @Override
   public void doAction(double timeInState)
   {
      // Always do this so that when a foot slips or is loaded in the air, the height gets adjusted.
      //      comHeightManager.setSupportLeg(transferToSide.getOppositeSide());

      balanceManager.computeNormalizedEllipticICPError(transferToSide);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      if (balanceManager.isICPPlanDone())
      {
         balanceManager.getCapturePoint(capturePoint2d);
         FrameConvexPolygon2DReadOnly supportPolygonInWorld = controllerToolbox.getBipedSupportPolygons().getSupportPolygonInWorld();
         FrameConvexPolygon2DReadOnly nextPolygonInWorld = failureDetectionControlModule.getCombinedFootPolygonWithNextFootstep();

         double distanceToSupport = supportPolygonInWorld.distance(capturePoint2d);
         boolean isICPInsideNextSupportPolygon = nextPolygonInWorld.isPointInside(capturePoint2d);

         if (distanceToSupport > balanceManager.getICPDistanceOutsideSupportForStep() || (distanceToSupport == 0.0 && isICPInsideNextSupportPolygon))
            return true;
         else if (balanceManager.getNormalizedEllipticICPError() < 1.0)
            return true;
      }

      return false;
   }

   @Override
   public void onEntry()
   {
      updateICPPlan();

      if (walkingMessageHandler.hasUpcomingFootsteps())
      {
         walkingMessageHandler.peekTiming(0, stepTiming);
      }
      else
      {
         stepTiming.setTimings(Double.NaN, Double.NaN);
      }

      double extraToeOffHeight = 0.0;

      balanceManager.getFinalDesiredCoMPosition(desiredCoM);
      NewTransferToAndNextFootstepsData transferToAndNextFootstepsData = walkingMessageHandler.createTransferToAndNextFootstepDataForDoubleSupport(
            transferToSide);
      transferToAndNextFootstepsData.setComAtEndOfState(desiredCoM);
   }

   protected void updateICPPlan()
   {
      balanceManager.clearICPPlan();
      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      if (walkingMessageHandler.hasUpcomingFootsteps())
      {
         walkingMessageHandler.peekFootstep(0, nextFootstep);
         failureDetectionControlModule.setNextFootstep(nextFootstep);
      }
      else
      {
         failureDetectionControlModule.setNextFootstep(null);
      }

      double transferTime = walkingMessageHandler.getNextTransferTime();
      pelvisOrientationManager.setTrajectoryTime(transferTime);
   }

   public boolean isInitialTransfer()
   {
      return getPreviousWalkingStateEnum() == SimpleWalkingStateEnum.STANDING;
   }
}