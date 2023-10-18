package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FlamingoStanceState extends SingleSupportState
{
   private final FramePoint2D capturePoint2d = new FramePoint2D();
   private final YoBoolean loadFoot;
   private final YoDouble loadFootStartTime;
   private final YoDouble loadFootDuration;
   private final YoDouble loadFootTransferDuration;

   private final BipedSupportPolygons bipedSupportPolygons;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CenterOfMassHeightManager comHeightManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;

   private final FootstepTiming footstepTiming = new FootstepTiming();
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   public FlamingoStanceState(WalkingStateEnum stateEnum,
                              WalkingControllerParameters walkingControllerParameters,
                              WalkingMessageHandler walkingMessageHandler,
                              HighLevelHumanoidControllerToolbox controllerToolbox,
                              HighLevelControlManagerFactory managerFactory,
                              WalkingFailureDetectionControlModule failureDetectionControlModule,
                              YoRegistry parentRegistry)
   {
      super(stateEnum, walkingMessageHandler, controllerToolbox, managerFactory, parentRegistry);
      this.controllerToolbox = controllerToolbox;

      bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();
      this.failureDetectionControlModule = failureDetectionControlModule;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();

      String namePrefix = supportSide.getOppositeSide().getLowerCaseName();
      loadFoot = new YoBoolean(namePrefix + "LoadFoot", registry);
      loadFootStartTime = new YoDouble(namePrefix + "LoadFootStartTime", registry);
      loadFootDuration = new YoDouble(namePrefix + "LoadFootDuration", registry);
      loadFootTransferDuration = new YoDouble(namePrefix + "LoadFootTransferDuration", registry);

      loadFoot.set(false);
      loadFootDuration.set(walkingControllerParameters.getLoadFootDuration()); //1.2
      loadFootTransferDuration.set(walkingControllerParameters.getLoadFootTransferDuration()); //0.8
   }

   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeFlamingoStateICPPlan();

      super.doAction(timeInState);

      if (loadFoot.getBooleanValue() && loadFootStartTime.isNaN())
         loadFootStartTime.set(timeInState);

      while (walkingMessageHandler.hasFootTrajectoryForFlamingoStance(swingSide))
         feetManager.handleFootTrajectoryCommand(walkingMessageHandler.pollFootTrajectoryForFlamingoStance(swingSide));
      while (walkingMessageHandler.hasLegTrajectoryForFlamingoStance(swingSide))
         feetManager.handleLegTrajectoryCommand(walkingMessageHandler.pollLegTrajectoryForFlamingoStance(swingSide));

      capturePoint2d.setIncludingFrame(balanceManager.getCapturePoint());

      boolean icpErrorIsTooLarge = balanceManager.getICPErrorMagnitude() > 0.05;

      if (icpErrorIsTooLarge)
      {
         FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();
         FrameConvexPolygon2D combinedFootPolygon = failureDetectionControlModule.getCombinedFootPolygon();
         if (!supportPolygonInWorld.isPointInside(capturePoint2d, 2.0e-2) && combinedFootPolygon.isPointInside(capturePoint2d))
         {
            feetManager.requestMoveStraightTouchdownForDisturbanceRecovery(swingSide);
            initiateFootLoadingProcedure(swingSide);
            balanceManager.requestICPPlannerToHoldCurrentCoMInNextDoubleSupport();
         }
         else if (!supportPolygonInWorld.isPointInside(capturePoint2d, failureDetectionControlModule.getICPDistanceFromFootPolygonThreshold()))
         {
            failureDetectionControlModule.reportRobotIsFalling();
         }
      }

      walkingMessageHandler.clearFlamingoCommands(supportSide);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      if (super.isDone(timeInState))
         return true;

      if (loadFoot.getBooleanValue() && timeInState > loadFootStartTime.getDoubleValue() + loadFootDuration.getDoubleValue())
      {
         loadFoot.set(false);
         return true;
      }

      return false;
   }

   @Override
   protected boolean hasMinimumTimePassed(double timeInState)
   {
      double minimumSwingTime = walkingMessageHandler.getDefaultSwingTime() * minimumSwingFraction.getDoubleValue();

      return timeInState > minimumSwingTime;
   }

   @Override
   public void onEntry()
   {
      super.onEntry();

      balanceManager.enablePelvisXYControl();
      if (walkingMessageHandler.hasFootTrajectoryForFlamingoStance(swingSide))
         feetManager.handleFootTrajectoryCommand(walkingMessageHandler.pollFootTrajectoryForFlamingoStance(swingSide));
      else if (walkingMessageHandler.hasLegTrajectoryForFlamingoStance(swingSide))
         feetManager.handleLegTrajectoryCommand(walkingMessageHandler.pollLegTrajectoryForFlamingoStance(swingSide));
      else
         throw new IllegalStateException("Entered flamingo stance state without Foot/LegTrajectoryCommand?!");

      double swingTime = Double.POSITIVE_INFINITY;
      double initialTransferTime = walkingMessageHandler.getInitialTransferTime();
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      footstepTiming.setTimings(swingTime, initialTransferTime);

      Footstep footstep = walkingMessageHandler.getFootstepAtCurrentLocation(swingSide);
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.addFootstepToPlan(footstep, footstepTiming);
      balanceManager.setICPPlanSupportSide(supportSide);
      balanceManager.initializeICPPlanForSingleSupport();

      pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(supportSide);
      comHeightManager.setSupportLeg(getSupportSide());
      loadFoot.set(false);
      loadFootStartTime.setToNaN();

      controllerToolbox.updateContactPointsForUpcomingFootstep(walkingMessageHandler.getFootstepAtCurrentLocation(swingSide));
   }

   @Override
   public void onExit(double timeInState)
   {
      super.onExit(timeInState);

      feetManager.initializeContactStatesForDoubleSupport(swingSide);

      balanceManager.disablePelvisXYControl();
   }

   @Override
   public boolean isStateSafeToConsumePelvisTrajectoryCommand()
   {
      return true;
   }

   @Override
   public boolean isStateSafeToConsumeManipulationCommands()
   {
      return true;
   }

   @Override
   public void handleFootLoadBearingCommand(FootLoadBearingCommand command)
   {
      if (command.getRequest(swingSide) == LoadBearingRequest.LOAD)
         initiateFootLoadingProcedure(swingSide);
   }

   private void initiateFootLoadingProcedure(RobotSide swingSide)
   {
      loadFoot.set(true);
      balanceManager.clearICPPlan();

      double swingTime = loadFootDuration.getDoubleValue();
      double transferTime = loadFootTransferDuration.getDoubleValue();
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      footstepTiming.setTimings(swingTime, transferTime);

      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(swingSide), footstepTiming);
      balanceManager.setICPPlanSupportSide(supportSide);
      balanceManager.initializeICPPlanForSingleSupport();

      balanceManager.freezePelvisXYControl();
   }
}