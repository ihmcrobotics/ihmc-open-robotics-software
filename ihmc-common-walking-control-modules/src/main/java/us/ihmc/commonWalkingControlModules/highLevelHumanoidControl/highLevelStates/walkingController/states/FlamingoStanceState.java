package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.legConfiguration.LegConfigurationManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
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
   private final LegConfigurationManager legConfigurationManager;

   private final FootstepTiming footstepTiming = new FootstepTiming();
   private final FootstepShiftFractions footstepShiftFractions = new FootstepShiftFractions();
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   public FlamingoStanceState(WalkingStateEnum stateEnum, WalkingMessageHandler walkingMessageHandler, HighLevelHumanoidControllerToolbox controllerToolbox,
                              HighLevelControlManagerFactory managerFactory, WalkingFailureDetectionControlModule failureDetectionControlModule,
                              YoVariableRegistry parentRegistry)
   {
      super(stateEnum, walkingMessageHandler, controllerToolbox, managerFactory, parentRegistry);
      this.controllerToolbox = controllerToolbox;

      bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();
      this.failureDetectionControlModule = failureDetectionControlModule;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
      legConfigurationManager = managerFactory.getOrCreateLegConfigurationManager();

      String namePrefix = supportSide.getOppositeSide().getLowerCaseName();
      loadFoot = new YoBoolean(namePrefix + "LoadFoot", registry);
      loadFootStartTime = new YoDouble(namePrefix + "LoadFootStartTime", registry);
      loadFootDuration = new YoDouble(namePrefix + "LoadFootDuration", registry);
      loadFootTransferDuration = new YoDouble(namePrefix + "LoadFootTransferDuration", registry);

      loadFoot.set(false);
      loadFootDuration.set(1.2);
      loadFootTransferDuration.set(0.8);
   }

   @Override
   public void doAction(double timeInState)
   {
      super.doAction(timeInState);

      if (loadFoot.getBooleanValue() && loadFootStartTime.isNaN())
         loadFootStartTime.set(timeInState);

      if (walkingMessageHandler.hasFootTrajectoryForFlamingoStance(swingSide))
      {
         while (walkingMessageHandler.hasFootTrajectoryForFlamingoStance(swingSide))
            feetManager.handleFootTrajectoryCommand(walkingMessageHandler.pollFootTrajectoryForFlamingoStance(swingSide));
      }

      balanceManager.getCapturePoint(capturePoint2d);

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
      }

      walkingMessageHandler.clearFootTrajectory(supportSide);
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
      double minimumSwingTime;
      if (balanceManager.isRecoveringFromDoubleSupportFall())
         minimumSwingTime = 0.15;
      else
         minimumSwingTime = walkingMessageHandler.getDefaultSwingTime() * minimumSwingFraction.getDoubleValue();

      return timeInState > minimumSwingTime;
   }

   @Override
   public void onEntry()
   {
      super.onEntry();

      balanceManager.enablePelvisXYControl();
      balanceManager.setNextFootstep(null);
      feetManager.handleFootTrajectoryCommand(walkingMessageHandler.pollFootTrajectoryForFlamingoStance(swingSide));

      double swingTime = Double.POSITIVE_INFINITY;
      double initialTransferTime = walkingMessageHandler.getInitialTransferTime();
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      double finalTransferSplitFraction = walkingMessageHandler.getFinalTransferSplitFraction();
      double finalTransferWeightDistribution = walkingMessageHandler.getFinalTransferWeightDistribution();
      footstepTiming.setTimings(swingTime, initialTransferTime);

      double swingDurationShiftFraction = walkingMessageHandler.getDefaultSwingDurationShiftFraction();
      double swingSplitFraction = walkingMessageHandler.getDefaultSwingSplitFraction();
      double transferSplitFraction = walkingMessageHandler.getDefaultTransferSplitFraction();
      footstepShiftFractions.setShiftFractions(swingDurationShiftFraction, swingSplitFraction, transferSplitFraction);

      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(swingSide), footstepTiming, footstepShiftFractions);
      balanceManager.setICPPlanSupportSide(supportSide);
      balanceManager.setFinalTransferSplitFraction(finalTransferSplitFraction);
      balanceManager.setFinalTransferWeightDistribution(finalTransferWeightDistribution);
      balanceManager.initializeICPPlanForSingleSupport(swingTime, initialTransferTime, finalTransferTime);

      pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(supportSide);
      comHeightManager.setSupportLeg(getSupportSide());
      loadFoot.set(false);
      loadFootStartTime.setToNaN();

      legConfigurationManager.startSwing(swingSide);
      controllerToolbox.updateContactPointsForUpcomingFootstep(walkingMessageHandler.getFootstepAtCurrentLocation(swingSide));
   }

   @Override
   public void onExit()
   {
      super.onExit();

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

      double finalTransferWeightDistribution = walkingMessageHandler.getFinalTransferWeightDistribution();
      double finalTransferSplitFraction = walkingMessageHandler.getFinalTransferSplitFraction();
      double swingDurationShiftFraction = walkingMessageHandler.getDefaultSwingDurationShiftFraction();
      double swingSplitFraction = walkingMessageHandler.getDefaultSwingSplitFraction();
      double transferSplitFraction = walkingMessageHandler.getDefaultTransferSplitFraction();
      footstepShiftFractions.setShiftFractions(swingDurationShiftFraction, swingSplitFraction, transferSplitFraction);

      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.setFinalTransferSplitFraction(finalTransferSplitFraction);
      balanceManager.setFinalTransferWeightDistribution(finalTransferWeightDistribution);
      balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(swingSide), footstepTiming, footstepShiftFractions);
      balanceManager.setUpcomingFootstep(walkingMessageHandler.getFootstepAtCurrentLocation(swingSide));
      balanceManager.setICPPlanSupportSide(supportSide);
      balanceManager.initializeICPPlanForSingleSupport(swingTime, transferTime, finalTransferTime);

      balanceManager.freezePelvisXYControl();
   }
}