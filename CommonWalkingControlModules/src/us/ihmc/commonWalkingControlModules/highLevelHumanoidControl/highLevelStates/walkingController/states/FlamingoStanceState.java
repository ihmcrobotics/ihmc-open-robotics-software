package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.kneeAngle.KneeAngleManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage.LoadBearingRequest;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class FlamingoStanceState extends SingleSupportState
{
   private final FramePoint2d capturePoint2d = new FramePoint2d();
   private final BooleanYoVariable loadFoot;
   private final DoubleYoVariable loadFootStartTime;
   private final DoubleYoVariable loadFootDuration;
   private final DoubleYoVariable loadFootTransferDuration;

   private final BipedSupportPolygons bipedSupportPolygons;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CenterOfMassHeightManager comHeightManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;
   private final KneeAngleManager kneeAngleManager;

   private final FootstepTiming footstepTiming = new FootstepTiming();

   public FlamingoStanceState(RobotSide supportSide, WalkingMessageHandler walkingMessageHandler, HighLevelHumanoidControllerToolbox controllerToolbox,
         HighLevelControlManagerFactory managerFactory, WalkingFailureDetectionControlModule failureDetectionControlModule, YoVariableRegistry parentRegistry)
   {
      super(supportSide, WalkingStateEnum.getFlamingoSingleSupportState(supportSide), walkingMessageHandler, controllerToolbox, managerFactory,
            parentRegistry);

      bipedSupportPolygons = controllerToolbox.getBipedSupportPolygons();
      this.failureDetectionControlModule = failureDetectionControlModule;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
      kneeAngleManager = managerFactory.getOrCreateKneeAngleManager();

      String namePrefix = supportSide.getOppositeSide().getLowerCaseName();
      loadFoot = new BooleanYoVariable(namePrefix + "LoadFoot", registry);
      loadFootStartTime = new DoubleYoVariable(namePrefix + "LoadFootStartTime", registry);
      loadFootDuration = new DoubleYoVariable(namePrefix + "LoadFootDuration", registry);
      loadFootTransferDuration = new DoubleYoVariable(namePrefix + "LoadFootTransferDuration", registry);

      loadFoot.set(false);
      loadFootDuration.set(1.2);
      loadFootTransferDuration.set(0.8);
   }

   @Override
   public void doAction()
   {
      super.doAction();

      if (walkingMessageHandler.hasFootTrajectoryForFlamingoStance(swingSide))
      {
         while(walkingMessageHandler.hasFootTrajectoryForFlamingoStance(swingSide))
            feetManager.handleFootTrajectoryCommand(walkingMessageHandler.pollFootTrajectoryForFlamingoStance(swingSide));
      }

      balanceManager.getCapturePoint(capturePoint2d);

      boolean icpErrorIsTooLarge = balanceManager.getICPErrorMagnitude() > 0.05;

      if (icpErrorIsTooLarge)
      {
         FrameConvexPolygon2d supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();
         FrameConvexPolygon2d combinedFootPolygon = failureDetectionControlModule.getCombinedFootPolygon();
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
   public boolean isDone()
   {
      if (super.isDone())
         return true;

      if (loadFoot.getBooleanValue() && getTimeInCurrentState() > loadFootStartTime.getDoubleValue() + loadFootDuration.getDoubleValue())
      {
         loadFoot.set(false);
         return true;
      }

      return false;
   }

   @Override
   protected boolean hasMinimumTimePassed()
   {
      double minimumSwingTime;
      if (balanceManager.isRecoveringFromDoubleSupportFall())
         minimumSwingTime = 0.15;
      else
         minimumSwingTime = walkingMessageHandler.getDefaultSwingTime() * minimumSwingFraction.getDoubleValue();

      return getTimeInCurrentState() > minimumSwingTime;
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();

      balanceManager.enablePelvisXYControl();
      balanceManager.setNextFootstep(null);
      feetManager.handleFootTrajectoryCommand(walkingMessageHandler.pollFootTrajectoryForFlamingoStance(swingSide));

      footstepTiming.setTimings(Double.POSITIVE_INFINITY, walkingMessageHandler.getDefaultTransferTime());
      balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(swingSide), footstepTiming);
      balanceManager.setICPPlanSupportSide(supportSide);
      double defaultSwingTime = Double.POSITIVE_INFINITY;
      double defaultTransferTime = walkingMessageHandler.getDefaultTransferTime();
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      balanceManager.initializeICPPlanForSingleSupport(defaultSwingTime, defaultTransferTime, finalTransferTime);

      pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(supportSide);
      comHeightManager.setSupportLeg(getSupportSide());
      loadFoot.set(false);

      kneeAngleManager.startSwing(swingSide);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();

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
      loadFootStartTime.set(getTimeInCurrentState());
      balanceManager.clearICPPlan();
      footstepTiming.setTimings(loadFootDuration.getDoubleValue(), loadFootTransferDuration.getDoubleValue());
      balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(swingSide), footstepTiming);
      balanceManager.setICPPlanSupportSide(supportSide);
      double defaultSwingTime = loadFootDuration.getDoubleValue();
      double defaultTransferTime = loadFootTransferDuration.getDoubleValue();
      double finalTransferTime = walkingMessageHandler.getFinalTransferTime();
      balanceManager.initializeICPPlanForSingleSupport(defaultSwingTime, defaultTransferTime, finalTransferTime);

      balanceManager.freezePelvisXYControl();
   }
}