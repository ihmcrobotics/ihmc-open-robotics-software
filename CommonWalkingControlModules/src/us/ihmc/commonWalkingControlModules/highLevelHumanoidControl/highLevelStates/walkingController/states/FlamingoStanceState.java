package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controlModules.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.desiredFootStep.WalkingMessageHandler;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.CenterOfMassHeightManager;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.EndEffectorLoadBearingCommand;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.EndEffector;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.LoadBearingRequest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class FlamingoStanceState extends SingleSupportState
{
   private final FramePoint2d capturePoint2d = new FramePoint2d();
   private final BooleanYoVariable loadFoot = new BooleanYoVariable("loadFoot", registry);
   private final DoubleYoVariable loadFootStartTime = new DoubleYoVariable("loadFootStartTime", registry);
   private final DoubleYoVariable loadFootDuration = new DoubleYoVariable("loadFootDuration", registry);
   private final DoubleYoVariable loadFootTransferDuration = new DoubleYoVariable("loadFootTransferDuration", registry);

   private final BipedSupportPolygons bipedSupportPolygons;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final CenterOfMassHeightManager comHeightManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;

   public FlamingoStanceState(RobotSide supportSide, WalkingMessageHandler walkingMessageHandler, MomentumBasedController momentumBasedController,
         HighLevelControlManagerFactory managerFactory, WalkingFailureDetectionControlModule failureDetectionControlModule, YoVariableRegistry parentRegistry)
   {
      super(supportSide, WalkingStateEnum.getFlamingoSingleSupportState(supportSide), walkingMessageHandler, momentumBasedController, managerFactory,
            parentRegistry);

      bipedSupportPolygons = momentumBasedController.getBipedSupportPolygons();
      this.failureDetectionControlModule = failureDetectionControlModule;

      comHeightManager = managerFactory.getOrCreateCenterOfMassHeightManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();

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
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();

      balanceManager.enablePelvisXYControl();
      feetManager.handleFootTrajectoryCommand(walkingMessageHandler.pollFootTrajectoryForFlamingoStance(swingSide));

      pelvisOrientationManager.setToHoldCurrentDesiredInSupportFoot(supportSide);
      comHeightManager.setSupportLeg(getSupportSide());
      loadFoot.set(false);
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

   public void handleEndEffectorLoadBearingCommand(EndEffectorLoadBearingCommand command)
   {
      if (command.getRequest(swingSide, EndEffector.FOOT) == LoadBearingRequest.LOAD)
         initiateFootLoadingProcedure(swingSide);
   }

   private void initiateFootLoadingProcedure(RobotSide swingSide)
   {
      loadFoot.set(true);
      loadFootStartTime.set(getTimeInCurrentState());
      balanceManager.setSingleSupportTime(loadFootDuration.getDoubleValue());
      balanceManager.setDoubleSupportTime(loadFootTransferDuration.getDoubleValue());
      balanceManager.clearICPPlan();
      balanceManager.addFootstepToPlan(walkingMessageHandler.getFootstepAtCurrentLocation(swingSide));
      balanceManager.setICPPlanSupportSide(supportSide);
      balanceManager.initializeICPPlanForSingleSupport();
      
      balanceManager.freezePelvisXYControl();
   }
}