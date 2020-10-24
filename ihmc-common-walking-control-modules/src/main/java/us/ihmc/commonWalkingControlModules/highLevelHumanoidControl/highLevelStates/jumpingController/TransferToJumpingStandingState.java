package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.capturePoint.JumpingBalanceManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FeetManager;
import us.ihmc.commonWalkingControlModules.controlModules.pelvis.PelvisOrientationManager;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

public class TransferToJumpingStandingState extends JumpingState
{
   private final JumpingControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final JumpingBalanceManager balanceManager;
   private final PelvisOrientationManager pelvisOrientationManager;
   private final FeetManager feetManager;

   private final FramePoint3D leftFootPosition = new FramePoint3D();
   private final FramePoint3D rightFootPosition = new FramePoint3D();

   private final Point3D midFootPosition = new Point3D();

   public TransferToJumpingStandingState(JumpingControllerToolbox controllerToolbox, HighLevelControlManagerFactory managerFactory,
                                         JumpingBalanceManager balanceManager, WalkingFailureDetectionControlModule failureDetectionControlModule, YoRegistry parentRegistry)
   {
      super(JumpingStateEnum.TO_STANDING, parentRegistry);

      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.balanceManager = balanceManager;

      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeICPPlan();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return balanceManager.isICPPlanDone();
   }

   @Override
   public void onEntry()
   {
      balanceManager.clearICPPlan();

      JumpingStateEnum previousStateEnum = getPreviousJumpingStateEnum();

      // This can happen if walking is paused or aborted while the robot is on its toes already. In that case
      // restore the full foot contact.
      if (previousStateEnum != null && previousStateEnum.isDoubleSupport())
         feetManager.initializeContactStatesForDoubleSupport(null);

      controllerToolbox.updateBipedSupportPolygons(); // need to always update biped support polygons after a change to the contact states

      failureDetectionControlModule.setNextFootstep(null);

      double finalTransferTime = controllerToolbox.getFinalTransferTime();
      leftFootPosition.setToZero(controllerToolbox.getFullRobotModel().getSoleFrames().get(RobotSide.LEFT));
      rightFootPosition.setToZero(controllerToolbox.getFullRobotModel().getSoleFrames().get(RobotSide.RIGHT));
      leftFootPosition.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPosition.changeFrame(ReferenceFrame.getWorldFrame());
      midFootPosition.interpolate(leftFootPosition, rightFootPosition, 0.5);

      // Just standing in double support, do nothing
      pelvisOrientationManager.centerInMidFeetZUpFrame(finalTransferTime);
      balanceManager.setFinalTransferTime(finalTransferTime);
      balanceManager.initializeICPPlanForTransferToStanding();
   }
}