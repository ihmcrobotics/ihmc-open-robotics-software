package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.capturePoint.JumpingBalanceManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

public class TransferToJumpingStandingState extends JumpingState
{
   private final JumpingControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final JumpingBalanceManager balanceManager;
   private final JumpingPelvisOrientationManager pelvisOrientationManager;
   private final JumpingFeetManager feetManager;

   private final FramePoint3D leftFootPosition = new FramePoint3D();
   private final FramePoint3D rightFootPosition = new FramePoint3D();

   private final Point3D midFootPosition = new Point3D();

   public TransferToJumpingStandingState(JumpingControllerToolbox controllerToolbox,
                                         JumpingControlManagerFactory managerFactory,
                                         WalkingFailureDetectionControlModule failureDetectionControlModule,
                                         YoRegistry parentRegistry)
   {
      super(JumpingStateEnum.TO_STANDING, parentRegistry);

      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;

      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeCoMPlanForStanding();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return balanceManager.isCoMPlanDone();
   }

   @Override
   public void onEntry()
   {
      for (RobotSide robotSide : RobotSide.values)
         feetManager.setFlatFootContactState(robotSide);

      balanceManager.clearICPPlan();
      balanceManager.setDesiredCoMHeight(controllerToolbox.getStandingHeight());

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
      balanceManager.initializeCoMPlanForTransferToStanding();
   }
}