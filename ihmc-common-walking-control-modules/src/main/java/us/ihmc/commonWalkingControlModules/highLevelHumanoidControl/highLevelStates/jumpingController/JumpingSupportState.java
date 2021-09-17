package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.capturePoint.JumpingBalanceManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class JumpingSupportState extends JumpingState
{
   private final JumpingControllerToolbox controllerToolbox;
   private final JumpingParameters jumpingParameters;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final JumpingGoalHandler jumpingGoalHandler;
   private final JumpingGoal jumpingGoal = new JumpingGoal();

   private final JumpingBalanceManager balanceManager;
   private final JumpingPelvisOrientationManager pelvisOrientationManager;
   private final JumpingFeetManager feetManager;

   private final JumpingGoalFootholdCalculator jumpingGoalFootholdCalculator = new JumpingGoalFootholdCalculator();

   public JumpingSupportState(JumpingGoalHandler jumpingGoalHandler,
                              JumpingControllerToolbox controllerToolbox,
                              JumpingControlManagerFactory managerFactory,
                              JumpingParameters jumpingParameters,
                              WalkingFailureDetectionControlModule failureDetectionControlModule,
                              YoRegistry parentRegistry)
   {
      super(JumpingStateEnum.SUPPORT, parentRegistry);

      this.jumpingGoalHandler = jumpingGoalHandler;
      this.controllerToolbox = controllerToolbox;
      this.jumpingParameters = jumpingParameters;
      this.failureDetectionControlModule = failureDetectionControlModule;

      balanceManager = managerFactory.getOrCreateBalanceManager();
      pelvisOrientationManager = managerFactory.getOrCreatePelvisOrientationManager();
      feetManager = managerFactory.getOrCreateFeetManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      updateJumpingGoalPoseFromTouchdownPosition();

      for (RobotSide swingSide : RobotSide.values)
      {
         footGoalPose.setIncludingFrame(jumpingGoalFootholdCalculator.getFootGoalPose(swingSide));
         footGoalPose.changeFrame(touchdownCoMFrame);

         feetManager.updateSwingTrajectoryPreview(swingSide, footGoalPose);
         balanceManager.setSwingFootTrajectory(swingSide, feetManager.getSwingTrajectory(swingSide));
      }

      balanceManager.computeCoMPlanForJumping(jumpingGoal);
   }

   private final FramePose3D footGoalPose = new FramePose3D();
   private final FramePose3D touchdownCoMPose = new FramePose3D();
   private final PoseReferenceFrame touchdownCoMFrame = new PoseReferenceFrame("touchdownCoMFrame", ReferenceFrame.getWorldFrame());

   @Override
   public void onEntry()
   {
      jumpingGoalHandler.peekNextJumpingGoal(jumpingGoal);

      // need to always update biped support polygons after a change to the contact states
      controllerToolbox.updateBipedSupportPolygons();

      updateJumpingGoalPoseFromTouchdownPosition();

      for (RobotSide robotSide : RobotSide.values)
      {
         footGoalPose.setIncludingFrame(jumpingGoalFootholdCalculator.getFootGoalPose(robotSide));
         footGoalPose.changeFrame(touchdownCoMFrame);

         feetManager.initializeSwingTrajectoryPreview(robotSide, footGoalPose, jumpingParameters.getSwingHeight(), jumpingGoal.getFlightDuration());
         balanceManager.setSwingFootTrajectory(robotSide, feetManager.getSwingTrajectory(robotSide));
      }

      balanceManager.setDesiredCoMHeight(controllerToolbox.getJumpingHeight());
      balanceManager.initializeCoMPlanForSupport(jumpingGoal);
      balanceManager.setMinimizeAngularMomentumRate(true);

      if (pelvisOrientationManager != null)
         pelvisOrientationManager.initializeStanding();

//      failureDetectionControlModule.setNextFootstep(null);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      // TODO add a fallback finished. Also add criteria on "legs are straight" and "feet are unloaded"
      if (timeInState >= jumpingGoal.getSupportDuration())
         return true;

      return areLegsStraight();
   }

   private boolean areLegsStraight()
   {
      double minKneeAngle = Double.POSITIVE_INFINITY;
      for (RobotSide robotSide : RobotSide.values)
      {
         minKneeAngle = Math.min(minKneeAngle, controllerToolbox.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE_PITCH).getQ());
      }

      return minKneeAngle <= jumpingParameters.getMinKneeAngleForTakeOff();
   }

   private void updateJumpingGoalPoseFromTouchdownPosition()
   {
      double width;
      if (!Double.isNaN(jumpingGoal.getGoalFootWidth()))
         width = jumpingGoal.getGoalFootWidth();
      else
         width = jumpingParameters.getDefaultFootWidth();

      touchdownCoMPose.getPosition().set(balanceManager.getTouchdownCoMPosition());
      touchdownCoMFrame.setPoseAndUpdate(touchdownCoMPose);

      jumpingGoalFootholdCalculator.computeGoalPose(controllerToolbox.getReferenceFrames().getMidFeetZUpFrame(), jumpingGoal.getGoalLength(), width, jumpingGoal.getGoalHeight(),
                                                    jumpingGoal.getGoalRotation());
   }

}