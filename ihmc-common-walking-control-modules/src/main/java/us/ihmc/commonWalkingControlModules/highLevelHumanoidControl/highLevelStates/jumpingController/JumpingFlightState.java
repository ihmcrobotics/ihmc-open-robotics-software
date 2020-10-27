package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.capturePoint.JumpingBalanceManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JumpingFlightState extends JumpingState
{
   private static final double swingHeight = 0.2;
   private static final double defaultWidth = 0.2;
   private final JumpingControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final JumpingBalanceManager balanceManager;
   private final JumpingFeetManager feetManager;
   private final SideDependentList<RigidBodyControlManager> handManagers = new SideDependentList<>();

   private final JumpingCoPTrajectoryParameters jumpingCoPTrajectoryParameters;
   private final JumpingGoalHandler jumpingGoalHandler;

   private final JumpingGoal jumpingGoal = new JumpingGoal();

   public JumpingFlightState(JumpingGoalHandler jumpingGoalHandler,
                             JumpingControllerToolbox controllerToolbox,
                             JumpingControlManagerFactory managerFactory,
                             WalkingFailureDetectionControlModule failureDetectionControlModule,
                             JumpingCoPTrajectoryParameters jumpingCoPTrajectoryParameters,
                             YoRegistry parentRegistry)
   {
      super(JumpingStateEnum.FLIGHT, parentRegistry);

      this.jumpingGoalHandler = jumpingGoalHandler;
      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.jumpingCoPTrajectoryParameters = jumpingCoPTrajectoryParameters;

      balanceManager = managerFactory.getOrCreateBalanceManager();
      feetManager = managerFactory.getOrCreateFeetManager();

      RigidBodyBasics chest = controllerToolbox.getFullRobotModel().getChest();
      if (chest != null)
      {
         ReferenceFrame chestBodyFrame = chest.getBodyFixedFrame();

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyBasics hand = controllerToolbox.getFullRobotModel().getHand(robotSide);
            if (hand != null)
            {
               ReferenceFrame handControlFrame = controllerToolbox.getFullRobotModel().getHandControlFrame(robotSide);
               RigidBodyControlManager handManager = managerFactory.getOrCreateRigidBodyManager(hand, chest, handControlFrame, chestBodyFrame);
               handManagers.put(robotSide, handManager);
            }
         }
      }
   }

   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeCoMPlanForJumping(jumpingGoal);
   }

   private final FramePose3D footGoalPose = new FramePose3D();
   private final FramePose3D midFootPose = new FramePose3D();
   private final FramePose3D goalPose = new FramePose3D();
   private final PoseReferenceFrame goalPoseFrame = new PoseReferenceFrame("goalPoseFrame", ReferenceFrame.getWorldFrame());

   @Override
   public void onEntry()
   {
      jumpingGoalHandler.pollNextJumpingGoal(jumpingGoal);

      // need to always update biped support polygons after a change to the contact states
      controllerToolbox.updateBipedSupportPolygons();

      // TODO trigger the swing in the feet manager
      balanceManager.initializeCoMPlanForFlight(jumpingGoal);

      midFootPose.setToZero(controllerToolbox.getReferenceFrames().getMidFeetUnderPelvisFrame());
      goalPose.setIncludingFrame(midFootPose);
      goalPose.setX(jumpingGoal.getGoalLength());
      if (!Double.isNaN(jumpingGoal.getGoalHeight()))
         goalPose.setZ(jumpingGoal.getGoalHeight());
      if (!Double.isNaN(jumpingGoal.getGoalRotation()))
         goalPose.getOrientation().setToYawOrientation(jumpingGoal.getGoalRotation());
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());
      goalPoseFrame.setPoseAndUpdate(goalPose);

      for (RobotSide robotSide : RobotSide.values)
      {
         footGoalPose.setToZero(goalPoseFrame);
         double width;
         if (!Double.isNaN(jumpingGoal.getGoalFootWidth()))
            width = robotSide.negateIfRightSide(0.5 * jumpingGoal.getGoalFootWidth());
         else
            width = robotSide.negateIfRightSide(0.5 * jumpingCoPTrajectoryParameters.getDefaultFootWidth());

         footGoalPose.setY(width);
         footGoalPose.changeFrame(ReferenceFrame.getWorldFrame());

         double flightDuration;
         if (!Double.isNaN(jumpingGoal.getFlightDuration()))
            flightDuration = jumpingGoal.getFlightDuration();
         else
            flightDuration = jumpingCoPTrajectoryParameters.getDefaultFlightDuration();
         feetManager.requestSwing(robotSide, footGoalPose, swingHeight, flightDuration);
         controllerToolbox.setFootContactStateFree(robotSide);
      }

//      if (pelvisOrientationManager != null)
//         pelvisOrientationManager.initializeStanding();

//      failureDetectionControlModule.setNextFootstep(null);
   }



   @Override
   public void onExit()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (handManagers.get(robotSide) != null)
            handManagers.get(robotSide).prepareForLocomotion();
      }

      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.IN_MOTION);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      // TODO check for both feet being in contact
      return true;
   }
}