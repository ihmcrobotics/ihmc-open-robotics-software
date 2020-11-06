package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import com.jme3.animation.Pose;
import us.ihmc.commonWalkingControlModules.capturePoint.JumpingBalanceManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlManager;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JumpingFlightState extends JumpingState
{
   private static final double swingHeight = 0.01;
   private final JumpingControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final JumpingBalanceManager balanceManager;
   private final JumpingFeetManager feetManager;

   private final JumpingParameters jumpingParameters;
   private final JumpingGoalHandler jumpingGoalHandler;

   private final JumpingGoal jumpingGoal = new JumpingGoal();

   public JumpingFlightState(JumpingGoalHandler jumpingGoalHandler,
                             JumpingControllerToolbox controllerToolbox,
                             JumpingControlManagerFactory managerFactory,
                             WalkingFailureDetectionControlModule failureDetectionControlModule,
                             JumpingParameters jumpingParameters,
                             YoRegistry parentRegistry)
   {
      super(JumpingStateEnum.FLIGHT, parentRegistry);

      this.jumpingGoalHandler = jumpingGoalHandler;
      this.controllerToolbox = controllerToolbox;
      this.failureDetectionControlModule = failureDetectionControlModule;
      this.jumpingParameters = jumpingParameters;

      balanceManager = managerFactory.getOrCreateBalanceManager();
      feetManager = managerFactory.getOrCreateFeetManager();
   }

   @Override
   public void doAction(double timeInState)
   {
      balanceManager.computeCoMPlanForJumping(jumpingGoal);
   }

   private final FramePose3D footGoalPose = new FramePose3D();
   private final FramePose3D midFootPose = new FramePose3D();
   private final FramePose3D goalPose = new FramePose3D();
   private final FramePose3D touchdownCoMPose = new FramePose3D();
   private final PoseReferenceFrame touchdownCoMFrame = new PoseReferenceFrame("touchdownCoMFrame", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame goalPoseFrame = new PoseReferenceFrame("goalPoseFrame", ReferenceFrame.getWorldFrame());

   @Override
   public void onEntry()
   {
      jumpingGoalHandler.pollNextJumpingGoal(jumpingGoal);
      balanceManager.setMinimizeAngularMomentumRate(false);

      // need to always update biped support polygons after a change to the contact states
      for (RobotSide robotSide : RobotSide.values)
         controllerToolbox.setFootContactStateFree(robotSide);
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

      touchdownCoMPose.getOrientation().set(goalPose.getOrientation());
      touchdownCoMPose.getPosition().set(balanceManager.getTouchdownCoMPosition());
      touchdownCoMFrame.setPoseAndUpdate(touchdownCoMPose);

      for (RobotSide robotSide : RobotSide.values)
      {
         footGoalPose.setToZero(goalPoseFrame);
         double width;
         if (!Double.isNaN(jumpingGoal.getGoalFootWidth()))
            width = robotSide.negateIfRightSide(0.5 * jumpingGoal.getGoalFootWidth());
         else
            width = robotSide.negateIfRightSide(0.5 * jumpingParameters.getDefaultFootWidth());

         footGoalPose.setY(width);
         footGoalPose.changeFrame(touchdownCoMFrame);

         correctTouchdownFootPose(footGoalPose);

         double flightDuration;
         if (!Double.isNaN(jumpingGoal.getFlightDuration()))
            flightDuration = jumpingGoal.getFlightDuration();
         else
            flightDuration = jumpingParameters.getDefaultFlightDuration();

         feetManager.requestSwing(robotSide, footGoalPose, swingHeight, flightDuration);
         controllerToolbox.setFootContactStateFree(robotSide);
      }

//      if (pelvisOrientationManager != null)
//         pelvisOrientationManager.initializeStanding();

//      failureDetectionControlModule.setNextFootstep(null);
   }

   private final FrameVector3D comVelocity = new FrameVector3D();

   private void correctTouchdownFootPose(FramePose3D footPose)
   {
      comVelocity.setIncludingFrame(controllerToolbox.getCenterOfMassJacobian().getCenterOfMassVelocity());
      comVelocity.changeFrame(touchdownCoMFrame);
      footPose.checkReferenceFrameMatch(touchdownCoMFrame);
      footPose.setX(comVelocity.getX() / controllerToolbox.getOmega0());
   }

   @Override
   public void onExit()
   {
      balanceManager.setMinimizeAngularMomentumRate(true);
   }

   private static final double minFractionThroughSwingForContact = 0.8;
   @Override
   public boolean isDone(double timeInState)
   {
      double desiredFlightDuration = Double.isNaN(jumpingGoal.getFlightDuration()) ? jumpingParameters.getDefaultFlightDuration() : jumpingGoal.getFlightDuration();
      if (timeInState < minFractionThroughSwingForContact * desiredFlightDuration)
         return false;

      boolean bothFeetHaveHitGround = true;
      for (RobotSide robotSide : RobotSide.values)
      {
         if (!controllerToolbox.getFootSwitches().get(robotSide).hasFootHitGround())
            bothFeetHaveHitGround = false;
         else if (!feetManager.getCurrentConstraintType(robotSide).isLoadBearing())
         {
            feetManager.setFlatFootContactState(robotSide);
            controllerToolbox.setFootContactStateFullyConstrained(robotSide);
            controllerToolbox.updateBipedSupportPolygons();
         }
      }


      return bothFeetHaveHitGround;
   }
}