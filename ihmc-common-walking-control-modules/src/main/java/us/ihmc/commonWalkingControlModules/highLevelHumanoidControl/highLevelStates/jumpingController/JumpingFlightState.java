package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.jumpingController;

import us.ihmc.commonWalkingControlModules.capturePoint.JumpingBalanceManager;
import us.ihmc.commonWalkingControlModules.controlModules.WalkingFailureDetectionControlModule;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JumpingFlightState extends JumpingState
{
   private final JumpingControllerToolbox controllerToolbox;
   private final WalkingFailureDetectionControlModule failureDetectionControlModule;

   private final JumpingBalanceManager balanceManager;
   private final JumpingFeetManager feetManager;

   private final JumpingParameters jumpingParameters;
   private final JumpingGoalHandler jumpingGoalHandler;

   private final RigidBodyTransform takeOffTransform = new RigidBodyTransform();
   private final ReferenceFrame takeOffFrame;
   private final JumpingGoalFootholdCalculator jumpingGoalFootholdCalculator = new JumpingGoalFootholdCalculator();

   private final JumpingGoal jumpingGoal = new JumpingGoal();

   private final YoDouble originalGoalLength = new YoDouble("originalGoalLength", registry);
   private final YoDouble adjustedGoalLength = new YoDouble("adjustedGoalLength", registry);

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

      takeOffFrame = new ReferenceFrame("takeOffFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(takeOffTransform);
         }
      };
   }

   @Override
   public void doAction(double timeInState)
   {
      updateJumpingGoalPoseFromTouchdownPosition();

      for (RobotSide robotSide : RobotSide.values)
      {
         footGoalPose.setIncludingFrame(jumpingGoalFootholdCalculator.getFootGoalPose(robotSide));
         footGoalPose.changeFrame(touchdownCoMFrame);

         feetManager.adjustSwingTrajectory(robotSide, footGoalPose);

         balanceManager.setSwingFootTrajectory(robotSide, feetManager.getSwingTrajectory(robotSide));
      }

      balanceManager.computeCoMPlanForJumping(jumpingGoal);
   }

   private final FramePose3D footGoalPose = new FramePose3D();
   private final FramePose3D touchdownCoMPose = new FramePose3D();
   private final PoseReferenceFrame touchdownCoMFrame = new PoseReferenceFrame("touchdownCoMFrame", ReferenceFrame.getWorldFrame());

   @Override
   public void onEntry()
   {
      takeOffTransform.set(controllerToolbox.getReferenceFrames().getMidFeetZUpFrame().getTransformToWorldFrame());

      jumpingGoalHandler.pollNextJumpingGoal(jumpingGoal);
      balanceManager.setMinimizeAngularMomentumRate(false);

      originalGoalLength.set(jumpingGoal.getGoalLength());

      // need to always update biped support polygons after a change to the contact states
      for (RobotSide robotSide : RobotSide.values)
         controllerToolbox.setFootContactStateFree(robotSide);
      controllerToolbox.updateBipedSupportPolygons();

      // FIXME need to re-target the orientation
      updateJumpingGoalPoseFromTouchdownPosition();
//      touchdownCoMPose.getOrientation().set(goalPose.getOrientation());

      // TODO trigger the swing in the feet manager
      balanceManager.initializeCoMPlanForFlight(jumpingGoal.getSupportDuration(), jumpingGoal.getFlightDuration());

      for (RobotSide robotSide : RobotSide.values)
      {
         footGoalPose.setIncludingFrame(jumpingGoalFootholdCalculator.getFootGoalPose(robotSide));
         footGoalPose.changeFrame(touchdownCoMFrame);

         double flightDuration;
         if (!Double.isNaN(jumpingGoal.getFlightDuration()))
            flightDuration = jumpingGoal.getFlightDuration();
         else
            flightDuration = jumpingParameters.getDefaultFlightDuration();

         feetManager.requestSwing(robotSide, footGoalPose, jumpingParameters.getSwingHeight(), flightDuration);
         balanceManager.setSwingFootTrajectory(robotSide, feetManager.getSwingTrajectory(robotSide));
         controllerToolbox.setFootContactStateFree(robotSide);
      }

//      if (pelvisOrientationManager != null)
//         pelvisOrientationManager.initializeStanding();

//      failureDetectionControlModule.setNextFootstep(null);
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

      double omega = Math.sqrt(controllerToolbox.getGravityZ() / touchdownCoMPose.getPosition().getZ());

      comVelocity.setIncludingFrame(controllerToolbox.getCenterOfMassJacobian().getCenterOfMassVelocity());
      comVelocity.changeFrame(touchdownCoMFrame);
      goalPoint.setToZero(touchdownCoMFrame);
      goalPoint.setX(comVelocity.getX() / omega);

      goalPoint.changeFrame(takeOffFrame);

      adjustedGoalLength.set(goalPoint.getX());

      jumpingGoal.setGoalLength(adjustedGoalLength.getDoubleValue());

      jumpingGoalFootholdCalculator.computeGoalPose(takeOffFrame, adjustedGoalLength.getDoubleValue(), width, jumpingGoal.getGoalHeight(), jumpingGoal.getGoalRotation());
   }

   private final FramePoint3D goalPoint = new FramePoint3D();
   private final FrameVector3D comVelocity = new FrameVector3D();


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