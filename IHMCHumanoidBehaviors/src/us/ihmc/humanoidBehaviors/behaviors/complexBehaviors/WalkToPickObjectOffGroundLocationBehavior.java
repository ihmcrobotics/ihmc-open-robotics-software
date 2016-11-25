package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkToPickObjectOffGroundLocationBehavior.WalkState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidBehaviors.taskExecutor.ArmTrajectoryTask;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;

public class WalkToPickObjectOffGroundLocationBehavior extends StateMachineBehavior<WalkState>
{

   public enum WalkState
   {
      GET_READY_TO_WALK, WALK
   }

   private final WalkToLocationBehavior walkToLocationBehavior;
   private final ReferenceFrame midZupFrame;

   private final HumanoidReferenceFrames referenceFrames;
   private Point3d pickUpLocation = null;
   private final double standingDistance = 0.4;
   private final AtlasPrimitiveActions atlasPrimitiveActions;

   public WalkToPickObjectOffGroundLocationBehavior(DoubleYoVariable yoTime, HumanoidReferenceFrames referenceFrames,
         CommunicationBridge outgoingCommunicationBridge, WholeBodyControllerParameters wholeBodyControllerParameters, FullHumanoidRobotModel fullRobotModel,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("WalkState", WalkState.class, yoTime, outgoingCommunicationBridge);

      this.atlasPrimitiveActions = atlasPrimitiveActions;
      this.referenceFrames = referenceFrames;
      midZupFrame = referenceFrames.getMidFeetZUpFrame();

      walkToLocationBehavior = new WalkToLocationBehavior(outgoingCommunicationBridge, fullRobotModel, referenceFrames,
            wholeBodyControllerParameters.getWalkingControllerParameters());

      setupStateMachine();
   }

   private void setupStateMachine()
   {

      double[] rightHandWiderHomeJointAngles = new double[] {-0.785398, 0.5143374964757462, 2.2503094898479272, -2.132492022530739, -0.22447272781774874,
            -0.4780687104960028, -0.24919417978503655};

      ArmTrajectoryMessage widerHome = new ArmTrajectoryMessage(RobotSide.RIGHT, 2, rightHandWiderHomeJointAngles);

      ArmTrajectoryTask<WalkState> rightArmHomeTask = new ArmTrajectoryTask<WalkState>(WalkState.GET_READY_TO_WALK, widerHome,
            atlasPrimitiveActions.rightArmTrajectoryBehavior);

      BehaviorAction<WalkState> walkToBallTask = new BehaviorAction<WalkState>(WalkState.WALK, walkToLocationBehavior)
      {

         @Override
         protected void setBehaviorInput()
         {
            walkToLocationBehavior.setTarget(getoffsetPoint());
         }
      };

      statemachine.addStateWithDoneTransition(rightArmHomeTask, WalkState.WALK);
      statemachine.addState(walkToBallTask);
      statemachine.setStartState(WalkState.GET_READY_TO_WALK);
   }

   private FramePose2d getoffsetPoint()
   {

      FramePoint2d ballPosition2d = new FramePoint2d(ReferenceFrame.getWorldFrame(), pickUpLocation.getX(), pickUpLocation.getY());
      FramePoint2d robotPosition = new FramePoint2d(midZupFrame, 0.0, 0.0);
      robotPosition.changeFrame(ReferenceFrame.getWorldFrame());
      FrameVector2d walkingDirection = new FrameVector2d(ReferenceFrame.getWorldFrame());
      walkingDirection.set(ballPosition2d);
      walkingDirection.sub(robotPosition);
      walkingDirection.normalize();
      double walkingYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());

      //get a point offset from the ball
      double x = ballPosition2d.getX() - walkingDirection.getX() * standingDistance;
      double y = ballPosition2d.getY() - walkingDirection.getY() * standingDistance;
      double rotationAngle = Math.toRadians(55);
      //rotate that point around the ball so that the robot stands to the side.

      double newX = ballPosition2d.getX() + (x - ballPosition2d.getX()) * Math.cos(rotationAngle) - (y - ballPosition2d.getY()) * Math.sin(rotationAngle);
      double newY = ballPosition2d.getY() + (x - ballPosition2d.getX()) * Math.sin(rotationAngle) + (y - ballPosition2d.getY()) * Math.cos(rotationAngle);

      FramePose2d poseToWalkTo = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(newX, newY), walkingYaw);
      return poseToWalkTo;
   }

   public void setPickUpLocation(Point3d grabLocation)
   {
      this.pickUpLocation = grabLocation;
   }

   @Override
   public void onBehaviorExited()
   {
      pickUpLocation = null;
   }
}