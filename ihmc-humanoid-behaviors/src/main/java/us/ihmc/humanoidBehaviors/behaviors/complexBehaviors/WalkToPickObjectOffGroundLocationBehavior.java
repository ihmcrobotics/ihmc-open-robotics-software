package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkToPickObjectOffGroundLocationBehavior.WalkState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidBehaviors.taskExecutor.ArmTrajectoryTask;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkToPickObjectOffGroundLocationBehavior extends StateMachineBehavior<WalkState>
{

   public enum WalkState
   {
      GET_READY_TO_WALK, WALK
   }

   private final WalkToLocationBehavior walkToLocationBehavior;
   private final ReferenceFrame midZupFrame;

   private final HumanoidReferenceFrames referenceFrames;
   private Point3D pickUpLocation = null;
   private final double standingDistance = 0.4;
   private final AtlasPrimitiveActions atlasPrimitiveActions;

   public WalkToPickObjectOffGroundLocationBehavior(YoDouble yoTime, HumanoidReferenceFrames referenceFrames,
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

      ArmTrajectoryMessage widerHome = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 2, rightHandWiderHomeJointAngles);

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

   private FramePose2D getoffsetPoint()
   {

      FramePoint2D ballPosition2d = new FramePoint2D(ReferenceFrame.getWorldFrame(), pickUpLocation.getX(), pickUpLocation.getY());
      FramePoint2D robotPosition = new FramePoint2D(midZupFrame, 0.0, 0.0);
      robotPosition.changeFrame(ReferenceFrame.getWorldFrame());
      FrameVector2D walkingDirection = new FrameVector2D(ReferenceFrame.getWorldFrame());
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

      FramePose2D poseToWalkTo = new FramePose2D(ReferenceFrame.getWorldFrame(), new Point2D(newX, newY), walkingYaw);
      return poseToWalkTo;
   }

   public void setPickUpLocation(Point3D grabLocation)
   {
      this.pickUpLocation = grabLocation;
   }

   @Override
   public void onBehaviorExited()
   {
      pickUpLocation = null;
   }
}