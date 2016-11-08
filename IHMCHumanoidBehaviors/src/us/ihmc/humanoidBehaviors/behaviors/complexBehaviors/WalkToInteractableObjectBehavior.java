package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

import com.jme3.math.Vector3f;

import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.WalkToInteractableObjectBehavior.WalkToObjectState;
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

public class WalkToInteractableObjectBehavior extends StateMachineBehavior<WalkToObjectState>
{
   
   protected Vector3f walkToPoint1;
   protected Vector3f walkToPoint2;
   ResetRobotBehavior reset;
      
   public enum WalkToObjectState
   {
      GET_READY_TO_WALK, WALK_TO_POINT_1, WALK_TO_POINT_2
   }

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   public WalkToInteractableObjectBehavior(DoubleYoVariable yoTime, CommunicationBridge outgoingCommunicationBridge, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("WalkState", WalkToObjectState.class, yoTime, outgoingCommunicationBridge);
      reset = new ResetRobotBehavior(outgoingCommunicationBridge, yoTime);
      this.atlasPrimitiveActions = atlasPrimitiveActions;
     
      setupStateMachine();
   }

   private void setupStateMachine()
   {

      BehaviorAction<WalkToObjectState> resetRobot = new BehaviorAction<WalkToObjectState>(WalkToObjectState.GET_READY_TO_WALK,reset);
      
      BehaviorAction<WalkToObjectState> walkToPoint1Task = new BehaviorAction<WalkToObjectState>(WalkToObjectState.WALK_TO_POINT_1, atlasPrimitiveActions.walkToLocationBehavior)
      {
//
//         @Override
//         protected void setBehaviorInput()
//         {
//
//            FramePoint2d ballPosition2d = new FramePoint2d(ReferenceFrame.getWorldFrame(), pickUpLocation.getX(), pickUpLocation.getY());
//            FramePoint2d robotPosition = new FramePoint2d(midZupFrame, 0.0, 0.0);
//            robotPosition.changeFrame(ReferenceFrame.getWorldFrame());
//            FrameVector2d walkingDirection = new FrameVector2d(ReferenceFrame.getWorldFrame());
//            walkingDirection.set(ballPosition2d);
//            walkingDirection.sub(robotPosition);
//            walkingDirection.normalize();
//            double walkingYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());
//
//            //get a point offset from the ball
//            double x = ballPosition2d.getX() - walkingDirection.getX() * standingDistance;
//            double y = ballPosition2d.getY() - walkingDirection.getY() * standingDistance;
//            double rotationAngle = Math.toRadians(55);
//            //rotate that point around the ball so that the robot stands to the side.
//
//            double newX = ballPosition2d.getX() + (x - ballPosition2d.getX()) * Math.cos(rotationAngle) - (y - ballPosition2d.getY()) * Math.sin(rotationAngle);
//            double newY = ballPosition2d.getY() + (x - ballPosition2d.getX()) * Math.sin(rotationAngle) + (y - ballPosition2d.getY()) * Math.cos(rotationAngle);
//
//            FramePose2d poseToWalkTo = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(newX, newY), walkingYaw);
//            return poseToWalkTo;
//         }
      };

//      statemachine.addStateWithDoneTransition(rightArmHomeTask, WalkToObjectState.WALK);
//      statemachine.addState(walkToBallTask);
      statemachine.setCurrentState(WalkToObjectState.GET_READY_TO_WALK);
   }

//   private FramePose2d getoffsetPoint()
//   {
//
//      FramePoint2d ballPosition2d = new FramePoint2d(ReferenceFrame.getWorldFrame(), pickUpLocation.getX(), pickUpLocation.getY());
//      FramePoint2d robotPosition = new FramePoint2d(midZupFrame, 0.0, 0.0);
//      robotPosition.changeFrame(ReferenceFrame.getWorldFrame());
//      FrameVector2d walkingDirection = new FrameVector2d(ReferenceFrame.getWorldFrame());
//      walkingDirection.set(ballPosition2d);
//      walkingDirection.sub(robotPosition);
//      walkingDirection.normalize();
//      double walkingYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());
//
//      //get a point offset from the ball
//      double x = ballPosition2d.getX() - walkingDirection.getX() * standingDistance;
//      double y = ballPosition2d.getY() - walkingDirection.getY() * standingDistance;
//      double rotationAngle = Math.toRadians(55);
//      //rotate that point around the ball so that the robot stands to the side.
//
//      double newX = ballPosition2d.getX() + (x - ballPosition2d.getX()) * Math.cos(rotationAngle) - (y - ballPosition2d.getY()) * Math.sin(rotationAngle);
//      double newY = ballPosition2d.getY() + (x - ballPosition2d.getX()) * Math.sin(rotationAngle) + (y - ballPosition2d.getY()) * Math.cos(rotationAngle);
//
//      FramePose2d poseToWalkTo = new FramePose2d(ReferenceFrame.getWorldFrame(), new Point2d(newX, newY), walkingYaw);
//      return poseToWalkTo;
//   }


   @Override
   public void doPostBehaviorCleanup()
   {
      super.doPostBehaviorCleanup();
   }
}