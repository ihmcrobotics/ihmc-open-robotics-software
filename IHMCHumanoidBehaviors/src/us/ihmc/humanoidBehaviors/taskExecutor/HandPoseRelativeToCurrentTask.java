package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class HandPoseRelativeToCurrentTask extends BehaviorTask
{
   private final HandPoseBehavior handPoseBehavior;

   private final RobotSide robotSide;
   private final FullRobotModel fullRobotModel;
   private final double trajectoryTime;

   private final double distanceToMove;
   private final Vector3d directionToMoveInWorld;

   private final boolean stopHandIfCollision;

   public HandPoseRelativeToCurrentTask(RobotSide robotSide, double distanceToMoveAlongCurrentOrientation, FullRobotModel fullRobotModel, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior,
         double trajectoryTime)
   {
      this(robotSide, null, distanceToMoveAlongCurrentOrientation, fullRobotModel, yoTime, handPoseBehavior, trajectoryTime, false);
   }
   
   public HandPoseRelativeToCurrentTask(RobotSide robotSide, Vector3d directionToMoveInWorld, double distanceToMove, FullRobotModel fullRobotModel, DoubleYoVariable yoTime,
         HandPoseBehavior handPoseBehavior, double trajectoryTime, boolean stopHandIfCollision)
   {
      super(handPoseBehavior, yoTime);
      this.handPoseBehavior = handPoseBehavior;

      this.robotSide = robotSide;
      this.fullRobotModel = fullRobotModel;
      this.trajectoryTime = trajectoryTime;

      this.directionToMoveInWorld = directionToMoveInWorld;
      this.distanceToMove = distanceToMove;

      this.stopHandIfCollision = stopHandIfCollision;
   }

   @Override
   protected void setBehaviorInput()
   {
      FramePose desiredHandPose = getCurrentHandPose(robotSide);
      Vector3d desiredHandPoseOffsetFromCurrent = new Vector3d();

      if (directionToMoveInWorld != null)
      {
         desiredHandPoseOffsetFromCurrent.set(directionToMoveInWorld);
      }
      else
      {
         FrameVector directionToMove = new FrameVector();
         directionToMove.setIncludingFrame(new PoseReferenceFrame("currentHandPoseFrame", desiredHandPose), 1.0, 0.0, 0.0);
         directionToMove.changeFrame(ReferenceFrame.getWorldFrame());
         directionToMove.get(desiredHandPoseOffsetFromCurrent);
      }

      if (desiredHandPoseOffsetFromCurrent.length() > 0.0)
         desiredHandPoseOffsetFromCurrent.normalize();
      desiredHandPoseOffsetFromCurrent.scale(distanceToMove);
      desiredHandPose.translate(desiredHandPoseOffsetFromCurrent);

      RigidBodyTransform desiredHandTransformToWorld = new RigidBodyTransform();
      desiredHandPose.getPose(desiredHandTransformToWorld);

      handPoseBehavior.setInput(Frame.WORLD, desiredHandTransformToWorld, robotSide, trajectoryTime, stopHandIfCollision);
   }

   private FramePose getCurrentHandPose(RobotSide robotSide)
   {
      FramePose ret = new FramePose();
      ret.setToZero(fullRobotModel.getHandControlFrame(robotSide));
      ret.changeFrame(ReferenceFrame.getWorldFrame());
      return ret;
   }
}
