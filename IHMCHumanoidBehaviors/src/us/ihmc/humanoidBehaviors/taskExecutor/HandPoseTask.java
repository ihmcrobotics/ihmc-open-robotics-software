package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class HandPoseTask extends BehaviorTask
{
   private final HandPosePacket handPosePacket;
   private final HandPoseBehavior handPoseBehavior;

   private final boolean doHandPoseRelativeToHandPoseAtTransitionIntoAction;

   private final RobotSide robotSide;
   private final FullRobotModel fullRobotModel;
   private final double trajectoryTime;

   private final double distanceToMove;
   private final Vector3d directionToMoveInWorld;

   private final boolean stopHandIfCollision;

   public HandPoseTask(RobotSide robotSide, double[] desiredArmJointAngles, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, double trajectoryTime)
   {
      this(robotSide, desiredArmJointAngles, yoTime, handPoseBehavior, trajectoryTime, 0.0);
   }

   public HandPoseTask(RobotSide robotSide, double[] desiredArmJointAngles, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, double trajectoryTime,
         double sleepTime)
   {
      this(robotSide, new HandPosePacket(robotSide, trajectoryTime, desiredArmJointAngles), handPoseBehavior, yoTime, sleepTime, false);
   }

   public HandPoseTask(RobotSide robotSide, HandPosePacket handPosePacket, HandPoseBehavior handPoseBehavior, DoubleYoVariable yoTime)
   {
      this(robotSide, handPosePacket, handPoseBehavior, yoTime, 0.0, false);
   }

   public HandPoseTask(RobotSide robotSide, HandPosePacket handPosePacket, HandPoseBehavior handPoseBehavior, DoubleYoVariable yoTime, double sleepTime,
         boolean stopHandIfCollision)
   {
      super(handPoseBehavior, yoTime, sleepTime);
      this.handPoseBehavior = handPoseBehavior;
      this.handPosePacket = handPosePacket;

      this.robotSide = robotSide;
      this.fullRobotModel = null;
      this.trajectoryTime = handPosePacket.getTrajectoryTime();

      doHandPoseRelativeToHandPoseAtTransitionIntoAction = false;
      this.directionToMoveInWorld = null;
      this.distanceToMove = 0.0;

      this.stopHandIfCollision = stopHandIfCollision;
   }

   public HandPoseTask(RobotSide robotSide, double trajectoryTime, FramePose desiredPose, Frame desiredPoseFrame, HandPoseBehavior handPoseBehavior,
         DoubleYoVariable yoTime)
   {
      this(robotSide, trajectoryTime, desiredPose, desiredPoseFrame, handPoseBehavior, yoTime, false);
   }

   public HandPoseTask(RobotSide robotSide, double trajectoryTime, FramePose desiredHandPose, Frame desiredPoseFrame, HandPoseBehavior handPoseBehavior,
         DoubleYoVariable yoTime, boolean stopHandIfCollision)
   {
      super(handPoseBehavior, yoTime);
      this.handPoseBehavior = handPoseBehavior;

      RigidBodyTransform desiredPoseTransformToPoseFrame = new RigidBodyTransform();
      desiredHandPose.getPose(desiredPoseTransformToPoseFrame);
      this.handPosePacket = PacketControllerTools.createHandPosePacket(desiredPoseFrame, desiredPoseTransformToPoseFrame, robotSide, trajectoryTime);

      this.robotSide = robotSide;
      this.fullRobotModel = null;
      this.trajectoryTime = trajectoryTime;

      doHandPoseRelativeToHandPoseAtTransitionIntoAction = false;
      this.directionToMoveInWorld = null;
      this.distanceToMove = 0.0;

      this.stopHandIfCollision = stopHandIfCollision;
   }

   public HandPoseTask(RobotSide robotSide, double distanceToMove, FullRobotModel fullRobotModel, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior,
         double trajectoryTime)
   {
      this(robotSide, null, distanceToMove, fullRobotModel, yoTime, handPoseBehavior, trajectoryTime, false);
   }

   public HandPoseTask(RobotSide robotSide, Vector3d directionToMoveInWorld, double distanceToMove, FullRobotModel fullRobotModel, DoubleYoVariable yoTime,
         HandPoseBehavior handPoseBehavior, double trajectoryTime)
   {
      this(robotSide, directionToMoveInWorld, distanceToMove, fullRobotModel, yoTime, handPoseBehavior, trajectoryTime, false);
   }

   public HandPoseTask(RobotSide robotSide, Vector3d directionToMoveInWorld, double distanceToMove, FullRobotModel fullRobotModel, DoubleYoVariable yoTime,
         HandPoseBehavior handPoseBehavior, double trajectoryTime, boolean stopHandIfCollision)
   {
      super(handPoseBehavior, yoTime);
      this.handPoseBehavior = handPoseBehavior;
      this.handPosePacket = null;

      doHandPoseRelativeToHandPoseAtTransitionIntoAction = true;
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
      if (doHandPoseRelativeToHandPoseAtTransitionIntoAction)
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
      else
      {
         handPoseBehavior.setInput(handPosePacket, stopHandIfCollision);
      }
   }

   private FramePose getCurrentHandPose(RobotSide robotSide)
   {
      FramePose ret = new FramePose();
      ret.setToZero(fullRobotModel.getHandControlFrame(robotSide));
      ret.changeFrame(ReferenceFrame.getWorldFrame());
      return ret;
   }
}
