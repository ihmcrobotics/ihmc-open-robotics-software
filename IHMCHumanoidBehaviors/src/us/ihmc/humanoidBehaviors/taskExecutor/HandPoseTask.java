package us.ihmc.humanoidBehaviors.taskExecutor;

import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class HandPoseTask extends BehaviorTask
{
   private static final boolean DEBUG = false;

   private final HandPosePacket handPosePacket;
   private final HandPoseBehavior handPoseBehavior;

   private final boolean doHandPoseRelativeToHandPoseAtTransitionIntoAction;
   private final RobotSide robotSide;
   private final FullRobotModel fullRobotModel;
   private final double trajectoryTime;
   private final Vector3d desiredHandPoseOffsetFromCurrent;

   public HandPoseTask(RobotSide robotSide, double[] desiredArmJointAngles, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, double trajectoryTime)
   {
      this(robotSide, desiredArmJointAngles, yoTime, handPoseBehavior, trajectoryTime, 0.0);
   }

   public HandPoseTask(RobotSide robotSide, double[] desiredArmJointAngles, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, double trajectoryTime,
         double sleepTime)
   {
      this(robotSide, new HandPosePacket(robotSide, trajectoryTime, desiredArmJointAngles), handPoseBehavior, yoTime, sleepTime);
   }

   public HandPoseTask(RobotSide robotSide, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, Frame frame, RigidBodyTransform pose,
         double trajectoryTime)
   {
      this(robotSide, PacketControllerTools.createHandPosePacket(frame, pose, robotSide, trajectoryTime), handPoseBehavior, yoTime);
   }

   public HandPoseTask(RobotSide robotSide, HandPosePacket handPosePacket, HandPoseBehavior handPoseBehavior, DoubleYoVariable yoTime)
   {
      this(robotSide, handPosePacket, handPoseBehavior, yoTime, 0.0);
   }

   public HandPoseTask(RobotSide robotSide, HandPosePacket handPosePacket, HandPoseBehavior handPoseBehavior, DoubleYoVariable yoTime, double sleepTime)
   {
      super(handPoseBehavior, yoTime);
      this.handPoseBehavior = handPoseBehavior;
      this.handPosePacket = handPosePacket;

      doHandPoseRelativeToHandPoseAtTransitionIntoAction = false;
      this.robotSide = robotSide;
      this.fullRobotModel = null;
      this.trajectoryTime = handPosePacket.getTrajectoryTime();
      this.desiredHandPoseOffsetFromCurrent = null;
   }

   public HandPoseTask(RobotSide robotSide, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, Frame frame, FramePose pose, double trajectoryTime)
   {
      super(handPoseBehavior, yoTime);
      this.handPoseBehavior = handPoseBehavior;

      RigidBodyTransform poseTransformToWorld = new RigidBodyTransform();
      pose.getPose(poseTransformToWorld);
      this.handPosePacket = PacketControllerTools.createHandPosePacket(frame, poseTransformToWorld, robotSide, trajectoryTime);

      doHandPoseRelativeToHandPoseAtTransitionIntoAction = false;
      this.robotSide = robotSide;
      this.fullRobotModel = null;
      this.trajectoryTime = trajectoryTime;
      this.desiredHandPoseOffsetFromCurrent = null;
   }

   public HandPoseTask(RobotSide robotSide, Vector3d directionToMoveInWorld, double distanceToMove, FullRobotModel fullRobotModel, DoubleYoVariable yoTime,
         HandPoseBehavior handPoseBehavior, double trajectoryTime)
   {
      super(handPoseBehavior, yoTime);
      this.handPoseBehavior = handPoseBehavior;
      this.handPosePacket = null;

      doHandPoseRelativeToHandPoseAtTransitionIntoAction = true;
      this.robotSide = robotSide;
      this.fullRobotModel = fullRobotModel;
      this.trajectoryTime = trajectoryTime;
      this.desiredHandPoseOffsetFromCurrent = new Vector3d(directionToMoveInWorld);
      if (desiredHandPoseOffsetFromCurrent.length() > 0.0)
         desiredHandPoseOffsetFromCurrent.normalize();
      desiredHandPoseOffsetFromCurrent.scale(distanceToMove);
   }

   @Override
   protected void setBehaviorInput()
   {
      if (doHandPoseRelativeToHandPoseAtTransitionIntoAction)
      {
         FramePose desiredHandPose = getCurrentHandPose(robotSide);
         desiredHandPose.translate(desiredHandPoseOffsetFromCurrent);

         RigidBodyTransform desiredHandTransformToWorld = new RigidBodyTransform();
         desiredHandPose.getPose(desiredHandTransformToWorld);

         handPoseBehavior.setInput(Frame.WORLD, desiredHandTransformToWorld, robotSide, trajectoryTime);
      }
      else
      {
         handPoseBehavior.setInput(handPosePacket);
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
