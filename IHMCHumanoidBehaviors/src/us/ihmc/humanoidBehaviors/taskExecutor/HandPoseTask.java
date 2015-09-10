package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandPoseTask extends BehaviorTask
{
   private final HandPosePacket handPosePacket;
   private final HandPoseBehavior handPoseBehavior;

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

      this.stopHandIfCollision = stopHandIfCollision;
   }

   public HandPoseTask(RobotSide robotSide, double trajectoryTime, FramePose desiredPose, Frame holdPoseInThisFrameIfRobotMoves,
         HandPoseBehavior handPoseBehavior, DoubleYoVariable yoTime)
   {
      this(robotSide, trajectoryTime, desiredPose, holdPoseInThisFrameIfRobotMoves, handPoseBehavior, yoTime, false);
   }

   public HandPoseTask(RobotSide robotSide, double trajectoryTime, FramePose desiredHandPose, Frame holdPoseInThisFrameIfRobotMoves,
         HandPoseBehavior handPoseBehavior, DoubleYoVariable yoTime, boolean stopHandIfCollision)
   {
      super(handPoseBehavior, yoTime);
      this.handPoseBehavior = handPoseBehavior;

      desiredHandPose.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      RigidBodyTransform desiredPoseTransformToWorld = new RigidBodyTransform();
      desiredHandPose.getPose(desiredPoseTransformToWorld);
      this.handPosePacket = PacketControllerTools.createHandPosePacket(holdPoseInThisFrameIfRobotMoves, desiredPoseTransformToWorld, robotSide, trajectoryTime);

      this.stopHandIfCollision = stopHandIfCollision;
   }

   @Override
   protected void setBehaviorInput()
   {
      handPoseBehavior.setInput(handPosePacket, stopHandIfCollision);
   }
}
