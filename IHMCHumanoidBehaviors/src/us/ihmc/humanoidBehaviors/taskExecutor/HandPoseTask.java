package us.ihmc.humanoidBehaviors.taskExecutor;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.communication.packets.manipulation.HandPosePacket.Frame;
import us.ihmc.communication.util.PacketControllerTools;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class HandPoseTask extends BehaviorTask
{
   private static final boolean DEBUG = false;
   private final HandPosePacket handPosePacket;
   private final HandPoseBehavior handPoseBehavior;

   public HandPoseTask(RobotSide robotSide, double[] desiredArmJointAngles, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, double trajectoryTime)
   {
      this(robotSide, desiredArmJointAngles, yoTime, handPoseBehavior, trajectoryTime, 0.0);
   }

   public HandPoseTask(RobotSide robotSide, double[] desiredArmJointAngles, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, double trajectoryTime,
         double sleepTime)
   {
      super(handPoseBehavior, yoTime, sleepTime);
      this.handPoseBehavior = handPoseBehavior;
      handPosePacket = new HandPosePacket(robotSide, trajectoryTime, desiredArmJointAngles);
   }

   public HandPoseTask(RobotSide robotSide, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, Frame frame, RigidBodyTransform pose,
         double trajectoryTime)
   {
      super(handPoseBehavior, yoTime);
      this.handPoseBehavior = handPoseBehavior;
      handPosePacket = PacketControllerTools.createHandPosePacket(frame, pose, robotSide, trajectoryTime);
   }

   public HandPoseTask(RobotSide robotSide, DoubleYoVariable yoTime, HandPoseBehavior handPoseBehavior, Frame frame, FramePose pose, double trajectoryTime)
   {
      super(handPoseBehavior, yoTime);
      this.handPoseBehavior = handPoseBehavior;

      RigidBodyTransform rigidBodyPose = new RigidBodyTransform();
      pose.getPose(rigidBodyPose);

      handPosePacket = PacketControllerTools.createHandPosePacket(frame, rigidBodyPose, robotSide, trajectoryTime);
   }

   public HandPoseTask(HandPosePacket handPosePacket, HandPoseBehavior handPoseBehavior, DoubleYoVariable yoTime)
   {
      super(handPoseBehavior, yoTime);
      this.handPoseBehavior = handPoseBehavior;
      this.handPosePacket = handPosePacket;
   }

   @Override
   protected void setBehaviorInput()
   {
      handPoseBehavior.setInput(handPosePacket);
   }

}
