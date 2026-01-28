package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.HandContactMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandContactCommand implements Command<HandContactCommand, HandContactMessage>
{
   private long sequenceId;
   private double trajectoryDuration;
   private RobotSide robotSide;
   private final FramePoint3D bracingPoint = new FramePoint3D();
   private final FrameVector3D bracingNormal = new FrameVector3D();

   @Override
   public void clear()
   {
      robotSide = null;
      trajectoryDuration = Double.NaN;
      bracingPoint.setToNaN();
      bracingNormal.setToNaN();
   }

   @Override
   public void setFromMessage(HandContactMessage message)
   {
      sequenceId = message.getSequenceId();
      robotSide = RobotSide.fromByte(message.getRobotSide());
      trajectoryDuration = message.getTrajectoryDuration();
      bracingPoint.set(ReferenceFrame.getWorldFrame(), message.getBracingPoint());
      bracingNormal.set(ReferenceFrame.getWorldFrame(), message.getBracingNormal());
   }

   @Override
   public Class<HandContactMessage> getMessageClass()
   {
      return HandContactMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && !bracingPoint.containsNaN() && !bracingNormal.containsNaN() && trajectoryDuration > 0.0;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(HandContactCommand other)
   {
      robotSide = other.robotSide;
      trajectoryDuration = other.trajectoryDuration;
      bracingPoint.set(other.bracingPoint);
      bracingNormal.set(other.bracingNormal);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public FramePoint3DReadOnly getBracingPoint()
   {
      return bracingPoint;
   }

   public FrameVector3DReadOnly getBracingNormal()
   {
      return bracingNormal;
   }
}
