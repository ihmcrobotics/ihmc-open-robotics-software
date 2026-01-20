package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.ReactiveBracingMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

public class ReactiveBracingCommand implements Command<ReactiveBracingCommand, ReactiveBracingMessage>
{
   private long sequenceId;
   private RobotSide robotSide;
   private final FramePoint3D bracingPoint = new FramePoint3D();
   private final FrameVector3D bracingNormal = new FrameVector3D();

   @Override
   public void clear()
   {
      robotSide = null;
      bracingPoint.setToNaN();
      bracingNormal.setToNaN();
   }

   @Override
   public void setFromMessage(ReactiveBracingMessage message)
   {
      sequenceId = message.getSequenceId();
      robotSide = RobotSide.fromByte(message.getRobotSide());
      bracingPoint.set(ReferenceFrame.getWorldFrame(), message.getBracingPoint());
      bracingNormal.set(ReferenceFrame.getWorldFrame(), message.getBracingNormal());
   }

   @Override
   public Class<ReactiveBracingMessage> getMessageClass()
   {
      return ReactiveBracingMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return robotSide != null && !bracingPoint.containsNaN() && !bracingNormal.containsNaN();
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }

   @Override
   public void set(ReactiveBracingCommand other)
   {
      robotSide = other.robotSide;
      bracingPoint.set(other.bracingPoint);
      bracingNormal.set(other.bracingNormal);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
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
