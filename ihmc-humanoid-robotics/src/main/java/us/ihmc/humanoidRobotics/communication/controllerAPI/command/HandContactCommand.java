package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.HandContactMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandContactCommand implements Command<HandContactCommand, HandContactMessage>
{
   private long sequenceId;
   private boolean load;
   private double trajectoryDuration;
   private RobotSide robotSide;
   private final FramePoint3D bracingPoint = new FramePoint3D();
   private final FrameVector3D bracingNormal = new FrameVector3D();

   @Override
   public void clear()
   {
      robotSide = null;
      load = false;
      trajectoryDuration = Double.NaN;
      bracingPoint.setToNaN();
      bracingNormal.setToNaN();
   }

   @Override
   public void setFromMessage(HandContactMessage message)
   {
      sequenceId = message.getSequenceId();
      load = message.getLoad();
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
      load = other.load;
      trajectoryDuration = other.trajectoryDuration;
      bracingPoint.set(other.bracingPoint);
      bracingNormal.set(other.bracingNormal);
   }

   public void setLoad(boolean load)
   {
      this.load = load;
   }

   public boolean load()
   {
      return load;
   }

   public void setTrajectoryDuration(double trajectoryDuration)
   {
      this.trajectoryDuration = trajectoryDuration;
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide = robotSide;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double getTrajectoryDuration()
   {
      return trajectoryDuration;
   }

   public FramePoint3DBasics getBracingPoint()
   {
      return bracingPoint;
   }

   public FrameVector3DBasics getBracingNormal()
   {
      return bracingNormal;
   }
}
