package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.HandLoadBearingMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandLoadBearingCommand implements Command<HandLoadBearingCommand, HandLoadBearingMessage>
{
   private long sequenceId;
   private RobotSide robotSide;
   private boolean load = false;
   private double coefficientOfFriction = 0.0;
   private final Point3D contactPointInBodyFrame = new Point3D();
   private final Vector3D contactNormalInWorldFrame = new Vector3D();

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public boolean getLoad()
   {
      return load;
   }

   public double getCoefficientOfFriction()
   {
      return coefficientOfFriction;
   }

   public Point3D getContactPointInBodyFrame()
   {
      return contactPointInBodyFrame;
   }

   public Vector3D getContactNormalInWorldFrame()
   {
      return contactNormalInWorldFrame;
   }

   @Override
   public void set(HandLoadBearingCommand other)
   {
      sequenceId = other.sequenceId;
      robotSide = other.robotSide;
      load = other.getLoad();
      coefficientOfFriction = other.getCoefficientOfFriction();
      contactPointInBodyFrame.set(other.contactPointInBodyFrame);
      contactNormalInWorldFrame.set(other.contactNormalInWorldFrame);
   }

   @Override
   public void setFromMessage(HandLoadBearingMessage message)
   {
      sequenceId = message.getSequenceId();
      robotSide = RobotSide.fromByte(message.getRobotSide());
      load = message.getLoad();
      coefficientOfFriction = message.getCoefficientOfFriction();
      contactPointInBodyFrame.set(message.getContactPointInBodyFrame());
      contactNormalInWorldFrame.set(message.getContactNormalInWorld());
   }

   @Override
   public void clear()
   {
      sequenceId = 0;
      robotSide = null;
      load = false;
      coefficientOfFriction = 0.0;
      contactPointInBodyFrame.setToZero();
      contactNormalInWorldFrame.setToZero();
   }

   @Override
   public Class<HandLoadBearingMessage> getMessageClass()
   {
      return HandLoadBearingMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      if (robotSide == null)
         return false;
      if (load)
      {
         if (coefficientOfFriction <= 0.0)
            return false;
         if (contactPointInBodyFrame.containsNaN())
            return false;
         if (contactNormalInWorldFrame.containsNaN())
            return false;
      }
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
