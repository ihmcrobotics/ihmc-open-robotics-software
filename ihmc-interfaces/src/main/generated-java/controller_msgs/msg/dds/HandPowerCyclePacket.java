package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Atlas specific message used to power cycle the hands.
 */
public class HandPowerCyclePacket implements Settable<HandPowerCyclePacket>, EpsilonComparable<HandPowerCyclePacket>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   private byte robot_side_ = (byte) 255;

   public HandPowerCyclePacket()
   {
   }

   public HandPowerCyclePacket(HandPowerCyclePacket other)
   {
      set(other);
   }

   public void set(HandPowerCyclePacket other)
   {
      robot_side_ = other.robot_side_;
   }

   public byte getRobotSide()
   {
      return robot_side_;
   }

   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   @Override
   public boolean epsilonEquals(HandPowerCyclePacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof HandPowerCyclePacket))
         return false;

      HandPowerCyclePacket otherMyClass = (HandPowerCyclePacket) other;

      if (this.robot_side_ != otherMyClass.robot_side_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandPowerCyclePacket {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);

      builder.append("}");
      return builder.toString();
   }
}