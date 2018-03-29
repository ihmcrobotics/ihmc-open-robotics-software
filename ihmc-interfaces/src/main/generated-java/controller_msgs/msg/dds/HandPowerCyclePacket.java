package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * Atlas specific message used to power cycle the hands.
 */
public class HandPowerCyclePacket extends Packet<HandPowerCyclePacket> implements Settable<HandPowerCyclePacket>, EpsilonComparable<HandPowerCyclePacket>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public byte robot_side_ = (byte) 255;

   public HandPowerCyclePacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public HandPowerCyclePacket(HandPowerCyclePacket other)
   {
      this();
      set(other);
   }

   public void set(HandPowerCyclePacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      robot_side_ = other.robot_side_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }

   public byte getRobotSide()
   {
      return robot_side_;
   }

   @Override
   public boolean epsilonEquals(HandPowerCyclePacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
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

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.robot_side_ != otherMyClass.robot_side_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandPowerCyclePacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);
      builder.append("}");
      return builder.toString();
   }
}
