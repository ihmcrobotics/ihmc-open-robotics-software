package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC humanoid behavior module.
 */
public class BehaviorControlModeResponsePacket extends Packet<BehaviorControlModeResponsePacket>
      implements Settable<BehaviorControlModeResponsePacket>, EpsilonComparable<BehaviorControlModeResponsePacket>
{
   public static final byte STOP = (byte) 0;
   public static final byte PAUSE = (byte) 1;
   public static final byte RESUME = (byte) 2;
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public byte behavior_control_mode_enum_request_ = (byte) 255;

   public BehaviorControlModeResponsePacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public BehaviorControlModeResponsePacket(BehaviorControlModeResponsePacket other)
   {
      this();
      set(other);
   }

   public void set(BehaviorControlModeResponsePacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      behavior_control_mode_enum_request_ = other.behavior_control_mode_enum_request_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setBehaviorControlModeEnumRequest(byte behavior_control_mode_enum_request)
   {
      behavior_control_mode_enum_request_ = behavior_control_mode_enum_request;
   }

   public byte getBehaviorControlModeEnumRequest()
   {
      return behavior_control_mode_enum_request_;
   }

   @Override
   public boolean epsilonEquals(BehaviorControlModeResponsePacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.behavior_control_mode_enum_request_, other.behavior_control_mode_enum_request_, epsilon))
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
      if (!(other instanceof BehaviorControlModeResponsePacket))
         return false;

      BehaviorControlModeResponsePacket otherMyClass = (BehaviorControlModeResponsePacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.behavior_control_mode_enum_request_ != otherMyClass.behavior_control_mode_enum_request_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorControlModeResponsePacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("behavior_control_mode_enum_request=");
      builder.append(this.behavior_control_mode_enum_request_);
      builder.append("}");
      return builder.toString();
   }
}
