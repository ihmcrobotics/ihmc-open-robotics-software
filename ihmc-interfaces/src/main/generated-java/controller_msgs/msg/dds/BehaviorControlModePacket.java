package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC humanoid behavior module.
 */
public class BehaviorControlModePacket extends Packet<BehaviorControlModePacket>
      implements Settable<BehaviorControlModePacket>, EpsilonComparable<BehaviorControlModePacket>
{
   public static final byte STOP = (byte) 0;
   public static final byte PAUSE = (byte) 1;
   public static final byte RESUME = (byte) 2;
   public byte behavior_control_mode_enum_request_ = (byte) 255;

   public BehaviorControlModePacket()
   {
   }

   public BehaviorControlModePacket(BehaviorControlModePacket other)
   {
      set(other);
   }

   public void set(BehaviorControlModePacket other)
   {
      behavior_control_mode_enum_request_ = other.behavior_control_mode_enum_request_;
   }

   public byte getBehaviorControlModeEnumRequest()
   {
      return behavior_control_mode_enum_request_;
   }

   public void setBehaviorControlModeEnumRequest(byte behavior_control_mode_enum_request)
   {
      behavior_control_mode_enum_request_ = behavior_control_mode_enum_request;
   }

   @Override
   public boolean epsilonEquals(BehaviorControlModePacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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
      if (!(other instanceof BehaviorControlModePacket))
         return false;

      BehaviorControlModePacket otherMyClass = (BehaviorControlModePacket) other;

      if (this.behavior_control_mode_enum_request_ != otherMyClass.behavior_control_mode_enum_request_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BehaviorControlModePacket {");
      builder.append("behavior_control_mode_enum_request=");
      builder.append(this.behavior_control_mode_enum_request_);

      builder.append("}");
      return builder.toString();
   }
}
