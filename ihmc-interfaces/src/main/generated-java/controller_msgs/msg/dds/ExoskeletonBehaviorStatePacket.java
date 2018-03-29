package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is used to switch the control scheme between force and position control. WARNING:
 * When in position control, the IHMC balance algorithms will be disabled and it is up to the user
 * to ensure stability.
 */
public class ExoskeletonBehaviorStatePacket extends Packet<ExoskeletonBehaviorStatePacket>
      implements Settable<ExoskeletonBehaviorStatePacket>, EpsilonComparable<ExoskeletonBehaviorStatePacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public byte exoskeleton_behavior_state_;

   public ExoskeletonBehaviorStatePacket()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public ExoskeletonBehaviorStatePacket(ExoskeletonBehaviorStatePacket other)
   {
      this();
      set(other);
   }

   public void set(ExoskeletonBehaviorStatePacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      exoskeleton_behavior_state_ = other.exoskeleton_behavior_state_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setExoskeletonBehaviorState(byte exoskeleton_behavior_state)
   {
      exoskeleton_behavior_state_ = exoskeleton_behavior_state;
   }

   public byte getExoskeletonBehaviorState()
   {
      return exoskeleton_behavior_state_;
   }

   @Override
   public boolean epsilonEquals(ExoskeletonBehaviorStatePacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.exoskeleton_behavior_state_, other.exoskeleton_behavior_state_, epsilon))
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
      if (!(other instanceof ExoskeletonBehaviorStatePacket))
         return false;

      ExoskeletonBehaviorStatePacket otherMyClass = (ExoskeletonBehaviorStatePacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.exoskeleton_behavior_state_ != otherMyClass.exoskeleton_behavior_state_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ExoskeletonBehaviorStatePacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("exoskeleton_behavior_state=");
      builder.append(this.exoskeleton_behavior_state_);
      builder.append("}");
      return builder.toString();
   }
}
