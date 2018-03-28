package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

public class ExoskeletonBehaviorStatePacket extends Packet<ExoskeletonBehaviorStatePacket>
      implements Settable<ExoskeletonBehaviorStatePacket>, EpsilonComparable<ExoskeletonBehaviorStatePacket>
{
   public byte exoskeleton_behavior_state_;

   public ExoskeletonBehaviorStatePacket()
   {
   }

   public ExoskeletonBehaviorStatePacket(ExoskeletonBehaviorStatePacket other)
   {
      set(other);
   }

   public void set(ExoskeletonBehaviorStatePacket other)
   {
      exoskeleton_behavior_state_ = other.exoskeleton_behavior_state_;

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

      if (this.exoskeleton_behavior_state_ != otherMyClass.exoskeleton_behavior_state_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ExoskeletonBehaviorStatePacket {");
      builder.append("exoskeleton_behavior_state=");
      builder.append(this.exoskeleton_behavior_state_);
      builder.append("}");
      return builder.toString();
   }
}
