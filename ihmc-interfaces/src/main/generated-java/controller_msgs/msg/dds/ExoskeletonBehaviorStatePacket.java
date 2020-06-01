package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is used to switch the control scheme between force and position control.
       * WARNING: When in position control, the IHMC balance algorithms will be disabled and it is up to the user to ensure stability.
       */
public class ExoskeletonBehaviorStatePacket extends Packet<ExoskeletonBehaviorStatePacket> implements Settable<ExoskeletonBehaviorStatePacket>, EpsilonComparable<ExoskeletonBehaviorStatePacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public byte exoskeleton_behavior_state_;

   public ExoskeletonBehaviorStatePacket()
   {



   }

   public ExoskeletonBehaviorStatePacket(ExoskeletonBehaviorStatePacket other)
   {
      this();
      set(other);
   }

   public void set(ExoskeletonBehaviorStatePacket other)
   {

      sequence_id_ = other.sequence_id_;


      exoskeleton_behavior_state_ = other.exoskeleton_behavior_state_;

   }


   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }


   public void setExoskeletonBehaviorState(byte exoskeleton_behavior_state)
   {
      exoskeleton_behavior_state_ = exoskeleton_behavior_state;
   }
   public byte getExoskeletonBehaviorState()
   {
      return exoskeleton_behavior_state_;
   }


   public static Supplier<ExoskeletonBehaviorStatePacketPubSubType> getPubSubType()
   {
      return ExoskeletonBehaviorStatePacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ExoskeletonBehaviorStatePacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ExoskeletonBehaviorStatePacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.exoskeleton_behavior_state_, other.exoskeleton_behavior_state_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ExoskeletonBehaviorStatePacket)) return false;

      ExoskeletonBehaviorStatePacket otherMyClass = (ExoskeletonBehaviorStatePacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.exoskeleton_behavior_state_ != otherMyClass.exoskeleton_behavior_state_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ExoskeletonBehaviorStatePacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("exoskeleton_behavior_state=");
      builder.append(this.exoskeleton_behavior_state_);
      builder.append("}");
      return builder.toString();
   }
}
