package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Old message, needs to be tested and cleaned up.
       */
public class SnapFootstepPacket extends Packet<SnapFootstepPacket> implements Settable<SnapFootstepPacket>, EpsilonComparable<SnapFootstepPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage>  footstep_data_;

   public us.ihmc.idl.IDLSequence.Integer  footstep_order_;

   public us.ihmc.idl.IDLSequence.Byte  flag_;

   public SnapFootstepPacket()
   {


      footstep_data_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage> (100, new controller_msgs.msg.dds.FootstepDataMessagePubSubType());

      footstep_order_ = new us.ihmc.idl.IDLSequence.Integer (100, "type_2");


      flag_ = new us.ihmc.idl.IDLSequence.Byte (100, "type_9");


   }

   public SnapFootstepPacket(SnapFootstepPacket other)
   {
      this();
      set(other);
   }

   public void set(SnapFootstepPacket other)
   {

      sequence_id_ = other.sequence_id_;


      footstep_data_.set(other.footstep_data_);

      footstep_order_.set(other.footstep_order_);

      flag_.set(other.flag_);
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



   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage>  getFootstepData()
   {
      return footstep_data_;
   }



   public us.ihmc.idl.IDLSequence.Integer  getFootstepOrder()
   {
      return footstep_order_;
   }



   public us.ihmc.idl.IDLSequence.Byte  getFlag()
   {
      return flag_;
   }


   public static Supplier<SnapFootstepPacketPubSubType> getPubSubType()
   {
      return SnapFootstepPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SnapFootstepPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SnapFootstepPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (this.footstep_data_.size() != other.footstep_data_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.footstep_data_.size(); i++)
         {  if (!this.footstep_data_.get(i).epsilonEquals(other.footstep_data_.get(i), epsilon)) return false; }
      }


      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.footstep_order_, other.footstep_order_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.flag_, other.flag_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SnapFootstepPacket)) return false;

      SnapFootstepPacket otherMyClass = (SnapFootstepPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.footstep_data_.equals(otherMyClass.footstep_data_)) return false;

      if (!this.footstep_order_.equals(otherMyClass.footstep_order_)) return false;

      if (!this.flag_.equals(otherMyClass.flag_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SnapFootstepPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("footstep_data=");
      builder.append(this.footstep_data_);      builder.append(", ");

      builder.append("footstep_order=");
      builder.append(this.footstep_order_);      builder.append(", ");

      builder.append("flag=");
      builder.append(this.flag_);
      builder.append("}");
      return builder.toString();
   }
}
