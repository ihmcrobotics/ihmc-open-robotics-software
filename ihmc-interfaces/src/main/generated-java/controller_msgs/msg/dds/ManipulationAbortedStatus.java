package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message is sent by the controller to notify the user that the current manipulation task has been aborted.
       */
public class ManipulationAbortedStatus extends Packet<ManipulationAbortedStatus> implements Settable<ManipulationAbortedStatus>, EpsilonComparable<ManipulationAbortedStatus>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public ManipulationAbortedStatus()
   {


   }

   public ManipulationAbortedStatus(ManipulationAbortedStatus other)
   {
      this();
      set(other);
   }

   public void set(ManipulationAbortedStatus other)
   {

      sequence_id_ = other.sequence_id_;

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


   public static Supplier<ManipulationAbortedStatusPubSubType> getPubSubType()
   {
      return ManipulationAbortedStatusPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ManipulationAbortedStatusPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ManipulationAbortedStatus other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ManipulationAbortedStatus)) return false;

      ManipulationAbortedStatus otherMyClass = (ManipulationAbortedStatus) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ManipulationAbortedStatus {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);
      builder.append("}");
      return builder.toString();
   }
}
