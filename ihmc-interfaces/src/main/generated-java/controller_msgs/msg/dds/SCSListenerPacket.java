package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of IHMC Simulation Construction Set.
       * Notifies the user when the simulation has stopped.
       */
public class SCSListenerPacket extends Packet<SCSListenerPacket> implements Settable<SCSListenerPacket>, EpsilonComparable<SCSListenerPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public boolean is_stopped_ = true;

   public SCSListenerPacket()
   {



   }

   public SCSListenerPacket(SCSListenerPacket other)
   {
      this();
      set(other);
   }

   public void set(SCSListenerPacket other)
   {

      sequence_id_ = other.sequence_id_;


      is_stopped_ = other.is_stopped_;

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


   public void setIsStopped(boolean is_stopped)
   {
      is_stopped_ = is_stopped;
   }
   public boolean getIsStopped()
   {
      return is_stopped_;
   }


   public static Supplier<SCSListenerPacketPubSubType> getPubSubType()
   {
      return SCSListenerPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SCSListenerPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SCSListenerPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_stopped_, other.is_stopped_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SCSListenerPacket)) return false;

      SCSListenerPacket otherMyClass = (SCSListenerPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.is_stopped_ != otherMyClass.is_stopped_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SCSListenerPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("is_stopped=");
      builder.append(this.is_stopped_);
      builder.append("}");
      return builder.toString();
   }
}
