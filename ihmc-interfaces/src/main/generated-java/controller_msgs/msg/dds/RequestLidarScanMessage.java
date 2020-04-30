package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message used to request a new LIDAR scan, and also enable some filters.
       */
public class RequestLidarScanMessage extends Packet<RequestLidarScanMessage> implements Settable<RequestLidarScanMessage>, EpsilonComparable<RequestLidarScanMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public boolean remove_shadows_ = true;

   public boolean remove_self_collisions_ = true;

   public RequestLidarScanMessage()
   {




   }

   public RequestLidarScanMessage(RequestLidarScanMessage other)
   {
      this();
      set(other);
   }

   public void set(RequestLidarScanMessage other)
   {

      sequence_id_ = other.sequence_id_;


      remove_shadows_ = other.remove_shadows_;


      remove_self_collisions_ = other.remove_self_collisions_;

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


   public void setRemoveShadows(boolean remove_shadows)
   {
      remove_shadows_ = remove_shadows;
   }
   public boolean getRemoveShadows()
   {
      return remove_shadows_;
   }


   public void setRemoveSelfCollisions(boolean remove_self_collisions)
   {
      remove_self_collisions_ = remove_self_collisions;
   }
   public boolean getRemoveSelfCollisions()
   {
      return remove_self_collisions_;
   }


   public static Supplier<RequestLidarScanMessagePubSubType> getPubSubType()
   {
      return RequestLidarScanMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return RequestLidarScanMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(RequestLidarScanMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.remove_shadows_, other.remove_shadows_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.remove_self_collisions_, other.remove_self_collisions_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof RequestLidarScanMessage)) return false;

      RequestLidarScanMessage otherMyClass = (RequestLidarScanMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.remove_shadows_ != otherMyClass.remove_shadows_) return false;


      if(this.remove_self_collisions_ != otherMyClass.remove_self_collisions_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RequestLidarScanMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("remove_shadows=");
      builder.append(this.remove_shadows_);      builder.append(", ");

      builder.append("remove_self_collisions=");
      builder.append(this.remove_self_collisions_);
      builder.append("}");
      return builder.toString();
   }
}
