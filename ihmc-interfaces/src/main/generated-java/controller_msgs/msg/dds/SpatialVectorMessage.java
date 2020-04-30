package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Represents a 6D vector composed of a 3D anuglar vector and a 3D linear vector.
       */
public class SpatialVectorMessage extends Packet<SpatialVectorMessage> implements Settable<SpatialVectorMessage>, EpsilonComparable<SpatialVectorMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public us.ihmc.euclid.tuple3D.Vector3D angular_part_;

   public us.ihmc.euclid.tuple3D.Vector3D linear_part_;

   public SpatialVectorMessage()
   {


      angular_part_ = new us.ihmc.euclid.tuple3D.Vector3D();

      linear_part_ = new us.ihmc.euclid.tuple3D.Vector3D();

   }

   public SpatialVectorMessage(SpatialVectorMessage other)
   {
      this();
      set(other);
   }

   public void set(SpatialVectorMessage other)
   {

      sequence_id_ = other.sequence_id_;


      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.angular_part_, angular_part_);

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.linear_part_, linear_part_);
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



   public us.ihmc.euclid.tuple3D.Vector3D getAngularPart()
   {
      return angular_part_;
   }



   public us.ihmc.euclid.tuple3D.Vector3D getLinearPart()
   {
      return linear_part_;
   }


   public static Supplier<SpatialVectorMessagePubSubType> getPubSubType()
   {
      return SpatialVectorMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return SpatialVectorMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(SpatialVectorMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.angular_part_.epsilonEquals(other.angular_part_, epsilon)) return false;

      if (!this.linear_part_.epsilonEquals(other.linear_part_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof SpatialVectorMessage)) return false;

      SpatialVectorMessage otherMyClass = (SpatialVectorMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.angular_part_.equals(otherMyClass.angular_part_)) return false;

      if (!this.linear_part_.equals(otherMyClass.linear_part_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SpatialVectorMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("angular_part=");
      builder.append(this.angular_part_);      builder.append(", ");

      builder.append("linear_part=");
      builder.append(this.linear_part_);
      builder.append("}");
      return builder.toString();
   }
}
