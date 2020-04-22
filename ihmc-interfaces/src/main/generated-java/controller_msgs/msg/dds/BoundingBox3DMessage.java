package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BoundingBox3DMessage extends Packet<BoundingBox3DMessage> implements Settable<BoundingBox3DMessage>, EpsilonComparable<BoundingBox3DMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public us.ihmc.euclid.tuple3D.Point3D min_point_;

   public us.ihmc.euclid.tuple3D.Point3D max_point_;

   public BoundingBox3DMessage()
   {


      min_point_ = new us.ihmc.euclid.tuple3D.Point3D();

      max_point_ = new us.ihmc.euclid.tuple3D.Point3D();

   }

   public BoundingBox3DMessage(BoundingBox3DMessage other)
   {
      this();
      set(other);
   }

   public void set(BoundingBox3DMessage other)
   {

      sequence_id_ = other.sequence_id_;


      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.min_point_, min_point_);

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.max_point_, max_point_);
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



   public us.ihmc.euclid.tuple3D.Point3D getMinPoint()
   {
      return min_point_;
   }



   public us.ihmc.euclid.tuple3D.Point3D getMaxPoint()
   {
      return max_point_;
   }


   public static Supplier<BoundingBox3DMessagePubSubType> getPubSubType()
   {
      return BoundingBox3DMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BoundingBox3DMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BoundingBox3DMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.min_point_.epsilonEquals(other.min_point_, epsilon)) return false;

      if (!this.max_point_.epsilonEquals(other.max_point_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BoundingBox3DMessage)) return false;

      BoundingBox3DMessage otherMyClass = (BoundingBox3DMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.min_point_.equals(otherMyClass.min_point_)) return false;

      if (!this.max_point_.equals(otherMyClass.max_point_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BoundingBox3DMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("min_point=");
      builder.append(this.min_point_);      builder.append(", ");

      builder.append("max_point=");
      builder.append(this.max_point_);
      builder.append("}");
      return builder.toString();
   }
}
