package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * General purpose message that contains vertices to represent a 2D polygon (z ignored).
       */
public class Polygon2DMessage extends Packet<Polygon2DMessage> implements Settable<Polygon2DMessage>, EpsilonComparable<Polygon2DMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  vertices_;

   public Polygon2DMessage()
   {


      vertices_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (50, new geometry_msgs.msg.dds.PointPubSubType());

   }

   public Polygon2DMessage(Polygon2DMessage other)
   {
      this();
      set(other);
   }

   public void set(Polygon2DMessage other)
   {

      sequence_id_ = other.sequence_id_;


      vertices_.set(other.vertices_);
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



   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getVertices()
   {
      return vertices_;
   }


   public static Supplier<Polygon2DMessagePubSubType> getPubSubType()
   {
      return Polygon2DMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return Polygon2DMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(Polygon2DMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (this.vertices_.size() != other.vertices_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.vertices_.size(); i++)
         {  if (!this.vertices_.get(i).epsilonEquals(other.vertices_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof Polygon2DMessage)) return false;

      Polygon2DMessage otherMyClass = (Polygon2DMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.vertices_.equals(otherMyClass.vertices_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("Polygon2DMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("vertices=");
      builder.append(this.vertices_);
      builder.append("}");
      return builder.toString();
   }
}
