package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class DetectedFacesPacket extends Packet<DetectedFacesPacket> implements Settable<DetectedFacesPacket>, EpsilonComparable<DetectedFacesPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public us.ihmc.idl.IDLSequence.StringBuilderHolder  ids_;

   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  positions_;

   public DetectedFacesPacket()
   {


      ids_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");

      positions_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D> (100, new geometry_msgs.msg.dds.PointPubSubType());

   }

   public DetectedFacesPacket(DetectedFacesPacket other)
   {
      this();
      set(other);
   }

   public void set(DetectedFacesPacket other)
   {

      sequence_id_ = other.sequence_id_;


      ids_.set(other.ids_);

      positions_.set(other.positions_);
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



   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getIds()
   {
      return ids_;
   }



   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Point3D>  getPositions()
   {
      return positions_;
   }


   public static Supplier<DetectedFacesPacketPubSubType> getPubSubType()
   {
      return DetectedFacesPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return DetectedFacesPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(DetectedFacesPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.ids_, other.ids_, epsilon)) return false;


      if (this.positions_.size() != other.positions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.positions_.size(); i++)
         {  if (!this.positions_.get(i).epsilonEquals(other.positions_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof DetectedFacesPacket)) return false;

      DetectedFacesPacket otherMyClass = (DetectedFacesPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.ids_.equals(otherMyClass.ids_)) return false;

      if (!this.positions_.equals(otherMyClass.positions_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("DetectedFacesPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("ids=");
      builder.append(this.ids_);      builder.append(", ");

      builder.append("positions=");
      builder.append(this.positions_);
      builder.append("}");
      return builder.toString();
   }
}
