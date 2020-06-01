package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class BoundingBoxesPacket extends Packet<BoundingBoxesPacket> implements Settable<BoundingBoxesPacket>, EpsilonComparable<BoundingBoxesPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public us.ihmc.idl.IDLSequence.Integer  bounding_boxes_x_coordinates_;

   public us.ihmc.idl.IDLSequence.Integer  bounding_boxes_y_coordinates_;

   public us.ihmc.idl.IDLSequence.Integer  bounding_boxes_widths_;

   public us.ihmc.idl.IDLSequence.Integer  bounding_boxes_heights_;

   public us.ihmc.idl.IDLSequence.StringBuilderHolder  labels_;

   public BoundingBoxesPacket()
   {


      bounding_boxes_x_coordinates_ = new us.ihmc.idl.IDLSequence.Integer (100, "type_2");


      bounding_boxes_y_coordinates_ = new us.ihmc.idl.IDLSequence.Integer (100, "type_2");


      bounding_boxes_widths_ = new us.ihmc.idl.IDLSequence.Integer (100, "type_2");


      bounding_boxes_heights_ = new us.ihmc.idl.IDLSequence.Integer (100, "type_2");


      labels_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (100, "type_d");

   }

   public BoundingBoxesPacket(BoundingBoxesPacket other)
   {
      this();
      set(other);
   }

   public void set(BoundingBoxesPacket other)
   {

      sequence_id_ = other.sequence_id_;


      bounding_boxes_x_coordinates_.set(other.bounding_boxes_x_coordinates_);

      bounding_boxes_y_coordinates_.set(other.bounding_boxes_y_coordinates_);

      bounding_boxes_widths_.set(other.bounding_boxes_widths_);

      bounding_boxes_heights_.set(other.bounding_boxes_heights_);

      labels_.set(other.labels_);
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



   public us.ihmc.idl.IDLSequence.Integer  getBoundingBoxesXCoordinates()
   {
      return bounding_boxes_x_coordinates_;
   }



   public us.ihmc.idl.IDLSequence.Integer  getBoundingBoxesYCoordinates()
   {
      return bounding_boxes_y_coordinates_;
   }



   public us.ihmc.idl.IDLSequence.Integer  getBoundingBoxesWidths()
   {
      return bounding_boxes_widths_;
   }



   public us.ihmc.idl.IDLSequence.Integer  getBoundingBoxesHeights()
   {
      return bounding_boxes_heights_;
   }



   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getLabels()
   {
      return labels_;
   }


   public static Supplier<BoundingBoxesPacketPubSubType> getPubSubType()
   {
      return BoundingBoxesPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BoundingBoxesPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BoundingBoxesPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.bounding_boxes_x_coordinates_, other.bounding_boxes_x_coordinates_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.bounding_boxes_y_coordinates_, other.bounding_boxes_y_coordinates_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.bounding_boxes_widths_, other.bounding_boxes_widths_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsIntegerSequence(this.bounding_boxes_heights_, other.bounding_boxes_heights_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.labels_, other.labels_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BoundingBoxesPacket)) return false;

      BoundingBoxesPacket otherMyClass = (BoundingBoxesPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.bounding_boxes_x_coordinates_.equals(otherMyClass.bounding_boxes_x_coordinates_)) return false;

      if (!this.bounding_boxes_y_coordinates_.equals(otherMyClass.bounding_boxes_y_coordinates_)) return false;

      if (!this.bounding_boxes_widths_.equals(otherMyClass.bounding_boxes_widths_)) return false;

      if (!this.bounding_boxes_heights_.equals(otherMyClass.bounding_boxes_heights_)) return false;

      if (!this.labels_.equals(otherMyClass.labels_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BoundingBoxesPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("bounding_boxes_x_coordinates=");
      builder.append(this.bounding_boxes_x_coordinates_);      builder.append(", ");

      builder.append("bounding_boxes_y_coordinates=");
      builder.append(this.bounding_boxes_y_coordinates_);      builder.append(", ");

      builder.append("bounding_boxes_widths=");
      builder.append(this.bounding_boxes_widths_);      builder.append(", ");

      builder.append("bounding_boxes_heights=");
      builder.append(this.bounding_boxes_heights_);      builder.append(", ");

      builder.append("labels=");
      builder.append(this.labels_);
      builder.append("}");
      return builder.toString();
   }
}
