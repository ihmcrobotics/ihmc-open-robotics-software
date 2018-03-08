package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class BoundingBoxesPacket implements Settable<BoundingBoxesPacket>, EpsilonComparable<BoundingBoxesPacket>
{
   private us.ihmc.idl.IDLSequence.Double bounding_boxes_x_coordinates_;
   private us.ihmc.idl.IDLSequence.Double bounding_boxes_y_coordinates_;
   private us.ihmc.idl.IDLSequence.Double bounding_boxes_widths_;
   private us.ihmc.idl.IDLSequence.Double bounding_boxes_heights_;
   private us.ihmc.idl.IDLSequence.StringBuilderHolder labels_;

   public BoundingBoxesPacket()
   {
      bounding_boxes_x_coordinates_ = new us.ihmc.idl.IDLSequence.Double(100, "type_6");

      bounding_boxes_y_coordinates_ = new us.ihmc.idl.IDLSequence.Double(100, "type_6");

      bounding_boxes_widths_ = new us.ihmc.idl.IDLSequence.Double(100, "type_6");

      bounding_boxes_heights_ = new us.ihmc.idl.IDLSequence.Double(100, "type_6");

      labels_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder(100, "type_d");
   }

   public BoundingBoxesPacket(BoundingBoxesPacket other)
   {
      set(other);
   }

   public void set(BoundingBoxesPacket other)
   {
      bounding_boxes_x_coordinates_.set(other.bounding_boxes_x_coordinates_);
      bounding_boxes_y_coordinates_.set(other.bounding_boxes_y_coordinates_);
      bounding_boxes_widths_.set(other.bounding_boxes_widths_);
      bounding_boxes_heights_.set(other.bounding_boxes_heights_);
      labels_.set(other.labels_);
   }

   public us.ihmc.idl.IDLSequence.Double getBoundingBoxesXCoordinates()
   {
      return bounding_boxes_x_coordinates_;
   }

   public us.ihmc.idl.IDLSequence.Double getBoundingBoxesYCoordinates()
   {
      return bounding_boxes_y_coordinates_;
   }

   public us.ihmc.idl.IDLSequence.Double getBoundingBoxesWidths()
   {
      return bounding_boxes_widths_;
   }

   public us.ihmc.idl.IDLSequence.Double getBoundingBoxesHeights()
   {
      return bounding_boxes_heights_;
   }

   public us.ihmc.idl.IDLSequence.StringBuilderHolder getLabels()
   {
      return labels_;
   }

   @Override
   public boolean epsilonEquals(BoundingBoxesPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.bounding_boxes_x_coordinates_, other.bounding_boxes_x_coordinates_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.bounding_boxes_y_coordinates_, other.bounding_boxes_y_coordinates_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.bounding_boxes_widths_, other.bounding_boxes_widths_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsDoubleSequence(this.bounding_boxes_heights_, other.bounding_boxes_heights_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.labels_, other.labels_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof BoundingBoxesPacket))
         return false;

      BoundingBoxesPacket otherMyClass = (BoundingBoxesPacket) other;

      if (!this.bounding_boxes_x_coordinates_.equals(otherMyClass.bounding_boxes_x_coordinates_))
         return false;

      if (!this.bounding_boxes_y_coordinates_.equals(otherMyClass.bounding_boxes_y_coordinates_))
         return false;

      if (!this.bounding_boxes_widths_.equals(otherMyClass.bounding_boxes_widths_))
         return false;

      if (!this.bounding_boxes_heights_.equals(otherMyClass.bounding_boxes_heights_))
         return false;

      if (!this.labels_.equals(otherMyClass.labels_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BoundingBoxesPacket {");
      builder.append("bounding_boxes_x_coordinates=");
      builder.append(this.bounding_boxes_x_coordinates_);

      builder.append(", ");
      builder.append("bounding_boxes_y_coordinates=");
      builder.append(this.bounding_boxes_y_coordinates_);

      builder.append(", ");
      builder.append("bounding_boxes_widths=");
      builder.append(this.bounding_boxes_widths_);

      builder.append(", ");
      builder.append("bounding_boxes_heights=");
      builder.append(this.bounding_boxes_heights_);

      builder.append(", ");
      builder.append("labels=");
      builder.append(this.labels_);

      builder.append("}");
      return builder.toString();
   }
}