package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC behavior module
 */
public class ObjectDetectorResultPacket extends Packet<ObjectDetectorResultPacket>
      implements Settable<ObjectDetectorResultPacket>, EpsilonComparable<ObjectDetectorResultPacket>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public controller_msgs.msg.dds.HeatMapPacket heat_map_;
   public controller_msgs.msg.dds.BoundingBoxesPacket bounding_boxes_;

   public ObjectDetectorResultPacket()
   {
      header_ = new std_msgs.msg.dds.Header();
      heat_map_ = new controller_msgs.msg.dds.HeatMapPacket();
      bounding_boxes_ = new controller_msgs.msg.dds.BoundingBoxesPacket();
   }

   public ObjectDetectorResultPacket(ObjectDetectorResultPacket other)
   {
      this();
      set(other);
   }

   public void set(ObjectDetectorResultPacket other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      controller_msgs.msg.dds.HeatMapPacketPubSubType.staticCopy(other.heat_map_, heat_map_);
      controller_msgs.msg.dds.BoundingBoxesPacketPubSubType.staticCopy(other.bounding_boxes_, bounding_boxes_);
   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public controller_msgs.msg.dds.HeatMapPacket getHeatMap()
   {
      return heat_map_;
   }

   public controller_msgs.msg.dds.BoundingBoxesPacket getBoundingBoxes()
   {
      return bounding_boxes_;
   }

   @Override
   public boolean epsilonEquals(ObjectDetectorResultPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!this.heat_map_.epsilonEquals(other.heat_map_, epsilon))
         return false;
      if (!this.bounding_boxes_.epsilonEquals(other.bounding_boxes_, epsilon))
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
      if (!(other instanceof ObjectDetectorResultPacket))
         return false;

      ObjectDetectorResultPacket otherMyClass = (ObjectDetectorResultPacket) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (!this.heat_map_.equals(otherMyClass.heat_map_))
         return false;
      if (!this.bounding_boxes_.equals(otherMyClass.bounding_boxes_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ObjectDetectorResultPacket {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("heat_map=");
      builder.append(this.heat_map_);
      builder.append(", ");
      builder.append("bounding_boxes=");
      builder.append(this.bounding_boxes_);
      builder.append("}");
      return builder.toString();
   }
}
