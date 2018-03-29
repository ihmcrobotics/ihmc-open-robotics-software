package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;

/**
 * This message is part of the IHMC height quad tree module
 */
public class HeightQuadTreeLeafMessage extends Packet<HeightQuadTreeLeafMessage>
      implements Settable<HeightQuadTreeLeafMessage>, EpsilonComparable<HeightQuadTreeLeafMessage>
{
   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header header_;
   public float center_x_;
   public float center_y_;
   public float height_;

   public HeightQuadTreeLeafMessage()
   {
      header_ = new std_msgs.msg.dds.Header();
   }

   public HeightQuadTreeLeafMessage(HeightQuadTreeLeafMessage other)
   {
      this();
      set(other);
   }

   public void set(HeightQuadTreeLeafMessage other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      center_x_ = other.center_x_;

      center_y_ = other.center_y_;

      height_ = other.height_;

   }

   /**
    * As of March 2018, the header for this message is only use for its sequence ID.
    */
   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public void setCenterX(float center_x)
   {
      center_x_ = center_x;
   }

   public float getCenterX()
   {
      return center_x_;
   }

   public void setCenterY(float center_y)
   {
      center_y_ = center_y;
   }

   public float getCenterY()
   {
      return center_y_;
   }

   public void setHeight(float height)
   {
      height_ = height;
   }

   public float getHeight()
   {
      return height_;
   }

   @Override
   public boolean epsilonEquals(HeightQuadTreeLeafMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.header_.epsilonEquals(other.header_, epsilon))
         return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.center_x_, other.center_x_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.center_y_, other.center_y_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_, other.height_, epsilon))
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
      if (!(other instanceof HeightQuadTreeLeafMessage))
         return false;

      HeightQuadTreeLeafMessage otherMyClass = (HeightQuadTreeLeafMessage) other;

      if (!this.header_.equals(otherMyClass.header_))
         return false;
      if (this.center_x_ != otherMyClass.center_x_)
         return false;

      if (this.center_y_ != otherMyClass.center_y_)
         return false;

      if (this.height_ != otherMyClass.height_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HeightQuadTreeLeafMessage {");
      builder.append("header=");
      builder.append(this.header_);
      builder.append(", ");
      builder.append("center_x=");
      builder.append(this.center_x_);
      builder.append(", ");
      builder.append("center_y=");
      builder.append(this.center_y_);
      builder.append(", ");
      builder.append("height=");
      builder.append(this.height_);
      builder.append("}");
      return builder.toString();
   }
}
