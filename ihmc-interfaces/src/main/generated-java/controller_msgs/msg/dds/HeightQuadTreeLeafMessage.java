package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC height quad tree module
 */
public class HeightQuadTreeLeafMessage implements Settable<HeightQuadTreeLeafMessage>, EpsilonComparable<HeightQuadTreeLeafMessage>
{
   private float x_center_;
   private float y_center_;
   private float height_;

   public HeightQuadTreeLeafMessage()
   {

   }

   public HeightQuadTreeLeafMessage(HeightQuadTreeLeafMessage other)
   {
      set(other);
   }

   public void set(HeightQuadTreeLeafMessage other)
   {
      x_center_ = other.x_center_;

      y_center_ = other.y_center_;

      height_ = other.height_;
   }

   public float getXCenter()
   {
      return x_center_;
   }

   public void setXCenter(float x_center)
   {
      x_center_ = x_center;
   }

   public float getYCenter()
   {
      return y_center_;
   }

   public void setYCenter(float y_center)
   {
      y_center_ = y_center;
   }

   public float getHeight()
   {
      return height_;
   }

   public void setHeight(float height)
   {
      height_ = height;
   }

   @Override
   public boolean epsilonEquals(HeightQuadTreeLeafMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_center_, other.x_center_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_center_, other.y_center_, epsilon))
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

      if (this.x_center_ != otherMyClass.x_center_)
         return false;

      if (this.y_center_ != otherMyClass.y_center_)
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
      builder.append("x_center=");
      builder.append(this.x_center_);

      builder.append(", ");
      builder.append("y_center=");
      builder.append(this.y_center_);

      builder.append(", ");
      builder.append("height=");
      builder.append(this.height_);

      builder.append("}");
      return builder.toString();
   }
}