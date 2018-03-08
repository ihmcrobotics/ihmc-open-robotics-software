package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC height quad tree module
 */
public class HeightQuadTreeMessage implements Settable<HeightQuadTreeMessage>, EpsilonComparable<HeightQuadTreeMessage>
{
   private float default_height_;
   private float resolution_;
   private float x_size_;
   private float y_size_;
   private us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.HeightQuadTreeLeafMessage> leaves_;

   public HeightQuadTreeMessage()
   {

      leaves_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.HeightQuadTreeLeafMessage>(5000,
                                                                                                      controller_msgs.msg.dds.HeightQuadTreeLeafMessage.class,
                                                                                                      new controller_msgs.msg.dds.HeightQuadTreeLeafMessagePubSubType());
   }

   public HeightQuadTreeMessage(HeightQuadTreeMessage other)
   {
      set(other);
   }

   public void set(HeightQuadTreeMessage other)
   {
      default_height_ = other.default_height_;

      resolution_ = other.resolution_;

      x_size_ = other.x_size_;

      y_size_ = other.y_size_;

      leaves_.set(other.leaves_);
   }

   public float getDefaultHeight()
   {
      return default_height_;
   }

   public void setDefaultHeight(float default_height)
   {
      default_height_ = default_height;
   }

   public float getResolution()
   {
      return resolution_;
   }

   public void setResolution(float resolution)
   {
      resolution_ = resolution;
   }

   public float getXSize()
   {
      return x_size_;
   }

   public void setXSize(float x_size)
   {
      x_size_ = x_size;
   }

   public float getYSize()
   {
      return y_size_;
   }

   public void setYSize(float y_size)
   {
      y_size_ = y_size;
   }

   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.HeightQuadTreeLeafMessage> getLeaves()
   {
      return leaves_;
   }

   @Override
   public boolean epsilonEquals(HeightQuadTreeMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.default_height_, other.default_height_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.resolution_, other.resolution_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.x_size_, other.x_size_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.y_size_, other.y_size_, epsilon))
         return false;

      if (this.leaves_.size() == other.leaves_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.leaves_.size(); i++)
         {
            if (!this.leaves_.get(i).epsilonEquals(other.leaves_.get(i), epsilon))
               return false;
         }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof HeightQuadTreeMessage))
         return false;

      HeightQuadTreeMessage otherMyClass = (HeightQuadTreeMessage) other;

      if (this.default_height_ != otherMyClass.default_height_)
         return false;

      if (this.resolution_ != otherMyClass.resolution_)
         return false;

      if (this.x_size_ != otherMyClass.x_size_)
         return false;

      if (this.y_size_ != otherMyClass.y_size_)
         return false;

      if (!this.leaves_.equals(otherMyClass.leaves_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HeightQuadTreeMessage {");
      builder.append("default_height=");
      builder.append(this.default_height_);

      builder.append(", ");
      builder.append("resolution=");
      builder.append(this.resolution_);

      builder.append(", ");
      builder.append("x_size=");
      builder.append(this.x_size_);

      builder.append(", ");
      builder.append("y_size=");
      builder.append(this.y_size_);

      builder.append(", ");
      builder.append("leaves=");
      builder.append(this.leaves_);

      builder.append("}");
      return builder.toString();
   }
}