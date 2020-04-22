package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC height quad tree module
       */
public class HeightQuadTreeMessage extends Packet<HeightQuadTreeMessage> implements Settable<HeightQuadTreeMessage>, EpsilonComparable<HeightQuadTreeMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public float default_height_;

   public float resolution_;

   public float size_x_;

   public float size_y_;

   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.HeightQuadTreeLeafMessage>  leaves_;

   public HeightQuadTreeMessage()
   {






      leaves_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.HeightQuadTreeLeafMessage> (5000, new controller_msgs.msg.dds.HeightQuadTreeLeafMessagePubSubType());

   }

   public HeightQuadTreeMessage(HeightQuadTreeMessage other)
   {
      this();
      set(other);
   }

   public void set(HeightQuadTreeMessage other)
   {

      sequence_id_ = other.sequence_id_;


      default_height_ = other.default_height_;


      resolution_ = other.resolution_;


      size_x_ = other.size_x_;


      size_y_ = other.size_y_;


      leaves_.set(other.leaves_);
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


   public void setDefaultHeight(float default_height)
   {
      default_height_ = default_height;
   }
   public float getDefaultHeight()
   {
      return default_height_;
   }


   public void setResolution(float resolution)
   {
      resolution_ = resolution;
   }
   public float getResolution()
   {
      return resolution_;
   }


   public void setSizeX(float size_x)
   {
      size_x_ = size_x;
   }
   public float getSizeX()
   {
      return size_x_;
   }


   public void setSizeY(float size_y)
   {
      size_y_ = size_y;
   }
   public float getSizeY()
   {
      return size_y_;
   }



   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.HeightQuadTreeLeafMessage>  getLeaves()
   {
      return leaves_;
   }


   public static Supplier<HeightQuadTreeMessagePubSubType> getPubSubType()
   {
      return HeightQuadTreeMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HeightQuadTreeMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HeightQuadTreeMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.default_height_, other.default_height_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.resolution_, other.resolution_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.size_x_, other.size_x_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.size_y_, other.size_y_, epsilon)) return false;


      if (this.leaves_.size() != other.leaves_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.leaves_.size(); i++)
         {  if (!this.leaves_.get(i).epsilonEquals(other.leaves_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HeightQuadTreeMessage)) return false;

      HeightQuadTreeMessage otherMyClass = (HeightQuadTreeMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.default_height_ != otherMyClass.default_height_) return false;


      if(this.resolution_ != otherMyClass.resolution_) return false;


      if(this.size_x_ != otherMyClass.size_x_) return false;


      if(this.size_y_ != otherMyClass.size_y_) return false;


      if (!this.leaves_.equals(otherMyClass.leaves_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HeightQuadTreeMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("default_height=");
      builder.append(this.default_height_);      builder.append(", ");

      builder.append("resolution=");
      builder.append(this.resolution_);      builder.append(", ");

      builder.append("size_x=");
      builder.append(this.size_x_);      builder.append(", ");

      builder.append("size_y=");
      builder.append(this.size_y_);      builder.append(", ");

      builder.append("leaves=");
      builder.append(this.leaves_);
      builder.append("}");
      return builder.toString();
   }
}
