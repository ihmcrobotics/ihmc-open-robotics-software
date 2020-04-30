package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC height quad tree module
       */
public class HeightQuadTreeLeafMessage extends Packet<HeightQuadTreeLeafMessage> implements Settable<HeightQuadTreeLeafMessage>, EpsilonComparable<HeightQuadTreeLeafMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public float center_x_;

   public float center_y_;

   public float height_;

   public HeightQuadTreeLeafMessage()
   {





   }

   public HeightQuadTreeLeafMessage(HeightQuadTreeLeafMessage other)
   {
      this();
      set(other);
   }

   public void set(HeightQuadTreeLeafMessage other)
   {

      sequence_id_ = other.sequence_id_;


      center_x_ = other.center_x_;


      center_y_ = other.center_y_;


      height_ = other.height_;

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


   public static Supplier<HeightQuadTreeLeafMessagePubSubType> getPubSubType()
   {
      return HeightQuadTreeLeafMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HeightQuadTreeLeafMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HeightQuadTreeLeafMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.center_x_, other.center_x_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.center_y_, other.center_y_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.height_, other.height_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HeightQuadTreeLeafMessage)) return false;

      HeightQuadTreeLeafMessage otherMyClass = (HeightQuadTreeLeafMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.center_x_ != otherMyClass.center_x_) return false;


      if(this.center_y_ != otherMyClass.center_y_) return false;


      if(this.height_ != otherMyClass.height_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HeightQuadTreeLeafMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("center_x=");
      builder.append(this.center_x_);      builder.append(", ");

      builder.append("center_y=");
      builder.append(this.center_y_);      builder.append(", ");

      builder.append("height=");
      builder.append(this.height_);
      builder.append("}");
      return builder.toString();
   }
}
