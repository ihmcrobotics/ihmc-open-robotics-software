package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class QuadrupedTeleopDesiredHeight extends Packet<QuadrupedTeleopDesiredHeight> implements Settable<QuadrupedTeleopDesiredHeight>, EpsilonComparable<QuadrupedTeleopDesiredHeight>
{
   /**
    * Unique ID used to identify this message, should preferably be consecutively increasing.
    */
   public long sequence_id_;
   public double desired_height_;

   public QuadrupedTeleopDesiredHeight()
   {
   }

   public QuadrupedTeleopDesiredHeight(QuadrupedTeleopDesiredHeight other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedTeleopDesiredHeight other)
   {
      sequence_id_ = other.sequence_id_;

      desired_height_ = other.desired_height_;

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

   public void setDesiredHeight(double desired_height)
   {
      desired_height_ = desired_height;
   }
   public double getDesiredHeight()
   {
      return desired_height_;
   }


   public static Supplier<QuadrupedTeleopDesiredHeightPubSubType> getPubSubType()
   {
      return QuadrupedTeleopDesiredHeightPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedTeleopDesiredHeightPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedTeleopDesiredHeight other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_height_, other.desired_height_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedTeleopDesiredHeight)) return false;

      QuadrupedTeleopDesiredHeight otherMyClass = (QuadrupedTeleopDesiredHeight) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.desired_height_ != otherMyClass.desired_height_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedTeleopDesiredHeight {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("desired_height=");
      builder.append(this.desired_height_);      builder.append(", ");
      builder.append("}");
      return builder.toString();
   }
}