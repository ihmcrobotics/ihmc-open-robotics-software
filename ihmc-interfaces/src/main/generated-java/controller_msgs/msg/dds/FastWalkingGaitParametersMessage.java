package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is sent to the Fast Walking Controller to control the gait profiles.
       */
public class FastWalkingGaitParametersMessage extends Packet<FastWalkingGaitParametersMessage> implements Settable<FastWalkingGaitParametersMessage>, EpsilonComparable<FastWalkingGaitParametersMessage>
{
   public double swing_height_;
   public double swing_duration_;
   public double double_support_fraction_;

   public FastWalkingGaitParametersMessage()
   {
   }

   public FastWalkingGaitParametersMessage(FastWalkingGaitParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(FastWalkingGaitParametersMessage other)
   {
      swing_height_ = other.swing_height_;

      swing_duration_ = other.swing_duration_;

      double_support_fraction_ = other.double_support_fraction_;

   }

   public void setSwingHeight(double swing_height)
   {
      swing_height_ = swing_height;
   }
   public double getSwingHeight()
   {
      return swing_height_;
   }

   public void setSwingDuration(double swing_duration)
   {
      swing_duration_ = swing_duration;
   }
   public double getSwingDuration()
   {
      return swing_duration_;
   }

   public void setDoubleSupportFraction(double double_support_fraction)
   {
      double_support_fraction_ = double_support_fraction;
   }
   public double getDoubleSupportFraction()
   {
      return double_support_fraction_;
   }


   public static Supplier<FastWalkingGaitParametersMessagePubSubType> getPubSubType()
   {
      return FastWalkingGaitParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FastWalkingGaitParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FastWalkingGaitParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_height_, other.swing_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_, other.swing_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.double_support_fraction_, other.double_support_fraction_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FastWalkingGaitParametersMessage)) return false;

      FastWalkingGaitParametersMessage otherMyClass = (FastWalkingGaitParametersMessage) other;

      if(this.swing_height_ != otherMyClass.swing_height_) return false;

      if(this.swing_duration_ != otherMyClass.swing_duration_) return false;

      if(this.double_support_fraction_ != otherMyClass.double_support_fraction_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FastWalkingGaitParametersMessage {");
      builder.append("swing_height=");
      builder.append(this.swing_height_);      builder.append(", ");
      builder.append("swing_duration=");
      builder.append(this.swing_duration_);      builder.append(", ");
      builder.append("double_support_fraction=");
      builder.append(this.double_support_fraction_);
      builder.append("}");
      return builder.toString();
   }
}
