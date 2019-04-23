package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message sets the parameters of the BipedalSupportPlanarRegionPublisher
       */
public class QuadrupedSupportPlanarRegionParametersMessage extends Packet<QuadrupedSupportPlanarRegionParametersMessage> implements Settable<QuadrupedSupportPlanarRegionParametersMessage>, EpsilonComparable<QuadrupedSupportPlanarRegionParametersMessage>
{
   /**
            * Enables the QuadrupedSupportPlanarRegionPublish module
            */
   public boolean enable_;
   /**
            * The support planar region is set to be a square at each of the feet.
            * This value specifies the half length of the side of teh square.
            */
   public double support_region_size_ = 1.0;

   public QuadrupedSupportPlanarRegionParametersMessage()
   {
   }

   public QuadrupedSupportPlanarRegionParametersMessage(QuadrupedSupportPlanarRegionParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(QuadrupedSupportPlanarRegionParametersMessage other)
   {
      enable_ = other.enable_;

      support_region_size_ = other.support_region_size_;

   }

   /**
            * Enables the QuadrupedSupportPlanarRegionPublish module
            */
   public void setEnable(boolean enable)
   {
      enable_ = enable;
   }
   /**
            * Enables the QuadrupedSupportPlanarRegionPublish module
            */
   public boolean getEnable()
   {
      return enable_;
   }

   /**
            * The support planar region is set to be a square at each of the feet.
            * This value specifies the half length of the side of teh square.
            */
   public void setSupportRegionSize(double support_region_size)
   {
      support_region_size_ = support_region_size;
   }
   /**
            * The support planar region is set to be a square at each of the feet.
            * This value specifies the half length of the side of teh square.
            */
   public double getSupportRegionSize()
   {
      return support_region_size_;
   }


   public static Supplier<QuadrupedSupportPlanarRegionParametersMessagePubSubType> getPubSubType()
   {
      return QuadrupedSupportPlanarRegionParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuadrupedSupportPlanarRegionParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuadrupedSupportPlanarRegionParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_, other.enable_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.support_region_size_, other.support_region_size_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuadrupedSupportPlanarRegionParametersMessage)) return false;

      QuadrupedSupportPlanarRegionParametersMessage otherMyClass = (QuadrupedSupportPlanarRegionParametersMessage) other;

      if(this.enable_ != otherMyClass.enable_) return false;

      if(this.support_region_size_ != otherMyClass.support_region_size_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedSupportPlanarRegionParametersMessage {");
      builder.append("enable=");
      builder.append(this.enable_);      builder.append(", ");
      builder.append("support_region_size=");
      builder.append(this.support_region_size_);
      builder.append("}");
      return builder.toString();
   }
}
