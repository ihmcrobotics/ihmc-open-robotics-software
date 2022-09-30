package quadruped_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message sets the parameters of the BipedalSupportPlanarRegionPublisher
       * The support planar region is set to be a rectangle at each of the feet.
       */
public class QuadrupedSupportPlanarRegionParametersMessage extends Packet<QuadrupedSupportPlanarRegionParametersMessage> implements Settable<QuadrupedSupportPlanarRegionParametersMessage>, EpsilonComparable<QuadrupedSupportPlanarRegionParametersMessage>
{
   /**
            * Enables the QuadrupedSupportPlanarRegionPublish module
            */
   public boolean enable_;
   /**
            * This value specifies the distance from the foot to the inside portion of the vertices (under the robot).
            */
   public double inside_support_region_size_ = -1.0;
   /**
            * This value specifies the distance from the foot to the outside portion of the vertices (not under the robot).
            */
   public double outside_support_region_size_ = -1.0;

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

      inside_support_region_size_ = other.inside_support_region_size_;

      outside_support_region_size_ = other.outside_support_region_size_;

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
            * This value specifies the distance from the foot to the inside portion of the vertices (under the robot).
            */
   public void setInsideSupportRegionSize(double inside_support_region_size)
   {
      inside_support_region_size_ = inside_support_region_size;
   }
   /**
            * This value specifies the distance from the foot to the inside portion of the vertices (under the robot).
            */
   public double getInsideSupportRegionSize()
   {
      return inside_support_region_size_;
   }

   /**
            * This value specifies the distance from the foot to the outside portion of the vertices (not under the robot).
            */
   public void setOutsideSupportRegionSize(double outside_support_region_size)
   {
      outside_support_region_size_ = outside_support_region_size;
   }
   /**
            * This value specifies the distance from the foot to the outside portion of the vertices (not under the robot).
            */
   public double getOutsideSupportRegionSize()
   {
      return outside_support_region_size_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.inside_support_region_size_, other.inside_support_region_size_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.outside_support_region_size_, other.outside_support_region_size_, epsilon)) return false;


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

      if(this.inside_support_region_size_ != otherMyClass.inside_support_region_size_) return false;

      if(this.outside_support_region_size_ != otherMyClass.outside_support_region_size_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuadrupedSupportPlanarRegionParametersMessage {");
      builder.append("enable=");
      builder.append(this.enable_);      builder.append(", ");
      builder.append("inside_support_region_size=");
      builder.append(this.inside_support_region_size_);      builder.append(", ");
      builder.append("outside_support_region_size=");
      builder.append(this.outside_support_region_size_);
      builder.append("}");
      return builder.toString();
   }
}
