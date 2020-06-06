package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message sets the parameters of the BipedalSupportPlanarRegionPublisher
       */
public class BipedalSupportPlanarRegionParametersMessage extends Packet<BipedalSupportPlanarRegionParametersMessage> implements Settable<BipedalSupportPlanarRegionParametersMessage>, EpsilonComparable<BipedalSupportPlanarRegionParametersMessage>
{

   /**
            * Enables the BipedalSupportPlanarRegionPublish module
            */
   public boolean enable_;

   /**
            * The support planar region is a scaled version of the actual support region.
            * This value specifies how much to scale by.
            */
   public double support_region_scale_factor_ = 1.0;

   public BipedalSupportPlanarRegionParametersMessage()
   {



   }

   public BipedalSupportPlanarRegionParametersMessage(BipedalSupportPlanarRegionParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(BipedalSupportPlanarRegionParametersMessage other)
   {

      enable_ = other.enable_;


      support_region_scale_factor_ = other.support_region_scale_factor_;

   }


   /**
            * Enables the BipedalSupportPlanarRegionPublish module
            */
   public void setEnable(boolean enable)
   {
      enable_ = enable;
   }
   /**
            * Enables the BipedalSupportPlanarRegionPublish module
            */
   public boolean getEnable()
   {
      return enable_;
   }


   /**
            * The support planar region is a scaled version of the actual support region.
            * This value specifies how much to scale by.
            */
   public void setSupportRegionScaleFactor(double support_region_scale_factor)
   {
      support_region_scale_factor_ = support_region_scale_factor;
   }
   /**
            * The support planar region is a scaled version of the actual support region.
            * This value specifies how much to scale by.
            */
   public double getSupportRegionScaleFactor()
   {
      return support_region_scale_factor_;
   }


   public static Supplier<BipedalSupportPlanarRegionParametersMessagePubSubType> getPubSubType()
   {
      return BipedalSupportPlanarRegionParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return BipedalSupportPlanarRegionParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(BipedalSupportPlanarRegionParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_, other.enable_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.support_region_scale_factor_, other.support_region_scale_factor_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof BipedalSupportPlanarRegionParametersMessage)) return false;

      BipedalSupportPlanarRegionParametersMessage otherMyClass = (BipedalSupportPlanarRegionParametersMessage) other;


      if(this.enable_ != otherMyClass.enable_) return false;


      if(this.support_region_scale_factor_ != otherMyClass.support_region_scale_factor_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BipedalSupportPlanarRegionParametersMessage {");

      builder.append("enable=");
      builder.append(this.enable_);      builder.append(", ");

      builder.append("support_region_scale_factor=");
      builder.append(this.support_region_scale_factor_);
      builder.append("}");
      return builder.toString();
   }
}
