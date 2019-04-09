package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message sets the parameters of the FlatGroundPlanarRegionPublisher
       */
public class FlatGroundPlanarRegionParametersMessage extends Packet<FlatGroundPlanarRegionParametersMessage> implements Settable<FlatGroundPlanarRegionParametersMessage>, EpsilonComparable<FlatGroundPlanarRegionParametersMessage>
{
   /**
            * Enables/Disables the FlatGroundPlanarRegionPublisher module
            */
   public boolean enable_;

   public FlatGroundPlanarRegionParametersMessage()
   {
   }

   public FlatGroundPlanarRegionParametersMessage(FlatGroundPlanarRegionParametersMessage other)
   {
      this();
      set(other);
   }

   public void set(FlatGroundPlanarRegionParametersMessage other)
   {
      enable_ = other.enable_;

   }

   /**
            * Enables/Disables the FlatGroundPlanarRegionPublisher module
            */
   public void setEnable(boolean enable)
   {
      enable_ = enable;
   }
   /**
            * Enables/Disables the FlatGroundPlanarRegionPublisher module
            */
   public boolean getEnable()
   {
      return enable_;
   }


   public static Supplier<FlatGroundPlanarRegionParametersMessagePubSubType> getPubSubType()
   {
      return FlatGroundPlanarRegionParametersMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FlatGroundPlanarRegionParametersMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FlatGroundPlanarRegionParametersMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_, other.enable_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FlatGroundPlanarRegionParametersMessage)) return false;

      FlatGroundPlanarRegionParametersMessage otherMyClass = (FlatGroundPlanarRegionParametersMessage) other;

      if(this.enable_ != otherMyClass.enable_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FlatGroundPlanarRegionParametersMessage {");
      builder.append("enable=");
      builder.append(this.enable_);
      builder.append("}");
      return builder.toString();
   }
}
