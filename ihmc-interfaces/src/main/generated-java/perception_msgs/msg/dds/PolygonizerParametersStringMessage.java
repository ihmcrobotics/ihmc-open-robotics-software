package perception_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * String serialized polygonizer parameters
       * Might be kinda long, so give it some space
       */
public class PolygonizerParametersStringMessage extends Packet<PolygonizerParametersStringMessage> implements Settable<PolygonizerParametersStringMessage>, EpsilonComparable<PolygonizerParametersStringMessage>
{
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  parameters_;

   public PolygonizerParametersStringMessage()
   {
      parameters_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (2056, "type_d");
   }

   public PolygonizerParametersStringMessage(PolygonizerParametersStringMessage other)
   {
      this();
      set(other);
   }

   public void set(PolygonizerParametersStringMessage other)
   {
      parameters_.set(other.parameters_);
   }


   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getParameters()
   {
      return parameters_;
   }


   public static Supplier<PolygonizerParametersStringMessagePubSubType> getPubSubType()
   {
      return PolygonizerParametersStringMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PolygonizerParametersStringMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PolygonizerParametersStringMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilderSequence(this.parameters_, other.parameters_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PolygonizerParametersStringMessage)) return false;

      PolygonizerParametersStringMessage otherMyClass = (PolygonizerParametersStringMessage) other;

      if (!this.parameters_.equals(otherMyClass.parameters_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PolygonizerParametersStringMessage {");
      builder.append("parameters=");
      builder.append(this.parameters_);
      builder.append("}");
      return builder.toString();
   }
}
