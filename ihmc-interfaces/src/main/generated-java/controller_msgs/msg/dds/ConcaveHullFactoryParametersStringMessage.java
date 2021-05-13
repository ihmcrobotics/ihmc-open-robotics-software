package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * String serialized concave hull factory parameters
       * Might be kinda long, so give it some space
       */
public class ConcaveHullFactoryParametersStringMessage extends Packet<ConcaveHullFactoryParametersStringMessage> implements Settable<ConcaveHullFactoryParametersStringMessage>, EpsilonComparable<ConcaveHullFactoryParametersStringMessage>
{
   public us.ihmc.idl.IDLSequence.StringBuilderHolder  parameters_;

   public ConcaveHullFactoryParametersStringMessage()
   {
      parameters_ = new us.ihmc.idl.IDLSequence.StringBuilderHolder (2056, "type_d");
   }

   public ConcaveHullFactoryParametersStringMessage(ConcaveHullFactoryParametersStringMessage other)
   {
      this();
      set(other);
   }

   public void set(ConcaveHullFactoryParametersStringMessage other)
   {
      parameters_.set(other.parameters_);
   }


   public us.ihmc.idl.IDLSequence.StringBuilderHolder  getParameters()
   {
      return parameters_;
   }


   public static Supplier<ConcaveHullFactoryParametersStringMessagePubSubType> getPubSubType()
   {
      return ConcaveHullFactoryParametersStringMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ConcaveHullFactoryParametersStringMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ConcaveHullFactoryParametersStringMessage other, double epsilon)
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
      if(!(other instanceof ConcaveHullFactoryParametersStringMessage)) return false;

      ConcaveHullFactoryParametersStringMessage otherMyClass = (ConcaveHullFactoryParametersStringMessage) other;

      if (!this.parameters_.equals(otherMyClass.parameters_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ConcaveHullFactoryParametersStringMessage {");
      builder.append("parameters=");
      builder.append(this.parameters_);
      builder.append("}");
      return builder.toString();
   }
}
