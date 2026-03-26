package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message is used to request an initialization fo the state estimator.
       */
public class ReinitializeStateEstimatorMessage extends Packet<ReinitializeStateEstimatorMessage> implements Settable<ReinitializeStateEstimatorMessage>, EpsilonComparable<ReinitializeStateEstimatorMessage>
{
   public boolean request_reinitialize_;

   public ReinitializeStateEstimatorMessage()
   {
   }

   public ReinitializeStateEstimatorMessage(ReinitializeStateEstimatorMessage other)
   {
      this();
      set(other);
   }

   public void set(ReinitializeStateEstimatorMessage other)
   {
      request_reinitialize_ = other.request_reinitialize_;

   }

   public void setRequestReinitialize(boolean request_reinitialize)
   {
      request_reinitialize_ = request_reinitialize;
   }
   public boolean getRequestReinitialize()
   {
      return request_reinitialize_;
   }


   public static Supplier<ReinitializeStateEstimatorMessagePubSubType> getPubSubType()
   {
      return ReinitializeStateEstimatorMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ReinitializeStateEstimatorMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ReinitializeStateEstimatorMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.request_reinitialize_, other.request_reinitialize_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ReinitializeStateEstimatorMessage)) return false;

      ReinitializeStateEstimatorMessage otherMyClass = (ReinitializeStateEstimatorMessage) other;

      if(this.request_reinitialize_ != otherMyClass.request_reinitialize_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ReinitializeStateEstimatorMessage {");
      builder.append("request_reinitialize=");
      builder.append(this.request_reinitialize_);
      builder.append("}");
      return builder.toString();
   }
}
