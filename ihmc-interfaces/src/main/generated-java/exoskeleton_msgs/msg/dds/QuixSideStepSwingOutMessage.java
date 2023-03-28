package exoskeleton_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       */
public class QuixSideStepSwingOutMessage extends Packet<QuixSideStepSwingOutMessage> implements Settable<QuixSideStepSwingOutMessage>, EpsilonComparable<QuixSideStepSwingOutMessage>
{
   public boolean side_step_in_swing_out_;

   public QuixSideStepSwingOutMessage()
   {
   }

   public QuixSideStepSwingOutMessage(QuixSideStepSwingOutMessage other)
   {
      this();
      set(other);
   }

   public void set(QuixSideStepSwingOutMessage other)
   {
      side_step_in_swing_out_ = other.side_step_in_swing_out_;

   }

   public void setSideStepInSwingOut(boolean side_step_in_swing_out)
   {
      side_step_in_swing_out_ = side_step_in_swing_out;
   }
   public boolean getSideStepInSwingOut()
   {
      return side_step_in_swing_out_;
   }


   public static Supplier<QuixSideStepSwingOutMessagePubSubType> getPubSubType()
   {
      return QuixSideStepSwingOutMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixSideStepSwingOutMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixSideStepSwingOutMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.side_step_in_swing_out_, other.side_step_in_swing_out_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixSideStepSwingOutMessage)) return false;

      QuixSideStepSwingOutMessage otherMyClass = (QuixSideStepSwingOutMessage) other;

      if(this.side_step_in_swing_out_ != otherMyClass.side_step_in_swing_out_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixSideStepSwingOutMessage {");
      builder.append("side_step_in_swing_out=");
      builder.append(this.side_step_in_swing_out_);
      builder.append("}");
      return builder.toString();
   }
}
