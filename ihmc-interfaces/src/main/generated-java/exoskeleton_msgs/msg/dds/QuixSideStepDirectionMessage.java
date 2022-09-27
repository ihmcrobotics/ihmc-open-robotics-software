package exoskeleton_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the Quix controller API.
       */
public class QuixSideStepDirectionMessage extends Packet<QuixSideStepDirectionMessage> implements Settable<QuixSideStepDirectionMessage>, EpsilonComparable<QuixSideStepDirectionMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   public byte step_direction_ = (byte) 255;

   public QuixSideStepDirectionMessage()
   {
   }

   public QuixSideStepDirectionMessage(QuixSideStepDirectionMessage other)
   {
      this();
      set(other);
   }

   public void set(QuixSideStepDirectionMessage other)
   {
      step_direction_ = other.step_direction_;

   }

   public void setStepDirection(byte step_direction)
   {
      step_direction_ = step_direction;
   }
   public byte getStepDirection()
   {
      return step_direction_;
   }


   public static Supplier<QuixSideStepDirectionMessagePubSubType> getPubSubType()
   {
      return QuixSideStepDirectionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return QuixSideStepDirectionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(QuixSideStepDirectionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_direction_, other.step_direction_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof QuixSideStepDirectionMessage)) return false;

      QuixSideStepDirectionMessage otherMyClass = (QuixSideStepDirectionMessage) other;

      if(this.step_direction_ != otherMyClass.step_direction_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("QuixSideStepDirectionMessage {");
      builder.append("step_direction=");
      builder.append(this.step_direction_);
      builder.append("}");
      return builder.toString();
   }
}
