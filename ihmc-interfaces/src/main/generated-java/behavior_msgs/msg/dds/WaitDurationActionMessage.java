package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class WaitDurationActionMessage extends Packet<WaitDurationActionMessage> implements Settable<WaitDurationActionMessage>, EpsilonComparable<WaitDurationActionMessage>
{
   /**
            * Wait duration
            */
   public double wait_duration_;

   public WaitDurationActionMessage()
   {
   }

   public WaitDurationActionMessage(WaitDurationActionMessage other)
   {
      this();
      set(other);
   }

   public void set(WaitDurationActionMessage other)
   {
      wait_duration_ = other.wait_duration_;

   }

   /**
            * Wait duration
            */
   public void setWaitDuration(double wait_duration)
   {
      wait_duration_ = wait_duration;
   }
   /**
            * Wait duration
            */
   public double getWaitDuration()
   {
      return wait_duration_;
   }


   public static Supplier<WaitDurationActionMessagePubSubType> getPubSubType()
   {
      return WaitDurationActionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WaitDurationActionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WaitDurationActionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wait_duration_, other.wait_duration_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WaitDurationActionMessage)) return false;

      WaitDurationActionMessage otherMyClass = (WaitDurationActionMessage) other;

      if(this.wait_duration_ != otherMyClass.wait_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WaitDurationActionMessage {");
      builder.append("wait_duration=");
      builder.append(this.wait_duration_);
      builder.append("}");
      return builder.toString();
   }
}
