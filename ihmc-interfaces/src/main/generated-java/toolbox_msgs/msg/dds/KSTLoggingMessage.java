package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class KSTLoggingMessage extends Packet<KSTLoggingMessage> implements Settable<KSTLoggingMessage>, EpsilonComparable<KSTLoggingMessage>
{
   /**
            * If true, the KST toolbox will begin logging. If false, it will export if currently logging.
            */
   public boolean start_logging_;

   public KSTLoggingMessage()
   {
   }

   public KSTLoggingMessage(KSTLoggingMessage other)
   {
      this();
      set(other);
   }

   public void set(KSTLoggingMessage other)
   {
      start_logging_ = other.start_logging_;

   }

   /**
            * If true, the KST toolbox will begin logging. If false, it will export if currently logging.
            */
   public void setStartLogging(boolean start_logging)
   {
      start_logging_ = start_logging;
   }
   /**
            * If true, the KST toolbox will begin logging. If false, it will export if currently logging.
            */
   public boolean getStartLogging()
   {
      return start_logging_;
   }


   public static Supplier<KSTLoggingMessagePubSubType> getPubSubType()
   {
      return KSTLoggingMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KSTLoggingMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KSTLoggingMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.start_logging_, other.start_logging_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KSTLoggingMessage)) return false;

      KSTLoggingMessage otherMyClass = (KSTLoggingMessage) other;

      if(this.start_logging_ != otherMyClass.start_logging_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KSTLoggingMessage {");
      builder.append("start_logging=");
      builder.append(this.start_logging_);
      builder.append("}");
      return builder.toString();
   }
}
