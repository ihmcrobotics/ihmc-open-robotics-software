package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A message for remotely operationg the lerobot policies
       */
public class LerobotInferenceOperationMessage extends Packet<LerobotInferenceOperationMessage> implements Settable<LerobotInferenceOperationMessage>, EpsilonComparable<LerobotInferenceOperationMessage>
{
   /**
            * Allows the user to operate the inference of visuomotor policies
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage latest_timestamp_modifiable_;
   /**
            * Whether the inference and action output to IK streaming is active
            */
   public boolean running_;

   public LerobotInferenceOperationMessage()
   {
      latest_timestamp_modifiable_ = new ihmc_common_msgs.msg.dds.LatestModificationMessage();
   }

   public LerobotInferenceOperationMessage(LerobotInferenceOperationMessage other)
   {
      this();
      set(other);
   }

   public void set(LerobotInferenceOperationMessage other)
   {
      ihmc_common_msgs.msg.dds.LatestModificationMessagePubSubType.staticCopy(other.latest_timestamp_modifiable_, latest_timestamp_modifiable_);
      running_ = other.running_;

   }


   /**
            * Allows the user to operate the inference of visuomotor policies
            */
   public ihmc_common_msgs.msg.dds.LatestModificationMessage getLatestTimestampModifiable()
   {
      return latest_timestamp_modifiable_;
   }

   /**
            * Whether the inference and action output to IK streaming is active
            */
   public void setRunning(boolean running)
   {
      running_ = running;
   }
   /**
            * Whether the inference and action output to IK streaming is active
            */
   public boolean getRunning()
   {
      return running_;
   }


   public static Supplier<LerobotInferenceOperationMessagePubSubType> getPubSubType()
   {
      return LerobotInferenceOperationMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return LerobotInferenceOperationMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(LerobotInferenceOperationMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.latest_timestamp_modifiable_.epsilonEquals(other.latest_timestamp_modifiable_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.running_, other.running_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof LerobotInferenceOperationMessage)) return false;

      LerobotInferenceOperationMessage otherMyClass = (LerobotInferenceOperationMessage) other;

      if (!this.latest_timestamp_modifiable_.equals(otherMyClass.latest_timestamp_modifiable_)) return false;
      if(this.running_ != otherMyClass.running_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("LerobotInferenceOperationMessage {");
      builder.append("latest_timestamp_modifiable=");
      builder.append(this.latest_timestamp_modifiable_);      builder.append(", ");
      builder.append("running=");
      builder.append(this.running_);
      builder.append("}");
      return builder.toString();
   }
}
