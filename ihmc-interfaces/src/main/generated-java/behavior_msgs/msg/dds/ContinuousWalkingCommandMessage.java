package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ContinuousWalkingCommandMessage extends Packet<ContinuousWalkingCommandMessage> implements Settable<ContinuousWalkingCommandMessage>, EpsilonComparable<ContinuousWalkingCommandMessage>
{
   /**
            * flag to enable/disable publishing to controller
            */
   public boolean publish_to_controller_;
   /**
            * flag to enable/disable continuous walking state machine
            */
   public boolean enable_continuous_walking_;
   /**
            * number of steps to send to controller
            */
   public int number_of_steps_to_send_;

   public ContinuousWalkingCommandMessage()
   {
   }

   public ContinuousWalkingCommandMessage(ContinuousWalkingCommandMessage other)
   {
      this();
      set(other);
   }

   public void set(ContinuousWalkingCommandMessage other)
   {
      publish_to_controller_ = other.publish_to_controller_;

      enable_continuous_walking_ = other.enable_continuous_walking_;

      number_of_steps_to_send_ = other.number_of_steps_to_send_;

   }

   /**
            * flag to enable/disable publishing to controller
            */
   public void setPublishToController(boolean publish_to_controller)
   {
      publish_to_controller_ = publish_to_controller;
   }
   /**
            * flag to enable/disable publishing to controller
            */
   public boolean getPublishToController()
   {
      return publish_to_controller_;
   }

   /**
            * flag to enable/disable continuous walking state machine
            */
   public void setEnableContinuousWalking(boolean enable_continuous_walking)
   {
      enable_continuous_walking_ = enable_continuous_walking;
   }
   /**
            * flag to enable/disable continuous walking state machine
            */
   public boolean getEnableContinuousWalking()
   {
      return enable_continuous_walking_;
   }

   /**
            * number of steps to send to controller
            */
   public void setNumberOfStepsToSend(int number_of_steps_to_send)
   {
      number_of_steps_to_send_ = number_of_steps_to_send;
   }
   /**
            * number of steps to send to controller
            */
   public int getNumberOfStepsToSend()
   {
      return number_of_steps_to_send_;
   }


   public static Supplier<ContinuousWalkingCommandMessagePubSubType> getPubSubType()
   {
      return ContinuousWalkingCommandMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ContinuousWalkingCommandMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ContinuousWalkingCommandMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.publish_to_controller_, other.publish_to_controller_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_continuous_walking_, other.enable_continuous_walking_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.number_of_steps_to_send_, other.number_of_steps_to_send_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ContinuousWalkingCommandMessage)) return false;

      ContinuousWalkingCommandMessage otherMyClass = (ContinuousWalkingCommandMessage) other;

      if(this.publish_to_controller_ != otherMyClass.publish_to_controller_) return false;

      if(this.enable_continuous_walking_ != otherMyClass.enable_continuous_walking_) return false;

      if(this.number_of_steps_to_send_ != otherMyClass.number_of_steps_to_send_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ContinuousWalkingCommandMessage {");
      builder.append("publish_to_controller=");
      builder.append(this.publish_to_controller_);      builder.append(", ");
      builder.append("enable_continuous_walking=");
      builder.append(this.enable_continuous_walking_);      builder.append(", ");
      builder.append("number_of_steps_to_send=");
      builder.append(this.number_of_steps_to_send_);
      builder.append("}");
      return builder.toString();
   }
}
