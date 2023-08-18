package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is meant to communicate the status of currently executing actions
       */
public class ActionExecutionStatusMessage extends Packet<ActionExecutionStatusMessage> implements Settable<ActionExecutionStatusMessage>, EpsilonComparable<ActionExecutionStatusMessage>
{
   /**
            * Executing action index
            */
   public int action_index_;
   /**
            * Nominal execution duration
            */
   public double nominal_execution_duration_;
   /**
            * Time since execution started
            */
   public double elapsed_execution_time_;

   public ActionExecutionStatusMessage()
   {
   }

   public ActionExecutionStatusMessage(ActionExecutionStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(ActionExecutionStatusMessage other)
   {
      action_index_ = other.action_index_;

      nominal_execution_duration_ = other.nominal_execution_duration_;

      elapsed_execution_time_ = other.elapsed_execution_time_;

   }

   /**
            * Executing action index
            */
   public void setActionIndex(int action_index)
   {
      action_index_ = action_index;
   }
   /**
            * Executing action index
            */
   public int getActionIndex()
   {
      return action_index_;
   }

   /**
            * Nominal execution duration
            */
   public void setNominalExecutionDuration(double nominal_execution_duration)
   {
      nominal_execution_duration_ = nominal_execution_duration;
   }
   /**
            * Nominal execution duration
            */
   public double getNominalExecutionDuration()
   {
      return nominal_execution_duration_;
   }

   /**
            * Time since execution started
            */
   public void setElapsedExecutionTime(double elapsed_execution_time)
   {
      elapsed_execution_time_ = elapsed_execution_time;
   }
   /**
            * Time since execution started
            */
   public double getElapsedExecutionTime()
   {
      return elapsed_execution_time_;
   }


   public static Supplier<ActionExecutionStatusMessagePubSubType> getPubSubType()
   {
      return ActionExecutionStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ActionExecutionStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ActionExecutionStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.action_index_, other.action_index_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.nominal_execution_duration_, other.nominal_execution_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.elapsed_execution_time_, other.elapsed_execution_time_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ActionExecutionStatusMessage)) return false;

      ActionExecutionStatusMessage otherMyClass = (ActionExecutionStatusMessage) other;

      if(this.action_index_ != otherMyClass.action_index_) return false;

      if(this.nominal_execution_duration_ != otherMyClass.nominal_execution_duration_) return false;

      if(this.elapsed_execution_time_ != otherMyClass.elapsed_execution_time_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ActionExecutionStatusMessage {");
      builder.append("action_index=");
      builder.append(this.action_index_);      builder.append(", ");
      builder.append("nominal_execution_duration=");
      builder.append(this.nominal_execution_duration_);      builder.append(", ");
      builder.append("elapsed_execution_time=");
      builder.append(this.elapsed_execution_time_);
      builder.append("}");
      return builder.toString();
   }
}
