package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class PushRecoveryResultMessage extends Packet<PushRecoveryResultMessage> implements Settable<PushRecoveryResultMessage>, EpsilonComparable<PushRecoveryResultMessage>
{
   public controller_msgs.msg.dds.FootstepDataListMessage recovery_steps_;
   public boolean is_step_recoverable_;
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionsListMessage>  step_constraint_list_;

   public PushRecoveryResultMessage()
   {
      recovery_steps_ = new controller_msgs.msg.dds.FootstepDataListMessage();
      step_constraint_list_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionsListMessage> (5, new controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType());

   }

   public PushRecoveryResultMessage(PushRecoveryResultMessage other)
   {
      this();
      set(other);
   }

   public void set(PushRecoveryResultMessage other)
   {
      controller_msgs.msg.dds.FootstepDataListMessagePubSubType.staticCopy(other.recovery_steps_, recovery_steps_);
      is_step_recoverable_ = other.is_step_recoverable_;

      step_constraint_list_.set(other.step_constraint_list_);
   }


   public controller_msgs.msg.dds.FootstepDataListMessage getRecoverySteps()
   {
      return recovery_steps_;
   }

   public void setIsStepRecoverable(boolean is_step_recoverable)
   {
      is_step_recoverable_ = is_step_recoverable;
   }
   public boolean getIsStepRecoverable()
   {
      return is_step_recoverable_;
   }


   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.PlanarRegionsListMessage>  getStepConstraintList()
   {
      return step_constraint_list_;
   }


   public static Supplier<PushRecoveryResultMessagePubSubType> getPubSubType()
   {
      return PushRecoveryResultMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PushRecoveryResultMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PushRecoveryResultMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.recovery_steps_.epsilonEquals(other.recovery_steps_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_step_recoverable_, other.is_step_recoverable_, epsilon)) return false;

      if (this.step_constraint_list_.size() != other.step_constraint_list_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.step_constraint_list_.size(); i++)
         {  if (!this.step_constraint_list_.get(i).epsilonEquals(other.step_constraint_list_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PushRecoveryResultMessage)) return false;

      PushRecoveryResultMessage otherMyClass = (PushRecoveryResultMessage) other;

      if (!this.recovery_steps_.equals(otherMyClass.recovery_steps_)) return false;
      if(this.is_step_recoverable_ != otherMyClass.is_step_recoverable_) return false;

      if (!this.step_constraint_list_.equals(otherMyClass.step_constraint_list_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PushRecoveryResultMessage {");
      builder.append("recovery_steps=");
      builder.append(this.recovery_steps_);      builder.append(", ");
      builder.append("is_step_recoverable=");
      builder.append(this.is_step_recoverable_);      builder.append(", ");
      builder.append("step_constraint_list=");
      builder.append(this.step_constraint_list_);
      builder.append("}");
      return builder.toString();
   }
}
