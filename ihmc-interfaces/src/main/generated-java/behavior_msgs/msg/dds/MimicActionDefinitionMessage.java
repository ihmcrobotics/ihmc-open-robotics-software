package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class MimicActionDefinitionMessage extends Packet<MimicActionDefinitionMessage> implements Settable<MimicActionDefinitionMessage>, EpsilonComparable<MimicActionDefinitionMessage>
{
   public static final byte POLICY_TRANSITION = (byte) 0;
   public static final byte EXIT_POLICY = (byte) 1;
   public static final byte EXECUTE_POLICY = (byte) 2;
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * The type of action as defined above
            */
   public byte mimic_action_type_;
   /**
            * The name of the mimic file
            */
   public java.lang.StringBuilder mimic_file_name_;
   /**
            * Wait time in seconds after the policy has been exited
            */
   public double wait_time_exit_policy_;

   public MimicActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      mimic_file_name_ = new java.lang.StringBuilder(255);
   }

   public MimicActionDefinitionMessage(MimicActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(MimicActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      mimic_action_type_ = other.mimic_action_type_;

      mimic_file_name_.setLength(0);
      mimic_file_name_.append(other.mimic_file_name_);

      wait_time_exit_policy_ = other.wait_time_exit_policy_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * The type of action as defined above
            */
   public void setMimicActionType(byte mimic_action_type)
   {
      mimic_action_type_ = mimic_action_type;
   }
   /**
            * The type of action as defined above
            */
   public byte getMimicActionType()
   {
      return mimic_action_type_;
   }

   /**
            * The name of the mimic file
            */
   public void setMimicFileName(java.lang.String mimic_file_name)
   {
      mimic_file_name_.setLength(0);
      mimic_file_name_.append(mimic_file_name);
   }

   /**
            * The name of the mimic file
            */
   public java.lang.String getMimicFileNameAsString()
   {
      return getMimicFileName().toString();
   }
   /**
            * The name of the mimic file
            */
   public java.lang.StringBuilder getMimicFileName()
   {
      return mimic_file_name_;
   }

   /**
            * Wait time in seconds after the policy has been exited
            */
   public void setWaitTimeExitPolicy(double wait_time_exit_policy)
   {
      wait_time_exit_policy_ = wait_time_exit_policy;
   }
   /**
            * Wait time in seconds after the policy has been exited
            */
   public double getWaitTimeExitPolicy()
   {
      return wait_time_exit_policy_;
   }


   public static Supplier<MimicActionDefinitionMessagePubSubType> getPubSubType()
   {
      return MimicActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MimicActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MimicActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.mimic_action_type_, other.mimic_action_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.mimic_file_name_, other.mimic_file_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wait_time_exit_policy_, other.wait_time_exit_policy_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MimicActionDefinitionMessage)) return false;

      MimicActionDefinitionMessage otherMyClass = (MimicActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.mimic_action_type_ != otherMyClass.mimic_action_type_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.mimic_file_name_, otherMyClass.mimic_file_name_)) return false;

      if(this.wait_time_exit_policy_ != otherMyClass.wait_time_exit_policy_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MimicActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("mimic_action_type=");
      builder.append(this.mimic_action_type_);      builder.append(", ");
      builder.append("mimic_file_name=");
      builder.append(this.mimic_file_name_);      builder.append(", ");
      builder.append("wait_time_exit_policy=");
      builder.append(this.wait_time_exit_policy_);
      builder.append("}");
      return builder.toString();
   }
}
