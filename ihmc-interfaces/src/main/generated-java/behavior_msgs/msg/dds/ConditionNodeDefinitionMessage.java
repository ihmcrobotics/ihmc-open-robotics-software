package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * COUNTER TYPE
       * LLM TYPE
       * PROXIMITY TYPE
       */
public class ConditionNodeDefinitionMessage extends Packet<ConditionNodeDefinitionMessage> implements Settable<ConditionNodeDefinitionMessage>, EpsilonComparable<ConditionNodeDefinitionMessage>
{
   public static final byte COUNTER_TYPE = (byte) 0;
   public static final byte LLM_TYPE = (byte) 1;
   public static final byte PROXIMITY_TYPE = (byte) 2;
   public static final byte ALWAYS_FAIL = (byte) 3;
   public static final byte ALWAYS_SUCCEED = (byte) 4;
   public static final byte PROXIMITY_XYZ = (byte) 0;
   public static final byte PROXIMITY_XY = (byte) 1;
   public static final byte PROXIMITY_Z = (byte) 2;
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.LeafNodeDefinitionMessage definition_;
   /**
            * The type of condition as defined above
            */
   public byte type_;
   /**
            * The number of times to fail before passing
            */
   public long count_to_;
   /**
            * Whether to reset the context on each run
            */
   public boolean reset_context_each_run_;
   /**
            * Whether to generate behavior state information and add it to the prompt
            */
   public boolean inject_behavior_state_;
   /**
            * Whether to generate environment state information and add it to the prompt
            */
   public boolean inject_environment_state_;
   /**
            * If a response match indicates success, else a match indicates failure
            */
   public boolean match_is_success_;
   /**
            * The prompt given once at the very beginning
            */
   public java.lang.StringBuilder system_;
   /**
            * The prompt defining how to make the boolean decision
            */
   public java.lang.StringBuilder prompt_;
   /**
            * A regular expression, where if it matches, the execution failed
            */
   public java.lang.StringBuilder response_matcher_;
   /**
            * The type of distance condition as defined above
            */
   public byte proximity_distance_type_;
   /**
            * Name of the object frame
            */
   public java.lang.StringBuilder proximity_object_frame_name_;
   /**
            * Name of the frame the distance is expressed in
            */
   public java.lang.StringBuilder proximity_reference_frame_name_;
   /**
            * The maximum distance between the object and the reference frame
            */
   public double proximity_distance_to_object_;
   /**
            * The maximum time that is spent in evaluating the condition
            */
   public double proximity_evaluation_time_;
   /**
            * Whether the failure of missing frame is handled internally by the condition node or not
            */
   public boolean manage_missing_frame_internally_;

   public ConditionNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.LeafNodeDefinitionMessage();
      system_ = new java.lang.StringBuilder(10000);
      prompt_ = new java.lang.StringBuilder(10000);
      response_matcher_ = new java.lang.StringBuilder(10000);
      proximity_object_frame_name_ = new java.lang.StringBuilder(255);
      proximity_reference_frame_name_ = new java.lang.StringBuilder(255);
   }

   public ConditionNodeDefinitionMessage(ConditionNodeDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(ConditionNodeDefinitionMessage other)
   {
      behavior_msgs.msg.dds.LeafNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      type_ = other.type_;

      count_to_ = other.count_to_;

      reset_context_each_run_ = other.reset_context_each_run_;

      inject_behavior_state_ = other.inject_behavior_state_;

      inject_environment_state_ = other.inject_environment_state_;

      match_is_success_ = other.match_is_success_;

      system_.setLength(0);
      system_.append(other.system_);

      prompt_.setLength(0);
      prompt_.append(other.prompt_);

      response_matcher_.setLength(0);
      response_matcher_.append(other.response_matcher_);

      proximity_distance_type_ = other.proximity_distance_type_;

      proximity_object_frame_name_.setLength(0);
      proximity_object_frame_name_.append(other.proximity_object_frame_name_);

      proximity_reference_frame_name_.setLength(0);
      proximity_reference_frame_name_.append(other.proximity_reference_frame_name_);

      proximity_distance_to_object_ = other.proximity_distance_to_object_;

      proximity_evaluation_time_ = other.proximity_evaluation_time_;

      manage_missing_frame_internally_ = other.manage_missing_frame_internally_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.LeafNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * The type of condition as defined above
            */
   public void setType(byte type)
   {
      type_ = type;
   }
   /**
            * The type of condition as defined above
            */
   public byte getType()
   {
      return type_;
   }

   /**
            * The number of times to fail before passing
            */
   public void setCountTo(long count_to)
   {
      count_to_ = count_to;
   }
   /**
            * The number of times to fail before passing
            */
   public long getCountTo()
   {
      return count_to_;
   }

   /**
            * Whether to reset the context on each run
            */
   public void setResetContextEachRun(boolean reset_context_each_run)
   {
      reset_context_each_run_ = reset_context_each_run;
   }
   /**
            * Whether to reset the context on each run
            */
   public boolean getResetContextEachRun()
   {
      return reset_context_each_run_;
   }

   /**
            * Whether to generate behavior state information and add it to the prompt
            */
   public void setInjectBehaviorState(boolean inject_behavior_state)
   {
      inject_behavior_state_ = inject_behavior_state;
   }
   /**
            * Whether to generate behavior state information and add it to the prompt
            */
   public boolean getInjectBehaviorState()
   {
      return inject_behavior_state_;
   }

   /**
            * Whether to generate environment state information and add it to the prompt
            */
   public void setInjectEnvironmentState(boolean inject_environment_state)
   {
      inject_environment_state_ = inject_environment_state;
   }
   /**
            * Whether to generate environment state information and add it to the prompt
            */
   public boolean getInjectEnvironmentState()
   {
      return inject_environment_state_;
   }

   /**
            * If a response match indicates success, else a match indicates failure
            */
   public void setMatchIsSuccess(boolean match_is_success)
   {
      match_is_success_ = match_is_success;
   }
   /**
            * If a response match indicates success, else a match indicates failure
            */
   public boolean getMatchIsSuccess()
   {
      return match_is_success_;
   }

   /**
            * The prompt given once at the very beginning
            */
   public void setSystem(java.lang.String system)
   {
      system_.setLength(0);
      system_.append(system);
   }

   /**
            * The prompt given once at the very beginning
            */
   public java.lang.String getSystemAsString()
   {
      return getSystem().toString();
   }
   /**
            * The prompt given once at the very beginning
            */
   public java.lang.StringBuilder getSystem()
   {
      return system_;
   }

   /**
            * The prompt defining how to make the boolean decision
            */
   public void setPrompt(java.lang.String prompt)
   {
      prompt_.setLength(0);
      prompt_.append(prompt);
   }

   /**
            * The prompt defining how to make the boolean decision
            */
   public java.lang.String getPromptAsString()
   {
      return getPrompt().toString();
   }
   /**
            * The prompt defining how to make the boolean decision
            */
   public java.lang.StringBuilder getPrompt()
   {
      return prompt_;
   }

   /**
            * A regular expression, where if it matches, the execution failed
            */
   public void setResponseMatcher(java.lang.String response_matcher)
   {
      response_matcher_.setLength(0);
      response_matcher_.append(response_matcher);
   }

   /**
            * A regular expression, where if it matches, the execution failed
            */
   public java.lang.String getResponseMatcherAsString()
   {
      return getResponseMatcher().toString();
   }
   /**
            * A regular expression, where if it matches, the execution failed
            */
   public java.lang.StringBuilder getResponseMatcher()
   {
      return response_matcher_;
   }

   /**
            * The type of distance condition as defined above
            */
   public void setProximityDistanceType(byte proximity_distance_type)
   {
      proximity_distance_type_ = proximity_distance_type;
   }
   /**
            * The type of distance condition as defined above
            */
   public byte getProximityDistanceType()
   {
      return proximity_distance_type_;
   }

   /**
            * Name of the object frame
            */
   public void setProximityObjectFrameName(java.lang.String proximity_object_frame_name)
   {
      proximity_object_frame_name_.setLength(0);
      proximity_object_frame_name_.append(proximity_object_frame_name);
   }

   /**
            * Name of the object frame
            */
   public java.lang.String getProximityObjectFrameNameAsString()
   {
      return getProximityObjectFrameName().toString();
   }
   /**
            * Name of the object frame
            */
   public java.lang.StringBuilder getProximityObjectFrameName()
   {
      return proximity_object_frame_name_;
   }

   /**
            * Name of the frame the distance is expressed in
            */
   public void setProximityReferenceFrameName(java.lang.String proximity_reference_frame_name)
   {
      proximity_reference_frame_name_.setLength(0);
      proximity_reference_frame_name_.append(proximity_reference_frame_name);
   }

   /**
            * Name of the frame the distance is expressed in
            */
   public java.lang.String getProximityReferenceFrameNameAsString()
   {
      return getProximityReferenceFrameName().toString();
   }
   /**
            * Name of the frame the distance is expressed in
            */
   public java.lang.StringBuilder getProximityReferenceFrameName()
   {
      return proximity_reference_frame_name_;
   }

   /**
            * The maximum distance between the object and the reference frame
            */
   public void setProximityDistanceToObject(double proximity_distance_to_object)
   {
      proximity_distance_to_object_ = proximity_distance_to_object;
   }
   /**
            * The maximum distance between the object and the reference frame
            */
   public double getProximityDistanceToObject()
   {
      return proximity_distance_to_object_;
   }

   /**
            * The maximum time that is spent in evaluating the condition
            */
   public void setProximityEvaluationTime(double proximity_evaluation_time)
   {
      proximity_evaluation_time_ = proximity_evaluation_time;
   }
   /**
            * The maximum time that is spent in evaluating the condition
            */
   public double getProximityEvaluationTime()
   {
      return proximity_evaluation_time_;
   }

   /**
            * Whether the failure of missing frame is handled internally by the condition node or not
            */
   public void setManageMissingFrameInternally(boolean manage_missing_frame_internally)
   {
      manage_missing_frame_internally_ = manage_missing_frame_internally;
   }
   /**
            * Whether the failure of missing frame is handled internally by the condition node or not
            */
   public boolean getManageMissingFrameInternally()
   {
      return manage_missing_frame_internally_;
   }


   public static Supplier<ConditionNodeDefinitionMessagePubSubType> getPubSubType()
   {
      return ConditionNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ConditionNodeDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ConditionNodeDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.type_, other.type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.count_to_, other.count_to_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.reset_context_each_run_, other.reset_context_each_run_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.inject_behavior_state_, other.inject_behavior_state_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.inject_environment_state_, other.inject_environment_state_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.match_is_success_, other.match_is_success_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.system_, other.system_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.prompt_, other.prompt_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.response_matcher_, other.response_matcher_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.proximity_distance_type_, other.proximity_distance_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.proximity_object_frame_name_, other.proximity_object_frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.proximity_reference_frame_name_, other.proximity_reference_frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.proximity_distance_to_object_, other.proximity_distance_to_object_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.proximity_evaluation_time_, other.proximity_evaluation_time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.manage_missing_frame_internally_, other.manage_missing_frame_internally_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ConditionNodeDefinitionMessage)) return false;

      ConditionNodeDefinitionMessage otherMyClass = (ConditionNodeDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.type_ != otherMyClass.type_) return false;

      if(this.count_to_ != otherMyClass.count_to_) return false;

      if(this.reset_context_each_run_ != otherMyClass.reset_context_each_run_) return false;

      if(this.inject_behavior_state_ != otherMyClass.inject_behavior_state_) return false;

      if(this.inject_environment_state_ != otherMyClass.inject_environment_state_) return false;

      if(this.match_is_success_ != otherMyClass.match_is_success_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.system_, otherMyClass.system_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.prompt_, otherMyClass.prompt_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.response_matcher_, otherMyClass.response_matcher_)) return false;

      if(this.proximity_distance_type_ != otherMyClass.proximity_distance_type_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.proximity_object_frame_name_, otherMyClass.proximity_object_frame_name_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.proximity_reference_frame_name_, otherMyClass.proximity_reference_frame_name_)) return false;

      if(this.proximity_distance_to_object_ != otherMyClass.proximity_distance_to_object_) return false;

      if(this.proximity_evaluation_time_ != otherMyClass.proximity_evaluation_time_) return false;

      if(this.manage_missing_frame_internally_ != otherMyClass.manage_missing_frame_internally_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ConditionNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("type=");
      builder.append(this.type_);      builder.append(", ");
      builder.append("count_to=");
      builder.append(this.count_to_);      builder.append(", ");
      builder.append("reset_context_each_run=");
      builder.append(this.reset_context_each_run_);      builder.append(", ");
      builder.append("inject_behavior_state=");
      builder.append(this.inject_behavior_state_);      builder.append(", ");
      builder.append("inject_environment_state=");
      builder.append(this.inject_environment_state_);      builder.append(", ");
      builder.append("match_is_success=");
      builder.append(this.match_is_success_);      builder.append(", ");
      builder.append("system=");
      builder.append(this.system_);      builder.append(", ");
      builder.append("prompt=");
      builder.append(this.prompt_);      builder.append(", ");
      builder.append("response_matcher=");
      builder.append(this.response_matcher_);      builder.append(", ");
      builder.append("proximity_distance_type=");
      builder.append(this.proximity_distance_type_);      builder.append(", ");
      builder.append("proximity_object_frame_name=");
      builder.append(this.proximity_object_frame_name_);      builder.append(", ");
      builder.append("proximity_reference_frame_name=");
      builder.append(this.proximity_reference_frame_name_);      builder.append(", ");
      builder.append("proximity_distance_to_object=");
      builder.append(this.proximity_distance_to_object_);      builder.append(", ");
      builder.append("proximity_evaluation_time=");
      builder.append(this.proximity_evaluation_time_);      builder.append(", ");
      builder.append("manage_missing_frame_internally=");
      builder.append(this.manage_missing_frame_internally_);
      builder.append("}");
      return builder.toString();
   }
}
