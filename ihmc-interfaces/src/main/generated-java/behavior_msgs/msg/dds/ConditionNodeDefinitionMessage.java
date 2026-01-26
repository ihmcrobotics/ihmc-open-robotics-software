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
       * SHAPE CONTAINS TYPE
       */
public class ConditionNodeDefinitionMessage extends Packet<ConditionNodeDefinitionMessage> implements Settable<ConditionNodeDefinitionMessage>, EpsilonComparable<ConditionNodeDefinitionMessage>
{
   public static final byte COUNTER_TYPE = (byte) 0;
   public static final byte LLM_TYPE = (byte) 1;
   public static final byte PROXIMITY_TYPE = (byte) 2;
   public static final byte SHAPE_CONTAINS = (byte) 3;
   public static final byte ALWAYS_FAIL = (byte) 4;
   public static final byte ALWAYS_SUCCEED = (byte) 5;
   public static final byte XYZ = (byte) 0;
   public static final byte XY = (byte) 1;
   public static final byte Z = (byte) 2;
   public static final byte CONTAINS_FRAME = (byte) 0;
   public static final byte CONTAINS_POINTS = (byte) 1;
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
   public byte distance_type_;
   /**
            * Name of frame A
            */
   public java.lang.StringBuilder frame_name_a_;
   /**
            * Name of frame B
            */
   public java.lang.StringBuilder frame_name_b_;
   /**
            * The minimum distance between the two frames
            */
   public double min_distance_;
   /**
            * The maximum distance between the two frames
            */
   public double max_distance_;
   /**
            * Timeout for waiting for the condition to be satisfied
            */
   public double timeout_;
   /**
            * The type of shape contains condition as defined above
            */
   public byte contains_type_;
   /**
            * Name of the frame the the shape's pose is expressed in
            */
   public java.lang.StringBuilder shape_parent_frame_name_;
   /**
            * Transform that expresses the pose of the shape in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage shape_transform_to_parent_;
   /**
            * Radius of the sphere used for checking containment
            */
   public float sphere_radius_;
   /**
            * Name of frame to check for containment
            */
   public java.lang.StringBuilder frame_name_;
   /**
            * Minimum number of points in the shape
            */
   public long min_points_;

   public ConditionNodeDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.LeafNodeDefinitionMessage();
      system_ = new java.lang.StringBuilder(10000);
      prompt_ = new java.lang.StringBuilder(10000);
      response_matcher_ = new java.lang.StringBuilder(10000);
      frame_name_a_ = new java.lang.StringBuilder(255);
      frame_name_b_ = new java.lang.StringBuilder(255);
      shape_parent_frame_name_ = new java.lang.StringBuilder(255);
      shape_transform_to_parent_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
      frame_name_ = new java.lang.StringBuilder(255);
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

      distance_type_ = other.distance_type_;

      frame_name_a_.setLength(0);
      frame_name_a_.append(other.frame_name_a_);

      frame_name_b_.setLength(0);
      frame_name_b_.append(other.frame_name_b_);

      min_distance_ = other.min_distance_;

      max_distance_ = other.max_distance_;

      timeout_ = other.timeout_;

      contains_type_ = other.contains_type_;

      shape_parent_frame_name_.setLength(0);
      shape_parent_frame_name_.append(other.shape_parent_frame_name_);

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.shape_transform_to_parent_, shape_transform_to_parent_);
      sphere_radius_ = other.sphere_radius_;

      frame_name_.setLength(0);
      frame_name_.append(other.frame_name_);

      min_points_ = other.min_points_;

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
   public void setDistanceType(byte distance_type)
   {
      distance_type_ = distance_type;
   }
   /**
            * The type of distance condition as defined above
            */
   public byte getDistanceType()
   {
      return distance_type_;
   }

   /**
            * Name of frame A
            */
   public void setFrameNameA(java.lang.String frame_name_a)
   {
      frame_name_a_.setLength(0);
      frame_name_a_.append(frame_name_a);
   }

   /**
            * Name of frame A
            */
   public java.lang.String getFrameNameAAsString()
   {
      return getFrameNameA().toString();
   }
   /**
            * Name of frame A
            */
   public java.lang.StringBuilder getFrameNameA()
   {
      return frame_name_a_;
   }

   /**
            * Name of frame B
            */
   public void setFrameNameB(java.lang.String frame_name_b)
   {
      frame_name_b_.setLength(0);
      frame_name_b_.append(frame_name_b);
   }

   /**
            * Name of frame B
            */
   public java.lang.String getFrameNameBAsString()
   {
      return getFrameNameB().toString();
   }
   /**
            * Name of frame B
            */
   public java.lang.StringBuilder getFrameNameB()
   {
      return frame_name_b_;
   }

   /**
            * The minimum distance between the two frames
            */
   public void setMinDistance(double min_distance)
   {
      min_distance_ = min_distance;
   }
   /**
            * The minimum distance between the two frames
            */
   public double getMinDistance()
   {
      return min_distance_;
   }

   /**
            * The maximum distance between the two frames
            */
   public void setMaxDistance(double max_distance)
   {
      max_distance_ = max_distance;
   }
   /**
            * The maximum distance between the two frames
            */
   public double getMaxDistance()
   {
      return max_distance_;
   }

   /**
            * Timeout for waiting for the condition to be satisfied
            */
   public void setTimeout(double timeout)
   {
      timeout_ = timeout;
   }
   /**
            * Timeout for waiting for the condition to be satisfied
            */
   public double getTimeout()
   {
      return timeout_;
   }

   /**
            * The type of shape contains condition as defined above
            */
   public void setContainsType(byte contains_type)
   {
      contains_type_ = contains_type;
   }
   /**
            * The type of shape contains condition as defined above
            */
   public byte getContainsType()
   {
      return contains_type_;
   }

   /**
            * Name of the frame the the shape's pose is expressed in
            */
   public void setShapeParentFrameName(java.lang.String shape_parent_frame_name)
   {
      shape_parent_frame_name_.setLength(0);
      shape_parent_frame_name_.append(shape_parent_frame_name);
   }

   /**
            * Name of the frame the the shape's pose is expressed in
            */
   public java.lang.String getShapeParentFrameNameAsString()
   {
      return getShapeParentFrameName().toString();
   }
   /**
            * Name of the frame the the shape's pose is expressed in
            */
   public java.lang.StringBuilder getShapeParentFrameName()
   {
      return shape_parent_frame_name_;
   }


   /**
            * Transform that expresses the pose of the shape in the parent frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getShapeTransformToParent()
   {
      return shape_transform_to_parent_;
   }

   /**
            * Radius of the sphere used for checking containment
            */
   public void setSphereRadius(float sphere_radius)
   {
      sphere_radius_ = sphere_radius;
   }
   /**
            * Radius of the sphere used for checking containment
            */
   public float getSphereRadius()
   {
      return sphere_radius_;
   }

   /**
            * Name of frame to check for containment
            */
   public void setFrameName(java.lang.String frame_name)
   {
      frame_name_.setLength(0);
      frame_name_.append(frame_name);
   }

   /**
            * Name of frame to check for containment
            */
   public java.lang.String getFrameNameAsString()
   {
      return getFrameName().toString();
   }
   /**
            * Name of frame to check for containment
            */
   public java.lang.StringBuilder getFrameName()
   {
      return frame_name_;
   }

   /**
            * Minimum number of points in the shape
            */
   public void setMinPoints(long min_points)
   {
      min_points_ = min_points;
   }
   /**
            * Minimum number of points in the shape
            */
   public long getMinPoints()
   {
      return min_points_;
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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.distance_type_, other.distance_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.frame_name_a_, other.frame_name_a_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.frame_name_b_, other.frame_name_b_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_distance_, other.min_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_distance_, other.max_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timeout_, other.timeout_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.contains_type_, other.contains_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.shape_parent_frame_name_, other.shape_parent_frame_name_, epsilon)) return false;

      if (!this.shape_transform_to_parent_.epsilonEquals(other.shape_transform_to_parent_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sphere_radius_, other.sphere_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.frame_name_, other.frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_points_, other.min_points_, epsilon)) return false;


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

      if(this.distance_type_ != otherMyClass.distance_type_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.frame_name_a_, otherMyClass.frame_name_a_)) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.frame_name_b_, otherMyClass.frame_name_b_)) return false;

      if(this.min_distance_ != otherMyClass.min_distance_) return false;

      if(this.max_distance_ != otherMyClass.max_distance_) return false;

      if(this.timeout_ != otherMyClass.timeout_) return false;

      if(this.contains_type_ != otherMyClass.contains_type_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.shape_parent_frame_name_, otherMyClass.shape_parent_frame_name_)) return false;

      if (!this.shape_transform_to_parent_.equals(otherMyClass.shape_transform_to_parent_)) return false;
      if(this.sphere_radius_ != otherMyClass.sphere_radius_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.frame_name_, otherMyClass.frame_name_)) return false;

      if(this.min_points_ != otherMyClass.min_points_) return false;


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
      builder.append("distance_type=");
      builder.append(this.distance_type_);      builder.append(", ");
      builder.append("frame_name_a=");
      builder.append(this.frame_name_a_);      builder.append(", ");
      builder.append("frame_name_b=");
      builder.append(this.frame_name_b_);      builder.append(", ");
      builder.append("min_distance=");
      builder.append(this.min_distance_);      builder.append(", ");
      builder.append("max_distance=");
      builder.append(this.max_distance_);      builder.append(", ");
      builder.append("timeout=");
      builder.append(this.timeout_);      builder.append(", ");
      builder.append("contains_type=");
      builder.append(this.contains_type_);      builder.append(", ");
      builder.append("shape_parent_frame_name=");
      builder.append(this.shape_parent_frame_name_);      builder.append(", ");
      builder.append("shape_transform_to_parent=");
      builder.append(this.shape_transform_to_parent_);      builder.append(", ");
      builder.append("sphere_radius=");
      builder.append(this.sphere_radius_);      builder.append(", ");
      builder.append("frame_name=");
      builder.append(this.frame_name_);      builder.append(", ");
      builder.append("min_points=");
      builder.append(this.min_points_);
      builder.append("}");
      return builder.toString();
   }
}
