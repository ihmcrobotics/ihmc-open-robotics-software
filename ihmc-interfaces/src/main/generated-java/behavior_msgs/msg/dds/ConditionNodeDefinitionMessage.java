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
   public byte condition_type_;
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
   public byte shape_contains_type_;
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
   /**
            * Maximum number of points in the shape
            */
   public long max_points_;
   /**
            * Whether to check the color
            */
   public boolean check_color_;
   /**
            * Min hue
            */
   public int hue_min_;
   /**
            * Max hue
            */
   public int hue_max_;
   /**
            * Min saturation
            */
   public int saturation_min_;
   /**
            * Max saturation
            */
   public int saturation_max_;
   /**
            * Min value
            */
   public int value_min_;
   /**
            * Max value
            */
   public int value_max_;

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
      condition_type_ = other.condition_type_;

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

      shape_contains_type_ = other.shape_contains_type_;

      shape_parent_frame_name_.setLength(0);
      shape_parent_frame_name_.append(other.shape_parent_frame_name_);

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.shape_transform_to_parent_, shape_transform_to_parent_);
      sphere_radius_ = other.sphere_radius_;

      frame_name_.setLength(0);
      frame_name_.append(other.frame_name_);

      min_points_ = other.min_points_;

      max_points_ = other.max_points_;

      check_color_ = other.check_color_;

      hue_min_ = other.hue_min_;

      hue_max_ = other.hue_max_;

      saturation_min_ = other.saturation_min_;

      saturation_max_ = other.saturation_max_;

      value_min_ = other.value_min_;

      value_max_ = other.value_max_;

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
   public void setConditionType(byte condition_type)
   {
      condition_type_ = condition_type;
   }
   /**
            * The type of condition as defined above
            */
   public byte getConditionType()
   {
      return condition_type_;
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
   public void setShapeContainsType(byte shape_contains_type)
   {
      shape_contains_type_ = shape_contains_type;
   }
   /**
            * The type of shape contains condition as defined above
            */
   public byte getShapeContainsType()
   {
      return shape_contains_type_;
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

   /**
            * Maximum number of points in the shape
            */
   public void setMaxPoints(long max_points)
   {
      max_points_ = max_points;
   }
   /**
            * Maximum number of points in the shape
            */
   public long getMaxPoints()
   {
      return max_points_;
   }

   /**
            * Whether to check the color
            */
   public void setCheckColor(boolean check_color)
   {
      check_color_ = check_color;
   }
   /**
            * Whether to check the color
            */
   public boolean getCheckColor()
   {
      return check_color_;
   }

   /**
            * Min hue
            */
   public void setHueMin(int hue_min)
   {
      hue_min_ = hue_min;
   }
   /**
            * Min hue
            */
   public int getHueMin()
   {
      return hue_min_;
   }

   /**
            * Max hue
            */
   public void setHueMax(int hue_max)
   {
      hue_max_ = hue_max;
   }
   /**
            * Max hue
            */
   public int getHueMax()
   {
      return hue_max_;
   }

   /**
            * Min saturation
            */
   public void setSaturationMin(int saturation_min)
   {
      saturation_min_ = saturation_min;
   }
   /**
            * Min saturation
            */
   public int getSaturationMin()
   {
      return saturation_min_;
   }

   /**
            * Max saturation
            */
   public void setSaturationMax(int saturation_max)
   {
      saturation_max_ = saturation_max;
   }
   /**
            * Max saturation
            */
   public int getSaturationMax()
   {
      return saturation_max_;
   }

   /**
            * Min value
            */
   public void setValueMin(int value_min)
   {
      value_min_ = value_min;
   }
   /**
            * Min value
            */
   public int getValueMin()
   {
      return value_min_;
   }

   /**
            * Max value
            */
   public void setValueMax(int value_max)
   {
      value_max_ = value_max;
   }
   /**
            * Max value
            */
   public int getValueMax()
   {
      return value_max_;
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
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.condition_type_, other.condition_type_, epsilon)) return false;

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

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.shape_contains_type_, other.shape_contains_type_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.shape_parent_frame_name_, other.shape_parent_frame_name_, epsilon)) return false;

      if (!this.shape_transform_to_parent_.epsilonEquals(other.shape_transform_to_parent_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sphere_radius_, other.sphere_radius_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.frame_name_, other.frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.min_points_, other.min_points_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_points_, other.max_points_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.check_color_, other.check_color_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.hue_min_, other.hue_min_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.hue_max_, other.hue_max_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.saturation_min_, other.saturation_min_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.saturation_max_, other.saturation_max_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.value_min_, other.value_min_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.value_max_, other.value_max_, epsilon)) return false;


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
      if(this.condition_type_ != otherMyClass.condition_type_) return false;

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

      if(this.shape_contains_type_ != otherMyClass.shape_contains_type_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.shape_parent_frame_name_, otherMyClass.shape_parent_frame_name_)) return false;

      if (!this.shape_transform_to_parent_.equals(otherMyClass.shape_transform_to_parent_)) return false;
      if(this.sphere_radius_ != otherMyClass.sphere_radius_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.frame_name_, otherMyClass.frame_name_)) return false;

      if(this.min_points_ != otherMyClass.min_points_) return false;

      if(this.max_points_ != otherMyClass.max_points_) return false;

      if(this.check_color_ != otherMyClass.check_color_) return false;

      if(this.hue_min_ != otherMyClass.hue_min_) return false;

      if(this.hue_max_ != otherMyClass.hue_max_) return false;

      if(this.saturation_min_ != otherMyClass.saturation_min_) return false;

      if(this.saturation_max_ != otherMyClass.saturation_max_) return false;

      if(this.value_min_ != otherMyClass.value_min_) return false;

      if(this.value_max_ != otherMyClass.value_max_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ConditionNodeDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("condition_type=");
      builder.append(this.condition_type_);      builder.append(", ");
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
      builder.append("shape_contains_type=");
      builder.append(this.shape_contains_type_);      builder.append(", ");
      builder.append("shape_parent_frame_name=");
      builder.append(this.shape_parent_frame_name_);      builder.append(", ");
      builder.append("shape_transform_to_parent=");
      builder.append(this.shape_transform_to_parent_);      builder.append(", ");
      builder.append("sphere_radius=");
      builder.append(this.sphere_radius_);      builder.append(", ");
      builder.append("frame_name=");
      builder.append(this.frame_name_);      builder.append(", ");
      builder.append("min_points=");
      builder.append(this.min_points_);      builder.append(", ");
      builder.append("max_points=");
      builder.append(this.max_points_);      builder.append(", ");
      builder.append("check_color=");
      builder.append(this.check_color_);      builder.append(", ");
      builder.append("hue_min=");
      builder.append(this.hue_min_);      builder.append(", ");
      builder.append("hue_max=");
      builder.append(this.hue_max_);      builder.append(", ");
      builder.append("saturation_min=");
      builder.append(this.saturation_min_);      builder.append(", ");
      builder.append("saturation_max=");
      builder.append(this.saturation_max_);      builder.append(", ");
      builder.append("value_min=");
      builder.append(this.value_min_);      builder.append(", ");
      builder.append("value_max=");
      builder.append(this.value_max_);
      builder.append("}");
      return builder.toString();
   }
}
