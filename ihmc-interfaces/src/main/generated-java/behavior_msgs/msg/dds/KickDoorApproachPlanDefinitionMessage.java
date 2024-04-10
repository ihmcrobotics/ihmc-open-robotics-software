package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class KickDoorApproachPlanDefinitionMessage extends Packet<KickDoorApproachPlanDefinitionMessage> implements Settable<KickDoorApproachPlanDefinitionMessage>, EpsilonComparable<KickDoorApproachPlanDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public java.lang.StringBuilder parent_frame_name_;
   /**
            * Swing duration
            */
   public double swing_duration_;
   /**
            * Transfer duration
            */
   public double transfer_duration_;
   /**
            * OVERRIDE (0) or QUEUE (1)
            */
   public int execution_mode_;
   /**
            * Specifies the side of the robot that will execute the kick.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * The impulse with which the kick should be executed.
            */
   public double kick_height_;
   /**
            * The impulse with which the kick should be executed.
            */
   public double kick_impulse_;
   /**
            * The target distance from the robot to where the kick should be aimed.
            */
   public double kick_target_distance_;
   /**
            * The distance towards the inside of the door from where the kick foot should be aligned.
            */
   public double horizontal_distance_from_handle_;
   /**
            * The stance foot width.
            */
   public double stance_foot_width_;
   /**
            * Weight distribution before the kick. 1.0 means all weight on the kicking foot. Default is 0.5.
            */
   public double prekick_weight_distribution_;

   public KickDoorApproachPlanDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      parent_frame_name_ = new java.lang.StringBuilder(255);
   }

   public KickDoorApproachPlanDefinitionMessage(KickDoorApproachPlanDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(KickDoorApproachPlanDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      parent_frame_name_.setLength(0);
      parent_frame_name_.append(other.parent_frame_name_);

      swing_duration_ = other.swing_duration_;

      transfer_duration_ = other.transfer_duration_;

      execution_mode_ = other.execution_mode_;

      robot_side_ = other.robot_side_;

      kick_height_ = other.kick_height_;

      kick_impulse_ = other.kick_impulse_;

      kick_target_distance_ = other.kick_target_distance_;

      horizontal_distance_from_handle_ = other.horizontal_distance_from_handle_;

      stance_foot_width_ = other.stance_foot_width_;

      prekick_weight_distribution_ = other.prekick_weight_distribution_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public void setParentFrameName(java.lang.String parent_frame_name)
   {
      parent_frame_name_.setLength(0);
      parent_frame_name_.append(parent_frame_name);
   }

   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public java.lang.String getParentFrameNameAsString()
   {
      return getParentFrameName().toString();
   }
   /**
            * Name of the parent frame the footsteps are expressed in
            */
   public java.lang.StringBuilder getParentFrameName()
   {
      return parent_frame_name_;
   }

   /**
            * Swing duration
            */
   public void setSwingDuration(double swing_duration)
   {
      swing_duration_ = swing_duration;
   }
   /**
            * Swing duration
            */
   public double getSwingDuration()
   {
      return swing_duration_;
   }

   /**
            * Transfer duration
            */
   public void setTransferDuration(double transfer_duration)
   {
      transfer_duration_ = transfer_duration;
   }
   /**
            * Transfer duration
            */
   public double getTransferDuration()
   {
      return transfer_duration_;
   }

   /**
            * OVERRIDE (0) or QUEUE (1)
            */
   public void setExecutionMode(int execution_mode)
   {
      execution_mode_ = execution_mode;
   }
   /**
            * OVERRIDE (0) or QUEUE (1)
            */
   public int getExecutionMode()
   {
      return execution_mode_;
   }

   /**
            * Specifies the side of the robot that will execute the kick.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot that will execute the kick.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
            * The impulse with which the kick should be executed.
            */
   public void setKickHeight(double kick_height)
   {
      kick_height_ = kick_height;
   }
   /**
            * The impulse with which the kick should be executed.
            */
   public double getKickHeight()
   {
      return kick_height_;
   }

   /**
            * The impulse with which the kick should be executed.
            */
   public void setKickImpulse(double kick_impulse)
   {
      kick_impulse_ = kick_impulse;
   }
   /**
            * The impulse with which the kick should be executed.
            */
   public double getKickImpulse()
   {
      return kick_impulse_;
   }

   /**
            * The target distance from the robot to where the kick should be aimed.
            */
   public void setKickTargetDistance(double kick_target_distance)
   {
      kick_target_distance_ = kick_target_distance;
   }
   /**
            * The target distance from the robot to where the kick should be aimed.
            */
   public double getKickTargetDistance()
   {
      return kick_target_distance_;
   }

   /**
            * The distance towards the inside of the door from where the kick foot should be aligned.
            */
   public void setHorizontalDistanceFromHandle(double horizontal_distance_from_handle)
   {
      horizontal_distance_from_handle_ = horizontal_distance_from_handle;
   }
   /**
            * The distance towards the inside of the door from where the kick foot should be aligned.
            */
   public double getHorizontalDistanceFromHandle()
   {
      return horizontal_distance_from_handle_;
   }

   /**
            * The stance foot width.
            */
   public void setStanceFootWidth(double stance_foot_width)
   {
      stance_foot_width_ = stance_foot_width;
   }
   /**
            * The stance foot width.
            */
   public double getStanceFootWidth()
   {
      return stance_foot_width_;
   }

   /**
            * Weight distribution before the kick. 1.0 means all weight on the kicking foot. Default is 0.5.
            */
   public void setPrekickWeightDistribution(double prekick_weight_distribution)
   {
      prekick_weight_distribution_ = prekick_weight_distribution;
   }
   /**
            * Weight distribution before the kick. 1.0 means all weight on the kicking foot. Default is 0.5.
            */
   public double getPrekickWeightDistribution()
   {
      return prekick_weight_distribution_;
   }


   public static Supplier<KickDoorApproachPlanDefinitionMessagePubSubType> getPubSubType()
   {
      return KickDoorApproachPlanDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return KickDoorApproachPlanDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(KickDoorApproachPlanDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.parent_frame_name_, other.parent_frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_, other.swing_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_duration_, other.transfer_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.execution_mode_, other.execution_mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kick_height_, other.kick_height_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kick_impulse_, other.kick_impulse_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kick_target_distance_, other.kick_target_distance_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.horizontal_distance_from_handle_, other.horizontal_distance_from_handle_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.stance_foot_width_, other.stance_foot_width_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.prekick_weight_distribution_, other.prekick_weight_distribution_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof KickDoorApproachPlanDefinitionMessage)) return false;

      KickDoorApproachPlanDefinitionMessage otherMyClass = (KickDoorApproachPlanDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if (!us.ihmc.idl.IDLTools.equals(this.parent_frame_name_, otherMyClass.parent_frame_name_)) return false;

      if(this.swing_duration_ != otherMyClass.swing_duration_) return false;

      if(this.transfer_duration_ != otherMyClass.transfer_duration_) return false;

      if(this.execution_mode_ != otherMyClass.execution_mode_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.kick_height_ != otherMyClass.kick_height_) return false;

      if(this.kick_impulse_ != otherMyClass.kick_impulse_) return false;

      if(this.kick_target_distance_ != otherMyClass.kick_target_distance_) return false;

      if(this.horizontal_distance_from_handle_ != otherMyClass.horizontal_distance_from_handle_) return false;

      if(this.stance_foot_width_ != otherMyClass.stance_foot_width_) return false;

      if(this.prekick_weight_distribution_ != otherMyClass.prekick_weight_distribution_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("KickDoorApproachPlanDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("parent_frame_name=");
      builder.append(this.parent_frame_name_);      builder.append(", ");
      builder.append("swing_duration=");
      builder.append(this.swing_duration_);      builder.append(", ");
      builder.append("transfer_duration=");
      builder.append(this.transfer_duration_);      builder.append(", ");
      builder.append("execution_mode=");
      builder.append(this.execution_mode_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("kick_height=");
      builder.append(this.kick_height_);      builder.append(", ");
      builder.append("kick_impulse=");
      builder.append(this.kick_impulse_);      builder.append(", ");
      builder.append("kick_target_distance=");
      builder.append(this.kick_target_distance_);      builder.append(", ");
      builder.append("horizontal_distance_from_handle=");
      builder.append(this.horizontal_distance_from_handle_);      builder.append(", ");
      builder.append("stance_foot_width=");
      builder.append(this.stance_foot_width_);      builder.append(", ");
      builder.append("prekick_weight_distribution=");
      builder.append(this.prekick_weight_distribution_);
      builder.append("}");
      return builder.toString();
   }
}
