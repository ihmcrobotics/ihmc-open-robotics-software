package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ScrewPrimitiveActionDefinitionMessage extends Packet<ScrewPrimitiveActionDefinitionMessage> implements Settable<ScrewPrimitiveActionDefinitionMessage>, EpsilonComparable<ScrewPrimitiveActionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * Specifies the side of the robot that this message refers to.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Name of the frame the this action is expressed in
            */
   public java.lang.StringBuilder object_frame_name_;
   /**
            * The pose of the screw axis in the object frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage screw_axis_pose_;
   /**
            * The magnitude of the translation component
            */
   public double translation_;
   /**
            * The magnitude of the rotation component
            */
   public double rotation_;
   /**
            * The max linear velocity
            */
   public double max_linear_velocity_;
   /**
            * The max angular velocity
            */
   public double max_angular_velocity_;
   /**
            * Whether the trajectory is controlled in jointspace (true) or hybrid jointspace and taskspace (false)
            */
   public boolean jointspace_only_;
   public double linear_position_weight_;
   public double angular_position_weight_;

   public ScrewPrimitiveActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      object_frame_name_ = new java.lang.StringBuilder(255);
      screw_axis_pose_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public ScrewPrimitiveActionDefinitionMessage(ScrewPrimitiveActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(ScrewPrimitiveActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      robot_side_ = other.robot_side_;

      object_frame_name_.setLength(0);
      object_frame_name_.append(other.object_frame_name_);

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.screw_axis_pose_, screw_axis_pose_);
      translation_ = other.translation_;

      rotation_ = other.rotation_;

      max_linear_velocity_ = other.max_linear_velocity_;

      max_angular_velocity_ = other.max_angular_velocity_;

      jointspace_only_ = other.jointspace_only_;

      linear_position_weight_ = other.linear_position_weight_;

      angular_position_weight_ = other.angular_position_weight_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Specifies the side of the robot that this message refers to.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot that this message refers to.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
            * Name of the frame the this action is expressed in
            */
   public void setObjectFrameName(java.lang.String object_frame_name)
   {
      object_frame_name_.setLength(0);
      object_frame_name_.append(object_frame_name);
   }

   /**
            * Name of the frame the this action is expressed in
            */
   public java.lang.String getObjectFrameNameAsString()
   {
      return getObjectFrameName().toString();
   }
   /**
            * Name of the frame the this action is expressed in
            */
   public java.lang.StringBuilder getObjectFrameName()
   {
      return object_frame_name_;
   }


   /**
            * The pose of the screw axis in the object frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getScrewAxisPose()
   {
      return screw_axis_pose_;
   }

   /**
            * The magnitude of the translation component
            */
   public void setTranslation(double translation)
   {
      translation_ = translation;
   }
   /**
            * The magnitude of the translation component
            */
   public double getTranslation()
   {
      return translation_;
   }

   /**
            * The magnitude of the rotation component
            */
   public void setRotation(double rotation)
   {
      rotation_ = rotation;
   }
   /**
            * The magnitude of the rotation component
            */
   public double getRotation()
   {
      return rotation_;
   }

   /**
            * The max linear velocity
            */
   public void setMaxLinearVelocity(double max_linear_velocity)
   {
      max_linear_velocity_ = max_linear_velocity;
   }
   /**
            * The max linear velocity
            */
   public double getMaxLinearVelocity()
   {
      return max_linear_velocity_;
   }

   /**
            * The max angular velocity
            */
   public void setMaxAngularVelocity(double max_angular_velocity)
   {
      max_angular_velocity_ = max_angular_velocity;
   }
   /**
            * The max angular velocity
            */
   public double getMaxAngularVelocity()
   {
      return max_angular_velocity_;
   }

   /**
            * Whether the trajectory is controlled in jointspace (true) or hybrid jointspace and taskspace (false)
            */
   public void setJointspaceOnly(boolean jointspace_only)
   {
      jointspace_only_ = jointspace_only;
   }
   /**
            * Whether the trajectory is controlled in jointspace (true) or hybrid jointspace and taskspace (false)
            */
   public boolean getJointspaceOnly()
   {
      return jointspace_only_;
   }

   public void setLinearPositionWeight(double linear_position_weight)
   {
      linear_position_weight_ = linear_position_weight;
   }
   public double getLinearPositionWeight()
   {
      return linear_position_weight_;
   }

   public void setAngularPositionWeight(double angular_position_weight)
   {
      angular_position_weight_ = angular_position_weight;
   }
   public double getAngularPositionWeight()
   {
      return angular_position_weight_;
   }


   public static Supplier<ScrewPrimitiveActionDefinitionMessagePubSubType> getPubSubType()
   {
      return ScrewPrimitiveActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ScrewPrimitiveActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ScrewPrimitiveActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_frame_name_, other.object_frame_name_, epsilon)) return false;

      if (!this.screw_axis_pose_.epsilonEquals(other.screw_axis_pose_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.translation_, other.translation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rotation_, other.rotation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_linear_velocity_, other.max_linear_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.max_angular_velocity_, other.max_angular_velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.jointspace_only_, other.jointspace_only_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.linear_position_weight_, other.linear_position_weight_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.angular_position_weight_, other.angular_position_weight_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ScrewPrimitiveActionDefinitionMessage)) return false;

      ScrewPrimitiveActionDefinitionMessage otherMyClass = (ScrewPrimitiveActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.object_frame_name_, otherMyClass.object_frame_name_)) return false;

      if (!this.screw_axis_pose_.equals(otherMyClass.screw_axis_pose_)) return false;
      if(this.translation_ != otherMyClass.translation_) return false;

      if(this.rotation_ != otherMyClass.rotation_) return false;

      if(this.max_linear_velocity_ != otherMyClass.max_linear_velocity_) return false;

      if(this.max_angular_velocity_ != otherMyClass.max_angular_velocity_) return false;

      if(this.jointspace_only_ != otherMyClass.jointspace_only_) return false;

      if(this.linear_position_weight_ != otherMyClass.linear_position_weight_) return false;

      if(this.angular_position_weight_ != otherMyClass.angular_position_weight_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ScrewPrimitiveActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("object_frame_name=");
      builder.append(this.object_frame_name_);      builder.append(", ");
      builder.append("screw_axis_pose=");
      builder.append(this.screw_axis_pose_);      builder.append(", ");
      builder.append("translation=");
      builder.append(this.translation_);      builder.append(", ");
      builder.append("rotation=");
      builder.append(this.rotation_);      builder.append(", ");
      builder.append("max_linear_velocity=");
      builder.append(this.max_linear_velocity_);      builder.append(", ");
      builder.append("max_angular_velocity=");
      builder.append(this.max_angular_velocity_);      builder.append(", ");
      builder.append("jointspace_only=");
      builder.append(this.jointspace_only_);      builder.append(", ");
      builder.append("linear_position_weight=");
      builder.append(this.linear_position_weight_);      builder.append(", ");
      builder.append("angular_position_weight=");
      builder.append(this.angular_position_weight_);
      builder.append("}");
      return builder.toString();
   }
}
