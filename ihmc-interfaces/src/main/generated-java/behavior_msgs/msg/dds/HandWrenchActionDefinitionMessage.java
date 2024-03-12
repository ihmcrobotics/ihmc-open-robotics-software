package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class HandWrenchActionDefinitionMessage extends Packet<HandWrenchActionDefinitionMessage> implements Settable<HandWrenchActionDefinitionMessage>, EpsilonComparable<HandWrenchActionDefinitionMessage>
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
   public java.lang.StringBuilder parent_frame_name_;
   /**
            * The trajectory duration
            */
   public double trajectory_duration_;
   /**
            * Amount of force to use along X
            */
   public double force_x_;
   /**
            * Amount of force to use along Y
            */
   public double force_y_;
   /**
            * Amount of force to use along Z
            */
   public double force_z_;
   /**
            * Amount of torque to use along X
            */
   public double torque_x_;
   /**
            * Amount of torque to use along Y
            */
   public double torque_y_;
   /**
            * Amount of torque to use along Z
            */
   public double torque_z_;

   public HandWrenchActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      parent_frame_name_ = new java.lang.StringBuilder(255);
   }

   public HandWrenchActionDefinitionMessage(HandWrenchActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(HandWrenchActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      robot_side_ = other.robot_side_;

      parent_frame_name_.setLength(0);
      parent_frame_name_.append(other.parent_frame_name_);

      trajectory_duration_ = other.trajectory_duration_;

      force_x_ = other.force_x_;

      force_y_ = other.force_y_;

      force_z_ = other.force_z_;

      torque_x_ = other.torque_x_;

      torque_y_ = other.torque_y_;

      torque_z_ = other.torque_z_;

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
   public void setParentFrameName(java.lang.String parent_frame_name)
   {
      parent_frame_name_.setLength(0);
      parent_frame_name_.append(parent_frame_name);
   }

   /**
            * Name of the frame the this action is expressed in
            */
   public java.lang.String getParentFrameNameAsString()
   {
      return getParentFrameName().toString();
   }
   /**
            * Name of the frame the this action is expressed in
            */
   public java.lang.StringBuilder getParentFrameName()
   {
      return parent_frame_name_;
   }

   /**
            * The trajectory duration
            */
   public void setTrajectoryDuration(double trajectory_duration)
   {
      trajectory_duration_ = trajectory_duration;
   }
   /**
            * The trajectory duration
            */
   public double getTrajectoryDuration()
   {
      return trajectory_duration_;
   }

   /**
            * Amount of force to use along X
            */
   public void setForceX(double force_x)
   {
      force_x_ = force_x;
   }
   /**
            * Amount of force to use along X
            */
   public double getForceX()
   {
      return force_x_;
   }

   /**
            * Amount of force to use along Y
            */
   public void setForceY(double force_y)
   {
      force_y_ = force_y;
   }
   /**
            * Amount of force to use along Y
            */
   public double getForceY()
   {
      return force_y_;
   }

   /**
            * Amount of force to use along Z
            */
   public void setForceZ(double force_z)
   {
      force_z_ = force_z;
   }
   /**
            * Amount of force to use along Z
            */
   public double getForceZ()
   {
      return force_z_;
   }

   /**
            * Amount of torque to use along X
            */
   public void setTorqueX(double torque_x)
   {
      torque_x_ = torque_x;
   }
   /**
            * Amount of torque to use along X
            */
   public double getTorqueX()
   {
      return torque_x_;
   }

   /**
            * Amount of torque to use along Y
            */
   public void setTorqueY(double torque_y)
   {
      torque_y_ = torque_y;
   }
   /**
            * Amount of torque to use along Y
            */
   public double getTorqueY()
   {
      return torque_y_;
   }

   /**
            * Amount of torque to use along Z
            */
   public void setTorqueZ(double torque_z)
   {
      torque_z_ = torque_z;
   }
   /**
            * Amount of torque to use along Z
            */
   public double getTorqueZ()
   {
      return torque_z_;
   }


   public static Supplier<HandWrenchActionDefinitionMessagePubSubType> getPubSubType()
   {
      return HandWrenchActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandWrenchActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandWrenchActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.parent_frame_name_, other.parent_frame_name_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_duration_, other.trajectory_duration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.force_x_, other.force_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.force_y_, other.force_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.force_z_, other.force_z_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.torque_x_, other.torque_x_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.torque_y_, other.torque_y_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.torque_z_, other.torque_z_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandWrenchActionDefinitionMessage)) return false;

      HandWrenchActionDefinitionMessage otherMyClass = (HandWrenchActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.parent_frame_name_, otherMyClass.parent_frame_name_)) return false;

      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;

      if(this.force_x_ != otherMyClass.force_x_) return false;

      if(this.force_y_ != otherMyClass.force_y_) return false;

      if(this.force_z_ != otherMyClass.force_z_) return false;

      if(this.torque_x_ != otherMyClass.torque_x_) return false;

      if(this.torque_y_ != otherMyClass.torque_y_) return false;

      if(this.torque_z_ != otherMyClass.torque_z_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandWrenchActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("parent_frame_name=");
      builder.append(this.parent_frame_name_);      builder.append(", ");
      builder.append("trajectory_duration=");
      builder.append(this.trajectory_duration_);      builder.append(", ");
      builder.append("force_x=");
      builder.append(this.force_x_);      builder.append(", ");
      builder.append("force_y=");
      builder.append(this.force_y_);      builder.append(", ");
      builder.append("force_z=");
      builder.append(this.force_z_);      builder.append(", ");
      builder.append("torque_x=");
      builder.append(this.torque_x_);      builder.append(", ");
      builder.append("torque_y=");
      builder.append(this.torque_y_);      builder.append(", ");
      builder.append("torque_z=");
      builder.append(this.torque_z_);
      builder.append("}");
      return builder.toString();
   }
}
