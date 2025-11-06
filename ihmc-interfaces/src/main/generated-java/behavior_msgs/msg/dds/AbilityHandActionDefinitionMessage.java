package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class AbilityHandActionDefinitionMessage extends Packet<AbilityHandActionDefinitionMessage> implements Settable<AbilityHandActionDefinitionMessage>, EpsilonComparable<AbilityHandActionDefinitionMessage>
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
            * AbilityHandManager.ControlMode ordinal
            */
   public int control_mode_;
   /**
            * Grip to execute, if grip control mode
            */
   public int grip_;
   /**
            * Goal positions for the joints
            */
   public float[] goal_positions_;
   /**
            * Goal positions for the joints
            */
   public float[] goal_velocities_;

   public AbilityHandActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      goal_positions_ = new float[6];

      goal_velocities_ = new float[6];

   }

   public AbilityHandActionDefinitionMessage(AbilityHandActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(AbilityHandActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      robot_side_ = other.robot_side_;

      control_mode_ = other.control_mode_;

      grip_ = other.grip_;

      for(int i1 = 0; i1 < goal_positions_.length; ++i1)
      {
            goal_positions_[i1] = other.goal_positions_[i1];

      }

      for(int i3 = 0; i3 < goal_velocities_.length; ++i3)
      {
            goal_velocities_[i3] = other.goal_velocities_[i3];

      }

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
            * AbilityHandManager.ControlMode ordinal
            */
   public void setControlMode(int control_mode)
   {
      control_mode_ = control_mode;
   }
   /**
            * AbilityHandManager.ControlMode ordinal
            */
   public int getControlMode()
   {
      return control_mode_;
   }

   /**
            * Grip to execute, if grip control mode
            */
   public void setGrip(int grip)
   {
      grip_ = grip;
   }
   /**
            * Grip to execute, if grip control mode
            */
   public int getGrip()
   {
      return grip_;
   }


   /**
            * Goal positions for the joints
            */
   public float[] getGoalPositions()
   {
      return goal_positions_;
   }


   /**
            * Goal positions for the joints
            */
   public float[] getGoalVelocities()
   {
      return goal_velocities_;
   }


   public static Supplier<AbilityHandActionDefinitionMessagePubSubType> getPubSubType()
   {
      return AbilityHandActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AbilityHandActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AbilityHandActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.control_mode_, other.control_mode_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.grip_, other.grip_, epsilon)) return false;

      for(int i5 = 0; i5 < goal_positions_.length; ++i5)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_positions_[i5], other.goal_positions_[i5], epsilon)) return false;
      }

      for(int i7 = 0; i7 < goal_velocities_.length; ++i7)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.goal_velocities_[i7], other.goal_velocities_[i7], epsilon)) return false;
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AbilityHandActionDefinitionMessage)) return false;

      AbilityHandActionDefinitionMessage otherMyClass = (AbilityHandActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.control_mode_ != otherMyClass.control_mode_) return false;

      if(this.grip_ != otherMyClass.grip_) return false;

      for(int i9 = 0; i9 < goal_positions_.length; ++i9)
      {
                if(this.goal_positions_[i9] != otherMyClass.goal_positions_[i9]) return false;

      }
      for(int i11 = 0; i11 < goal_velocities_.length; ++i11)
      {
                if(this.goal_velocities_[i11] != otherMyClass.goal_velocities_[i11]) return false;

      }

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AbilityHandActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("control_mode=");
      builder.append(this.control_mode_);      builder.append(", ");
      builder.append("grip=");
      builder.append(this.grip_);      builder.append(", ");
      builder.append("goal_positions=");
      builder.append(java.util.Arrays.toString(this.goal_positions_));      builder.append(", ");
      builder.append("goal_velocities=");
      builder.append(java.util.Arrays.toString(this.goal_velocities_));
      builder.append("}");
      return builder.toString();
   }
}
