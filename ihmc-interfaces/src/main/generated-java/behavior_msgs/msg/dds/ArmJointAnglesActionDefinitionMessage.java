package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ArmJointAnglesActionDefinitionMessage extends Packet<ArmJointAnglesActionDefinitionMessage> implements Settable<ArmJointAnglesActionDefinitionMessage>, EpsilonComparable<ArmJointAnglesActionDefinitionMessage>
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
            * Preset arm configuration
            */
   public int preset_;
   /**
            * Joint angles
            */
   public double[] joint_angles_;
   /**
            * Duration of the trajectory
            */
   public double trajectory_duration_;

   public ArmJointAnglesActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      joint_angles_ = new double[7];

   }

   public ArmJointAnglesActionDefinitionMessage(ArmJointAnglesActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(ArmJointAnglesActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      robot_side_ = other.robot_side_;

      preset_ = other.preset_;

      for(int i1 = 0; i1 < joint_angles_.length; ++i1)
      {
            joint_angles_[i1] = other.joint_angles_[i1];

      }

      trajectory_duration_ = other.trajectory_duration_;

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
            * Preset arm configuration
            */
   public void setPreset(int preset)
   {
      preset_ = preset;
   }
   /**
            * Preset arm configuration
            */
   public int getPreset()
   {
      return preset_;
   }


   /**
            * Joint angles
            */
   public double[] getJointAngles()
   {
      return joint_angles_;
   }

   /**
            * Duration of the trajectory
            */
   public void setTrajectoryDuration(double trajectory_duration)
   {
      trajectory_duration_ = trajectory_duration;
   }
   /**
            * Duration of the trajectory
            */
   public double getTrajectoryDuration()
   {
      return trajectory_duration_;
   }


   public static Supplier<ArmJointAnglesActionDefinitionMessagePubSubType> getPubSubType()
   {
      return ArmJointAnglesActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ArmJointAnglesActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ArmJointAnglesActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.preset_, other.preset_, epsilon)) return false;

      for(int i3 = 0; i3 < joint_angles_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_angles_[i3], other.joint_angles_[i3], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.trajectory_duration_, other.trajectory_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ArmJointAnglesActionDefinitionMessage)) return false;

      ArmJointAnglesActionDefinitionMessage otherMyClass = (ArmJointAnglesActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if(this.preset_ != otherMyClass.preset_) return false;

      for(int i5 = 0; i5 < joint_angles_.length; ++i5)
      {
                if(this.joint_angles_[i5] != otherMyClass.joint_angles_[i5]) return false;

      }
      if(this.trajectory_duration_ != otherMyClass.trajectory_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ArmJointAnglesActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("preset=");
      builder.append(this.preset_);      builder.append(", ");
      builder.append("joint_angles=");
      builder.append(java.util.Arrays.toString(this.joint_angles_));      builder.append(", ");
      builder.append("trajectory_duration=");
      builder.append(this.trajectory_duration_);
      builder.append("}");
      return builder.toString();
   }
}
