package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class HandPoseJointAnglesStatusMessage extends Packet<HandPoseJointAnglesStatusMessage> implements Settable<HandPoseJointAnglesStatusMessage>, EpsilonComparable<HandPoseJointAnglesStatusMessage>
{
   /**
            * Used for syncing action sequences
            */
   public behavior_msgs.msg.dds.ActionInformationMessage action_information_;
   /**
            * Specifies the side of the robot that this message refers to.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Joint angles
            */
   public double[] joint_angles_;
   /**
            * Quality of the IK solution
            */
   public double solution_quality_;

   public HandPoseJointAnglesStatusMessage()
   {
      action_information_ = new behavior_msgs.msg.dds.ActionInformationMessage();
      joint_angles_ = new double[7];

   }

   public HandPoseJointAnglesStatusMessage(HandPoseJointAnglesStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(HandPoseJointAnglesStatusMessage other)
   {
      behavior_msgs.msg.dds.ActionInformationMessagePubSubType.staticCopy(other.action_information_, action_information_);
      robot_side_ = other.robot_side_;

      for(int i1 = 0; i1 < joint_angles_.length; ++i1)
      {
            joint_angles_[i1] = other.joint_angles_[i1];

      }

      solution_quality_ = other.solution_quality_;

   }


   /**
            * Used for syncing action sequences
            */
   public behavior_msgs.msg.dds.ActionInformationMessage getActionInformation()
   {
      return action_information_;
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
            * Joint angles
            */
   public double[] getJointAngles()
   {
      return joint_angles_;
   }

   /**
            * Quality of the IK solution
            */
   public void setSolutionQuality(double solution_quality)
   {
      solution_quality_ = solution_quality;
   }
   /**
            * Quality of the IK solution
            */
   public double getSolutionQuality()
   {
      return solution_quality_;
   }


   public static Supplier<HandPoseJointAnglesStatusMessagePubSubType> getPubSubType()
   {
      return HandPoseJointAnglesStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return HandPoseJointAnglesStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(HandPoseJointAnglesStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.action_information_.epsilonEquals(other.action_information_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      for(int i3 = 0; i3 < joint_angles_.length; ++i3)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.joint_angles_[i3], other.joint_angles_[i3], epsilon)) return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.solution_quality_, other.solution_quality_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof HandPoseJointAnglesStatusMessage)) return false;

      HandPoseJointAnglesStatusMessage otherMyClass = (HandPoseJointAnglesStatusMessage) other;

      if (!this.action_information_.equals(otherMyClass.action_information_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      for(int i5 = 0; i5 < joint_angles_.length; ++i5)
      {
                if(this.joint_angles_[i5] != otherMyClass.joint_angles_[i5]) return false;

      }
      if(this.solution_quality_ != otherMyClass.solution_quality_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HandPoseJointAnglesStatusMessage {");
      builder.append("action_information=");
      builder.append(this.action_information_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("joint_angles=");
      builder.append(java.util.Arrays.toString(this.joint_angles_));      builder.append(", ");
      builder.append("solution_quality=");
      builder.append(this.solution_quality_);
      builder.append("}");
      return builder.toString();
   }
}
