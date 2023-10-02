package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A single footstep
       */
public class FootstepActionDefinitionMessage extends Packet<FootstepActionDefinitionMessage> implements Settable<FootstepActionDefinitionMessage>, EpsilonComparable<FootstepActionDefinitionMessage>
{
   /**
            * Side, left or right foot
            */
   public byte robot_side_ = (byte) 255;
   /**
            * The footstep's sole pose
            */
   public us.ihmc.euclid.geometry.Pose3D sole_pose_;

   public FootstepActionDefinitionMessage()
   {
      sole_pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public FootstepActionDefinitionMessage(FootstepActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepActionDefinitionMessage other)
   {
      robot_side_ = other.robot_side_;

      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.sole_pose_, sole_pose_);
   }

   /**
            * Side, left or right foot
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Side, left or right foot
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }


   /**
            * The footstep's sole pose
            */
   public us.ihmc.euclid.geometry.Pose3D getSolePose()
   {
      return sole_pose_;
   }


   public static Supplier<FootstepActionDefinitionMessagePubSubType> getPubSubType()
   {
      return FootstepActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!this.sole_pose_.epsilonEquals(other.sole_pose_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepActionDefinitionMessage)) return false;

      FootstepActionDefinitionMessage otherMyClass = (FootstepActionDefinitionMessage) other;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!this.sole_pose_.equals(otherMyClass.sole_pose_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepActionDefinitionMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("sole_pose=");
      builder.append(this.sole_pose_);
      builder.append("}");
      return builder.toString();
   }
}
