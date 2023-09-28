package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * A single footstep
       */
public class FootstepActionDescriptionMessage extends Packet<FootstepActionDescriptionMessage> implements Settable<FootstepActionDescriptionMessage>, EpsilonComparable<FootstepActionDescriptionMessage>
{
   /**
            * Side, left or right foot
            */
   public byte robot_side_ = (byte) 255;
   /**
            * The footstep's sole pose
            */
   public us.ihmc.euclid.geometry.Pose3D sole_pose_;

   public FootstepActionDescriptionMessage()
   {
      sole_pose_ = new us.ihmc.euclid.geometry.Pose3D();
   }

   public FootstepActionDescriptionMessage(FootstepActionDescriptionMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepActionDescriptionMessage other)
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


   public static Supplier<FootstepActionDescriptionMessagePubSubType> getPubSubType()
   {
      return FootstepActionDescriptionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepActionDescriptionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepActionDescriptionMessage other, double epsilon)
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
      if(!(other instanceof FootstepActionDescriptionMessage)) return false;

      FootstepActionDescriptionMessage otherMyClass = (FootstepActionDescriptionMessage) other;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!this.sole_pose_.equals(otherMyClass.sole_pose_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepActionDescriptionMessage {");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("sole_pose=");
      builder.append(this.sole_pose_);
      builder.append("}");
      return builder.toString();
   }
}
