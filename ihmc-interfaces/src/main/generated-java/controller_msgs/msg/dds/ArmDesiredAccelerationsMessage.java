package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message gives the user the option to bypass IHMC feedback controllers for the arm joints by sending desired arm joint accelerations.
       * One needs experience in control when activating the bypass as it can result in unexpected behaviors for unreasonable accelerations.
       */
public class ArmDesiredAccelerationsMessage extends Packet<ArmDesiredAccelerationsMessage> implements Settable<ArmDesiredAccelerationsMessage>, EpsilonComparable<ArmDesiredAccelerationsMessage>
{
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;
   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Specifies the side of the robot that will execute the command.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * The desired joint acceleration information.
            */
   public controller_msgs.msg.dds.DesiredAccelerationsMessage desired_accelerations_;

   public ArmDesiredAccelerationsMessage()
   {
      desired_accelerations_ = new controller_msgs.msg.dds.DesiredAccelerationsMessage();
   }

   public ArmDesiredAccelerationsMessage(ArmDesiredAccelerationsMessage other)
   {
      this();
      set(other);
   }

   public void set(ArmDesiredAccelerationsMessage other)
   {
      sequence_id_ = other.sequence_id_;

      robot_side_ = other.robot_side_;

      controller_msgs.msg.dds.DesiredAccelerationsMessagePubSubType.staticCopy(other.desired_accelerations_, desired_accelerations_);
   }

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public void setSequenceId(long sequence_id)
   {
      sequence_id_ = sequence_id;
   }
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long getSequenceId()
   {
      return sequence_id_;
   }

   /**
            * Specifies the side of the robot that will execute the command.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot that will execute the command.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }


   /**
            * The desired joint acceleration information.
            */
   public controller_msgs.msg.dds.DesiredAccelerationsMessage getDesiredAccelerations()
   {
      return desired_accelerations_;
   }


   public static Supplier<ArmDesiredAccelerationsMessagePubSubType> getPubSubType()
   {
      return ArmDesiredAccelerationsMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ArmDesiredAccelerationsMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ArmDesiredAccelerationsMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!this.desired_accelerations_.epsilonEquals(other.desired_accelerations_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ArmDesiredAccelerationsMessage)) return false;

      ArmDesiredAccelerationsMessage otherMyClass = (ArmDesiredAccelerationsMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!this.desired_accelerations_.equals(otherMyClass.desired_accelerations_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ArmDesiredAccelerationsMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("desired_accelerations=");
      builder.append(this.desired_accelerations_);
      builder.append("}");
      return builder.toString();
   }
}
