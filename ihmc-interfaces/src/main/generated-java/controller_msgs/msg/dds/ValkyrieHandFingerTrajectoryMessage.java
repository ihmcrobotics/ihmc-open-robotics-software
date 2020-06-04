package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ValkyrieHandFingerTrajectoryMessage extends Packet<ValkyrieHandFingerTrajectoryMessage> implements Settable<ValkyrieHandFingerTrajectoryMessage>, EpsilonComparable<ValkyrieHandFingerTrajectoryMessage>
{

   /**
          * This message commands the finger controller to move robot valkyrie finger.
          */
   public static final byte ROBOT_SIDE_LEFT = (byte) 0;

   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;

   public static final byte THUMB_MOTOR_ROLL = (byte) 0;

   public static final byte THUMB_MOTOR_PITCH1 = (byte) 1;

   public static final byte THUMB_MOTOR_PITCH2 = (byte) 2;

   public static final byte INDEX_FINGER_MOTOR_PITCH1 = (byte) 3;

   public static final byte MIDDLE_FINGER_MOTOR_PITCH1 = (byte) 4;

   public static final byte PINKY_MOTOR_PITCH1 = (byte) 5;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Specifies the side of the robot that will execute the trajectory.
            */
   public byte robot_side_ = (byte) 255;

   /**
            * Specifies the list of finger motor names that we want to control.
            */
   public us.ihmc.idl.IDLSequence.Byte  valkyrie_finger_motor_names_;

   /**
            * The indexing for the joints refers the list of finger motor names.
            */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage jointspace_trajectory_;

   public ValkyrieHandFingerTrajectoryMessage()
   {



      valkyrie_finger_motor_names_ = new us.ihmc.idl.IDLSequence.Byte (6, "type_9");


      jointspace_trajectory_ = new controller_msgs.msg.dds.JointspaceTrajectoryMessage();

   }

   public ValkyrieHandFingerTrajectoryMessage(ValkyrieHandFingerTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(ValkyrieHandFingerTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


      robot_side_ = other.robot_side_;


      valkyrie_finger_motor_names_.set(other.valkyrie_finger_motor_names_);

      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.staticCopy(other.jointspace_trajectory_, jointspace_trajectory_);
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
            * Specifies the side of the robot that will execute the trajectory.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot that will execute the trajectory.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }



   /**
            * Specifies the list of finger motor names that we want to control.
            */
   public us.ihmc.idl.IDLSequence.Byte  getValkyrieFingerMotorNames()
   {
      return valkyrie_finger_motor_names_;
   }



   /**
            * The indexing for the joints refers the list of finger motor names.
            */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage getJointspaceTrajectory()
   {
      return jointspace_trajectory_;
   }


   public static Supplier<ValkyrieHandFingerTrajectoryMessagePubSubType> getPubSubType()
   {
      return ValkyrieHandFingerTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ValkyrieHandFingerTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ValkyrieHandFingerTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsByteSequence(this.valkyrie_finger_motor_names_, other.valkyrie_finger_motor_names_, epsilon)) return false;


      if (!this.jointspace_trajectory_.epsilonEquals(other.jointspace_trajectory_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ValkyrieHandFingerTrajectoryMessage)) return false;

      ValkyrieHandFingerTrajectoryMessage otherMyClass = (ValkyrieHandFingerTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      if (!this.valkyrie_finger_motor_names_.equals(otherMyClass.valkyrie_finger_motor_names_)) return false;

      if (!this.jointspace_trajectory_.equals(otherMyClass.jointspace_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ValkyrieHandFingerTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");

      builder.append("valkyrie_finger_motor_names=");
      builder.append(this.valkyrie_finger_motor_names_);      builder.append(", ");

      builder.append("jointspace_trajectory=");
      builder.append(this.jointspace_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
