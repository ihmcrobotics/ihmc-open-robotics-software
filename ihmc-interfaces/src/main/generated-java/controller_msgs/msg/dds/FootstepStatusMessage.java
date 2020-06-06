package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message gives the status of the current footstep from the controller as well as the position and orientation of the footstep in world coordinates.
       */
public class FootstepStatusMessage extends Packet<FootstepStatusMessage> implements Settable<FootstepStatusMessage>, EpsilonComparable<FootstepStatusMessage>
{

   public static final byte FOOTSTEP_STATUS_STARTED = (byte) 0;

   public static final byte FOOTSTEP_STATUS_COMPLETED = (byte) 1;

   public static final byte ROBOT_SIDE_LEFT = (byte) 0;

   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * The current footstep status enum value.
            */
   public byte footstep_status_ = (byte) 255;

   /**
            * footstep_index starts at 0 and monotonically increases with each completed footstep in a given FootstepDataListMessage.
            */
   public int footstep_index_;

   /**
            * The robot side (left or right) that this footstep status correlates to.
            */
   public byte robot_side_ = (byte) 255;

   /**
            * Specifies the desired position of the foot sent to the controller as opposed to where the foot actually landed.
            */
   public us.ihmc.euclid.tuple3D.Point3D desired_foot_position_in_world_;

   /**
            * Specifies the desired orientation of the foot sent to the controller as opposed to where the foot actually landed.
            */
   public us.ihmc.euclid.tuple4D.Quaternion desired_foot_orientation_in_world_;

   /**
            * Specifies the position of where the foot actually landed.
            */
   public us.ihmc.euclid.tuple3D.Point3D actual_foot_position_in_world_;

   /**
            * Specifies the orientation of where the foot actually landed.
            */
   public us.ihmc.euclid.tuple4D.Quaternion actual_foot_orientation_in_world_;

   /**
            * This is the swing duration of the step.
            */
   public double swing_duration_;

   public FootstepStatusMessage()
   {





      desired_foot_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();

      desired_foot_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();

      actual_foot_position_in_world_ = new us.ihmc.euclid.tuple3D.Point3D();

      actual_foot_orientation_in_world_ = new us.ihmc.euclid.tuple4D.Quaternion();


   }

   public FootstepStatusMessage(FootstepStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(FootstepStatusMessage other)
   {

      sequence_id_ = other.sequence_id_;


      footstep_status_ = other.footstep_status_;


      footstep_index_ = other.footstep_index_;


      robot_side_ = other.robot_side_;


      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.desired_foot_position_in_world_, desired_foot_position_in_world_);

      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.desired_foot_orientation_in_world_, desired_foot_orientation_in_world_);

      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.actual_foot_position_in_world_, actual_foot_position_in_world_);

      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.actual_foot_orientation_in_world_, actual_foot_orientation_in_world_);

      swing_duration_ = other.swing_duration_;

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
            * The current footstep status enum value.
            */
   public void setFootstepStatus(byte footstep_status)
   {
      footstep_status_ = footstep_status;
   }
   /**
            * The current footstep status enum value.
            */
   public byte getFootstepStatus()
   {
      return footstep_status_;
   }


   /**
            * footstep_index starts at 0 and monotonically increases with each completed footstep in a given FootstepDataListMessage.
            */
   public void setFootstepIndex(int footstep_index)
   {
      footstep_index_ = footstep_index;
   }
   /**
            * footstep_index starts at 0 and monotonically increases with each completed footstep in a given FootstepDataListMessage.
            */
   public int getFootstepIndex()
   {
      return footstep_index_;
   }


   /**
            * The robot side (left or right) that this footstep status correlates to.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * The robot side (left or right) that this footstep status correlates to.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }



   /**
            * Specifies the desired position of the foot sent to the controller as opposed to where the foot actually landed.
            */
   public us.ihmc.euclid.tuple3D.Point3D getDesiredFootPositionInWorld()
   {
      return desired_foot_position_in_world_;
   }



   /**
            * Specifies the desired orientation of the foot sent to the controller as opposed to where the foot actually landed.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getDesiredFootOrientationInWorld()
   {
      return desired_foot_orientation_in_world_;
   }



   /**
            * Specifies the position of where the foot actually landed.
            */
   public us.ihmc.euclid.tuple3D.Point3D getActualFootPositionInWorld()
   {
      return actual_foot_position_in_world_;
   }



   /**
            * Specifies the orientation of where the foot actually landed.
            */
   public us.ihmc.euclid.tuple4D.Quaternion getActualFootOrientationInWorld()
   {
      return actual_foot_orientation_in_world_;
   }


   /**
            * This is the swing duration of the step.
            */
   public void setSwingDuration(double swing_duration)
   {
      swing_duration_ = swing_duration;
   }
   /**
            * This is the swing duration of the step.
            */
   public double getSwingDuration()
   {
      return swing_duration_;
   }


   public static Supplier<FootstepStatusMessagePubSubType> getPubSubType()
   {
      return FootstepStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return FootstepStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(FootstepStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.footstep_status_, other.footstep_status_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.footstep_index_, other.footstep_index_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      if (!this.desired_foot_position_in_world_.epsilonEquals(other.desired_foot_position_in_world_, epsilon)) return false;

      if (!this.desired_foot_orientation_in_world_.epsilonEquals(other.desired_foot_orientation_in_world_, epsilon)) return false;

      if (!this.actual_foot_position_in_world_.epsilonEquals(other.actual_foot_position_in_world_, epsilon)) return false;

      if (!this.actual_foot_orientation_in_world_.epsilonEquals(other.actual_foot_orientation_in_world_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_, other.swing_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof FootstepStatusMessage)) return false;

      FootstepStatusMessage otherMyClass = (FootstepStatusMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.footstep_status_ != otherMyClass.footstep_status_) return false;


      if(this.footstep_index_ != otherMyClass.footstep_index_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      if (!this.desired_foot_position_in_world_.equals(otherMyClass.desired_foot_position_in_world_)) return false;

      if (!this.desired_foot_orientation_in_world_.equals(otherMyClass.desired_foot_orientation_in_world_)) return false;

      if (!this.actual_foot_position_in_world_.equals(otherMyClass.actual_foot_position_in_world_)) return false;

      if (!this.actual_foot_orientation_in_world_.equals(otherMyClass.actual_foot_orientation_in_world_)) return false;

      if(this.swing_duration_ != otherMyClass.swing_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("FootstepStatusMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("footstep_status=");
      builder.append(this.footstep_status_);      builder.append(", ");

      builder.append("footstep_index=");
      builder.append(this.footstep_index_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");

      builder.append("desired_foot_position_in_world=");
      builder.append(this.desired_foot_position_in_world_);      builder.append(", ");

      builder.append("desired_foot_orientation_in_world=");
      builder.append(this.desired_foot_orientation_in_world_);      builder.append(", ");

      builder.append("actual_foot_position_in_world=");
      builder.append(this.actual_foot_position_in_world_);      builder.append(", ");

      builder.append("actual_foot_orientation_in_world=");
      builder.append(this.actual_foot_orientation_in_world_);      builder.append(", ");

      builder.append("swing_duration=");
      builder.append(this.swing_duration_);
      builder.append("}");
      return builder.toString();
   }
}
