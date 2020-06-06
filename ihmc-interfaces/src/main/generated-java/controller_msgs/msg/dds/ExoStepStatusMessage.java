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
public class ExoStepStatusMessage extends Packet<ExoStepStatusMessage> implements Settable<ExoStepStatusMessage>, EpsilonComparable<ExoStepStatusMessage>
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
   public double desired_step_length_;

   public double desired_step_height_;

   public double desired_step_pitch_;

   public ExoStepStatusMessage()
   {








   }

   public ExoStepStatusMessage(ExoStepStatusMessage other)
   {
      this();
      set(other);
   }

   public void set(ExoStepStatusMessage other)
   {

      sequence_id_ = other.sequence_id_;


      footstep_status_ = other.footstep_status_;


      footstep_index_ = other.footstep_index_;


      robot_side_ = other.robot_side_;


      desired_step_length_ = other.desired_step_length_;


      desired_step_height_ = other.desired_step_height_;


      desired_step_pitch_ = other.desired_step_pitch_;

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
   public void setDesiredStepLength(double desired_step_length)
   {
      desired_step_length_ = desired_step_length;
   }
   /**
            * Specifies the desired position of the foot sent to the controller as opposed to where the foot actually landed.
            */
   public double getDesiredStepLength()
   {
      return desired_step_length_;
   }


   public void setDesiredStepHeight(double desired_step_height)
   {
      desired_step_height_ = desired_step_height;
   }
   public double getDesiredStepHeight()
   {
      return desired_step_height_;
   }


   public void setDesiredStepPitch(double desired_step_pitch)
   {
      desired_step_pitch_ = desired_step_pitch;
   }
   public double getDesiredStepPitch()
   {
      return desired_step_pitch_;
   }


   public static Supplier<ExoStepStatusMessagePubSubType> getPubSubType()
   {
      return ExoStepStatusMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ExoStepStatusMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ExoStepStatusMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.footstep_status_, other.footstep_status_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.footstep_index_, other.footstep_index_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_step_length_, other.desired_step_length_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_step_height_, other.desired_step_height_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.desired_step_pitch_, other.desired_step_pitch_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ExoStepStatusMessage)) return false;

      ExoStepStatusMessage otherMyClass = (ExoStepStatusMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.footstep_status_ != otherMyClass.footstep_status_) return false;


      if(this.footstep_index_ != otherMyClass.footstep_index_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      if(this.desired_step_length_ != otherMyClass.desired_step_length_) return false;


      if(this.desired_step_height_ != otherMyClass.desired_step_height_) return false;


      if(this.desired_step_pitch_ != otherMyClass.desired_step_pitch_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ExoStepStatusMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("footstep_status=");
      builder.append(this.footstep_status_);      builder.append(", ");

      builder.append("footstep_index=");
      builder.append(this.footstep_index_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");

      builder.append("desired_step_length=");
      builder.append(this.desired_step_length_);      builder.append(", ");

      builder.append("desired_step_height=");
      builder.append(this.desired_step_height_);      builder.append(", ");

      builder.append("desired_step_pitch=");
      builder.append(this.desired_step_pitch_);
      builder.append("}");
      return builder.toString();
   }
}
