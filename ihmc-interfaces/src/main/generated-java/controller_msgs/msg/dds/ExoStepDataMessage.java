package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message specifies the position, orientation and side (left or right) of a desired footstep in world frame.
       */
public class ExoStepDataMessage extends Packet<ExoStepDataMessage> implements Settable<ExoStepDataMessage>, EpsilonComparable<ExoStepDataMessage>
{

   public static final byte ROBOT_SIDE_LEFT = (byte) 0;

   public static final byte ROBOT_SIDE_RIGHT = (byte) 1;

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Specifies which foot will swing to reach the footstep.
            */
   public byte robot_side_ = (byte) 255;

   /**
            * Specifies the length of the footstep.
            */
   public double step_length_;

   /**
            * Specifies the height of the footstep.
            */
   public double step_height_;

   /**
            * Specifies the ending pitch of the footstep.
            */
   public double step_pitch_;

   /**
            * Contains information on how high the robot should swing its foot.
            * This affects trajectory types TRAJECTORY_TYPE_DEFAULT and TRAJECTORY_TYPE_OBSTACLE_CLEARANCE.
            * If a value smaller then the minimal swing height is chosen (e.g. 0.0) the swing height will be changed to a default value.
            */
   public double swing_height_ = -1.0;

   /**
            * The swingDuration is the time a foot is not in ground contact during a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default swing_duration.
            */
   public double swing_duration_ = -1.0;

   /**
            * The transferDuration is the time spent with the feet in ground contact before a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
            */
   public double transfer_duration_ = -1.0;

   public ExoStepDataMessage()
   {









   }

   public ExoStepDataMessage(ExoStepDataMessage other)
   {
      this();
      set(other);
   }

   public void set(ExoStepDataMessage other)
   {

      sequence_id_ = other.sequence_id_;


      robot_side_ = other.robot_side_;


      step_length_ = other.step_length_;


      step_height_ = other.step_height_;


      step_pitch_ = other.step_pitch_;


      swing_height_ = other.swing_height_;


      swing_duration_ = other.swing_duration_;


      transfer_duration_ = other.transfer_duration_;

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
            * Specifies which foot will swing to reach the footstep.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies which foot will swing to reach the footstep.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }


   /**
            * Specifies the length of the footstep.
            */
   public void setStepLength(double step_length)
   {
      step_length_ = step_length;
   }
   /**
            * Specifies the length of the footstep.
            */
   public double getStepLength()
   {
      return step_length_;
   }


   /**
            * Specifies the height of the footstep.
            */
   public void setStepHeight(double step_height)
   {
      step_height_ = step_height;
   }
   /**
            * Specifies the height of the footstep.
            */
   public double getStepHeight()
   {
      return step_height_;
   }


   /**
            * Specifies the ending pitch of the footstep.
            */
   public void setStepPitch(double step_pitch)
   {
      step_pitch_ = step_pitch;
   }
   /**
            * Specifies the ending pitch of the footstep.
            */
   public double getStepPitch()
   {
      return step_pitch_;
   }


   /**
            * Contains information on how high the robot should swing its foot.
            * This affects trajectory types TRAJECTORY_TYPE_DEFAULT and TRAJECTORY_TYPE_OBSTACLE_CLEARANCE.
            * If a value smaller then the minimal swing height is chosen (e.g. 0.0) the swing height will be changed to a default value.
            */
   public void setSwingHeight(double swing_height)
   {
      swing_height_ = swing_height;
   }
   /**
            * Contains information on how high the robot should swing its foot.
            * This affects trajectory types TRAJECTORY_TYPE_DEFAULT and TRAJECTORY_TYPE_OBSTACLE_CLEARANCE.
            * If a value smaller then the minimal swing height is chosen (e.g. 0.0) the swing height will be changed to a default value.
            */
   public double getSwingHeight()
   {
      return swing_height_;
   }


   /**
            * The swingDuration is the time a foot is not in ground contact during a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default swing_duration.
            */
   public void setSwingDuration(double swing_duration)
   {
      swing_duration_ = swing_duration;
   }
   /**
            * The swingDuration is the time a foot is not in ground contact during a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default swing_duration.
            */
   public double getSwingDuration()
   {
      return swing_duration_;
   }


   /**
            * The transferDuration is the time spent with the feet in ground contact before a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
            */
   public void setTransferDuration(double transfer_duration)
   {
      transfer_duration_ = transfer_duration;
   }
   /**
            * The transferDuration is the time spent with the feet in ground contact before a step.
            * If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
            */
   public double getTransferDuration()
   {
      return transfer_duration_;
   }


   public static Supplier<ExoStepDataMessagePubSubType> getPubSubType()
   {
      return ExoStepDataMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ExoStepDataMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ExoStepDataMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_length_, other.step_length_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_height_, other.step_height_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.step_pitch_, other.step_pitch_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_height_, other.swing_height_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.swing_duration_, other.swing_duration_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.transfer_duration_, other.transfer_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ExoStepDataMessage)) return false;

      ExoStepDataMessage otherMyClass = (ExoStepDataMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.robot_side_ != otherMyClass.robot_side_) return false;


      if(this.step_length_ != otherMyClass.step_length_) return false;


      if(this.step_height_ != otherMyClass.step_height_) return false;


      if(this.step_pitch_ != otherMyClass.step_pitch_) return false;


      if(this.swing_height_ != otherMyClass.swing_height_) return false;


      if(this.swing_duration_ != otherMyClass.swing_duration_) return false;


      if(this.transfer_duration_ != otherMyClass.transfer_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ExoStepDataMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");

      builder.append("step_length=");
      builder.append(this.step_length_);      builder.append(", ");

      builder.append("step_height=");
      builder.append(this.step_height_);      builder.append(", ");

      builder.append("step_pitch=");
      builder.append(this.step_pitch_);      builder.append(", ");

      builder.append("swing_height=");
      builder.append(this.swing_height_);      builder.append(", ");

      builder.append("swing_duration=");
      builder.append(this.swing_duration_);      builder.append(", ");

      builder.append("transfer_duration=");
      builder.append(this.transfer_duration_);
      builder.append("}");
      return builder.toString();
   }
}
