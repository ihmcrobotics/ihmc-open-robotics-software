package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This class is used to build a wrench (force & moment) profile over time.
       * A linear interpolation is used to interpolate between trajectory points.
       */
public class WrenchTrajectoryMessage extends Packet<WrenchTrajectoryMessage> implements Settable<WrenchTrajectoryMessage>, EpsilonComparable<WrenchTrajectoryMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * List of trajectory points (in taskpsace) to go through while executing the trajectory.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.WrenchTrajectoryPointMessage>  wrench_trajectory_points_;

   /**
            * Frame information for this message.
            */
   public controller_msgs.msg.dds.FrameInformation frame_information_;

   /**
            * Flag that tells the controller whether the use of a custom control frame is requested.
            */
   public boolean use_custom_control_frame_;

   /**
            * Pose of custom control frame expressed in the end-effector frame.
            * This is the frame attached to the rigid body that the wrench trajectory is defined for.
            */
   public us.ihmc.euclid.geometry.Pose3D control_frame_pose_;

   /**
            * Properties for queueing trajectories.
            */
   public controller_msgs.msg.dds.QueueableMessage queueing_properties_;

   public WrenchTrajectoryMessage()
   {


      wrench_trajectory_points_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.WrenchTrajectoryPointMessage> (50, new controller_msgs.msg.dds.WrenchTrajectoryPointMessagePubSubType());

      frame_information_ = new controller_msgs.msg.dds.FrameInformation();


      control_frame_pose_ = new us.ihmc.euclid.geometry.Pose3D();

      queueing_properties_ = new controller_msgs.msg.dds.QueueableMessage();

   }

   public WrenchTrajectoryMessage(WrenchTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(WrenchTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


      wrench_trajectory_points_.set(other.wrench_trajectory_points_);

      controller_msgs.msg.dds.FrameInformationPubSubType.staticCopy(other.frame_information_, frame_information_);

      use_custom_control_frame_ = other.use_custom_control_frame_;


      geometry_msgs.msg.dds.PosePubSubType.staticCopy(other.control_frame_pose_, control_frame_pose_);

      controller_msgs.msg.dds.QueueableMessagePubSubType.staticCopy(other.queueing_properties_, queueing_properties_);
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
            * List of trajectory points (in taskpsace) to go through while executing the trajectory.
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.WrenchTrajectoryPointMessage>  getWrenchTrajectoryPoints()
   {
      return wrench_trajectory_points_;
   }



   /**
            * Frame information for this message.
            */
   public controller_msgs.msg.dds.FrameInformation getFrameInformation()
   {
      return frame_information_;
   }


   /**
            * Flag that tells the controller whether the use of a custom control frame is requested.
            */
   public void setUseCustomControlFrame(boolean use_custom_control_frame)
   {
      use_custom_control_frame_ = use_custom_control_frame;
   }
   /**
            * Flag that tells the controller whether the use of a custom control frame is requested.
            */
   public boolean getUseCustomControlFrame()
   {
      return use_custom_control_frame_;
   }



   /**
            * Pose of custom control frame expressed in the end-effector frame.
            * This is the frame attached to the rigid body that the wrench trajectory is defined for.
            */
   public us.ihmc.euclid.geometry.Pose3D getControlFramePose()
   {
      return control_frame_pose_;
   }



   /**
            * Properties for queueing trajectories.
            */
   public controller_msgs.msg.dds.QueueableMessage getQueueingProperties()
   {
      return queueing_properties_;
   }


   public static Supplier<WrenchTrajectoryMessagePubSubType> getPubSubType()
   {
      return WrenchTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WrenchTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WrenchTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (this.wrench_trajectory_points_.size() != other.wrench_trajectory_points_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.wrench_trajectory_points_.size(); i++)
         {  if (!this.wrench_trajectory_points_.get(i).epsilonEquals(other.wrench_trajectory_points_.get(i), epsilon)) return false; }
      }


      if (!this.frame_information_.epsilonEquals(other.frame_information_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.use_custom_control_frame_, other.use_custom_control_frame_, epsilon)) return false;


      if (!this.control_frame_pose_.epsilonEquals(other.control_frame_pose_, epsilon)) return false;

      if (!this.queueing_properties_.epsilonEquals(other.queueing_properties_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WrenchTrajectoryMessage)) return false;

      WrenchTrajectoryMessage otherMyClass = (WrenchTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.wrench_trajectory_points_.equals(otherMyClass.wrench_trajectory_points_)) return false;

      if (!this.frame_information_.equals(otherMyClass.frame_information_)) return false;

      if(this.use_custom_control_frame_ != otherMyClass.use_custom_control_frame_) return false;


      if (!this.control_frame_pose_.equals(otherMyClass.control_frame_pose_)) return false;

      if (!this.queueing_properties_.equals(otherMyClass.queueing_properties_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WrenchTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("wrench_trajectory_points=");
      builder.append(this.wrench_trajectory_points_);      builder.append(", ");

      builder.append("frame_information=");
      builder.append(this.frame_information_);      builder.append(", ");

      builder.append("use_custom_control_frame=");
      builder.append(this.use_custom_control_frame_);      builder.append(", ");

      builder.append("control_frame_pose=");
      builder.append(this.control_frame_pose_);      builder.append(", ");

      builder.append("queueing_properties=");
      builder.append(this.queueing_properties_);
      builder.append("}");
      return builder.toString();
   }
}
