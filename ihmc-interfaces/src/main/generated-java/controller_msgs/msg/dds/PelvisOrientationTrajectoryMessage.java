package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to move in taskspace the pelvis to the desired orientation while going through the specified trajectory points.
       * A Hermite based curve (third order) is used to interpolate the orientations.
       * This message allows controlling the pelvis orientation without interfering with position that will still be controlled to maintain the current desired capture point position.
       * To execute a normal trajectory to reach a desired pelvis orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.
       */
public class PelvisOrientationTrajectoryMessage extends Packet<PelvisOrientationTrajectoryMessage> implements Settable<PelvisOrientationTrajectoryMessage>, EpsilonComparable<PelvisOrientationTrajectoryMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * When the robot is walking, restrictions on upper-body motion may be applied.
            * To by-pass the safety check and force the execution of this message, set this field to true.
            */
   public boolean force_execution_;

   /**
            * Whether the pelvis orientation is allowed to be controlled by the user when the robot is walking.
            */
   public boolean enable_user_pelvis_control_during_walking_;

   /**
            * The orientation trajectory information.
            */
   public controller_msgs.msg.dds.SO3TrajectoryMessage so3_trajectory_;

   public PelvisOrientationTrajectoryMessage()
   {




      so3_trajectory_ = new controller_msgs.msg.dds.SO3TrajectoryMessage();

   }

   public PelvisOrientationTrajectoryMessage(PelvisOrientationTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(PelvisOrientationTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


      force_execution_ = other.force_execution_;


      enable_user_pelvis_control_during_walking_ = other.enable_user_pelvis_control_during_walking_;


      controller_msgs.msg.dds.SO3TrajectoryMessagePubSubType.staticCopy(other.so3_trajectory_, so3_trajectory_);
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
            * When the robot is walking, restrictions on upper-body motion may be applied.
            * To by-pass the safety check and force the execution of this message, set this field to true.
            */
   public void setForceExecution(boolean force_execution)
   {
      force_execution_ = force_execution;
   }
   /**
            * When the robot is walking, restrictions on upper-body motion may be applied.
            * To by-pass the safety check and force the execution of this message, set this field to true.
            */
   public boolean getForceExecution()
   {
      return force_execution_;
   }


   /**
            * Whether the pelvis orientation is allowed to be controlled by the user when the robot is walking.
            */
   public void setEnableUserPelvisControlDuringWalking(boolean enable_user_pelvis_control_during_walking)
   {
      enable_user_pelvis_control_during_walking_ = enable_user_pelvis_control_during_walking;
   }
   /**
            * Whether the pelvis orientation is allowed to be controlled by the user when the robot is walking.
            */
   public boolean getEnableUserPelvisControlDuringWalking()
   {
      return enable_user_pelvis_control_during_walking_;
   }



   /**
            * The orientation trajectory information.
            */
   public controller_msgs.msg.dds.SO3TrajectoryMessage getSo3Trajectory()
   {
      return so3_trajectory_;
   }


   public static Supplier<PelvisOrientationTrajectoryMessagePubSubType> getPubSubType()
   {
      return PelvisOrientationTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PelvisOrientationTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PelvisOrientationTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.force_execution_, other.force_execution_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_user_pelvis_control_during_walking_, other.enable_user_pelvis_control_during_walking_, epsilon)) return false;


      if (!this.so3_trajectory_.epsilonEquals(other.so3_trajectory_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PelvisOrientationTrajectoryMessage)) return false;

      PelvisOrientationTrajectoryMessage otherMyClass = (PelvisOrientationTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.force_execution_ != otherMyClass.force_execution_) return false;


      if(this.enable_user_pelvis_control_during_walking_ != otherMyClass.enable_user_pelvis_control_during_walking_) return false;


      if (!this.so3_trajectory_.equals(otherMyClass.so3_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PelvisOrientationTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("force_execution=");
      builder.append(this.force_execution_);      builder.append(", ");

      builder.append("enable_user_pelvis_control_during_walking=");
      builder.append(this.enable_user_pelvis_control_during_walking_);      builder.append(", ");

      builder.append("so3_trajectory=");
      builder.append(this.so3_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
