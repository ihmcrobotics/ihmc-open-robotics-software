package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to move the pelvis to a new height in the trajectory frame while going through the specified trajectory points.
       * Sending this command will not affect the pelvis horizontal position.
       * To control the pelvis 3D position use the PelvisTrajectoryMessage instead.
       */
public class PelvisHeightTrajectoryMessage extends Packet<PelvisHeightTrajectoryMessage> implements Settable<PelvisHeightTrajectoryMessage>, EpsilonComparable<PelvisHeightTrajectoryMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics.
            */
   public boolean enable_user_pelvis_control_;

   /**
            * If enable_user_pelvis_control is true then enable_user_pelvis_height_control_during_walking
            * will keep the height manager in user mode while walking.
            * If this is false the height manager will switch back to controller mode when walking.
            */
   public boolean enable_user_pelvis_control_during_walking_;

   /**
            * The position trajectory information.
            */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage euclidean_trajectory_;

   public PelvisHeightTrajectoryMessage()
   {




      euclidean_trajectory_ = new controller_msgs.msg.dds.EuclideanTrajectoryMessage();

   }

   public PelvisHeightTrajectoryMessage(PelvisHeightTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(PelvisHeightTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


      enable_user_pelvis_control_ = other.enable_user_pelvis_control_;


      enable_user_pelvis_control_during_walking_ = other.enable_user_pelvis_control_during_walking_;


      controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.staticCopy(other.euclidean_trajectory_, euclidean_trajectory_);
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
            * Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics.
            */
   public void setEnableUserPelvisControl(boolean enable_user_pelvis_control)
   {
      enable_user_pelvis_control_ = enable_user_pelvis_control;
   }
   /**
            * Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg kinematics.
            */
   public boolean getEnableUserPelvisControl()
   {
      return enable_user_pelvis_control_;
   }


   /**
            * If enable_user_pelvis_control is true then enable_user_pelvis_height_control_during_walking
            * will keep the height manager in user mode while walking.
            * If this is false the height manager will switch back to controller mode when walking.
            */
   public void setEnableUserPelvisControlDuringWalking(boolean enable_user_pelvis_control_during_walking)
   {
      enable_user_pelvis_control_during_walking_ = enable_user_pelvis_control_during_walking;
   }
   /**
            * If enable_user_pelvis_control is true then enable_user_pelvis_height_control_during_walking
            * will keep the height manager in user mode while walking.
            * If this is false the height manager will switch back to controller mode when walking.
            */
   public boolean getEnableUserPelvisControlDuringWalking()
   {
      return enable_user_pelvis_control_during_walking_;
   }



   /**
            * The position trajectory information.
            */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage getEuclideanTrajectory()
   {
      return euclidean_trajectory_;
   }


   public static Supplier<PelvisHeightTrajectoryMessagePubSubType> getPubSubType()
   {
      return PelvisHeightTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return PelvisHeightTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(PelvisHeightTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_user_pelvis_control_, other.enable_user_pelvis_control_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.enable_user_pelvis_control_during_walking_, other.enable_user_pelvis_control_during_walking_, epsilon)) return false;


      if (!this.euclidean_trajectory_.epsilonEquals(other.euclidean_trajectory_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof PelvisHeightTrajectoryMessage)) return false;

      PelvisHeightTrajectoryMessage otherMyClass = (PelvisHeightTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.enable_user_pelvis_control_ != otherMyClass.enable_user_pelvis_control_) return false;


      if(this.enable_user_pelvis_control_during_walking_ != otherMyClass.enable_user_pelvis_control_during_walking_) return false;


      if (!this.euclidean_trajectory_.equals(otherMyClass.euclidean_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("PelvisHeightTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("enable_user_pelvis_control=");
      builder.append(this.enable_user_pelvis_control_);      builder.append(", ");

      builder.append("enable_user_pelvis_control_during_walking=");
      builder.append(this.enable_user_pelvis_control_during_walking_);      builder.append(", ");

      builder.append("euclidean_trajectory=");
      builder.append(this.euclidean_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
