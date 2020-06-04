package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to move the neck in jointspace to the desired joint angles while going through the specified trajectory points.
       * A third order polynomial function is used to interpolate between trajectory points.
       */
public class NeckTrajectoryMessage extends Packet<NeckTrajectoryMessage> implements Settable<NeckTrajectoryMessage>, EpsilonComparable<NeckTrajectoryMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * The trajectories for each joint in order from the one closest to the chest to the one the closest to the head.
            */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage jointspace_trajectory_;

   public NeckTrajectoryMessage()
   {


      jointspace_trajectory_ = new controller_msgs.msg.dds.JointspaceTrajectoryMessage();

   }

   public NeckTrajectoryMessage(NeckTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(NeckTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


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
            * The trajectories for each joint in order from the one closest to the chest to the one the closest to the head.
            */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage getJointspaceTrajectory()
   {
      return jointspace_trajectory_;
   }


   public static Supplier<NeckTrajectoryMessagePubSubType> getPubSubType()
   {
      return NeckTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return NeckTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(NeckTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.jointspace_trajectory_.epsilonEquals(other.jointspace_trajectory_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof NeckTrajectoryMessage)) return false;

      NeckTrajectoryMessage otherMyClass = (NeckTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.jointspace_trajectory_.equals(otherMyClass.jointspace_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("NeckTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("jointspace_trajectory=");
      builder.append(this.jointspace_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
