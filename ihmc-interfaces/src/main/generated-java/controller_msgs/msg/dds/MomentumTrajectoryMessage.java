package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message can be used to send a predefined angular momentum trajectory to the controller.
       * This trajectory will be used for ICP planning.
       */
public class MomentumTrajectoryMessage extends Packet<MomentumTrajectoryMessage> implements Settable<MomentumTrajectoryMessage>, EpsilonComparable<MomentumTrajectoryMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * List of angular momentum trajectory waypoints.
            * Each waypoint contains the angular momentum and the angular momentum rate at a given time.
            * NOTE: As the angular momentum can't be encoded using the SO3TrajectoryMessage, the EuclideanTrajectoryMessage is used.
            */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage angular_momentum_trajectory_;

   public MomentumTrajectoryMessage()
   {


      angular_momentum_trajectory_ = new controller_msgs.msg.dds.EuclideanTrajectoryMessage();

   }

   public MomentumTrajectoryMessage(MomentumTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(MomentumTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


      controller_msgs.msg.dds.EuclideanTrajectoryMessagePubSubType.staticCopy(other.angular_momentum_trajectory_, angular_momentum_trajectory_);
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
            * List of angular momentum trajectory waypoints.
            * Each waypoint contains the angular momentum and the angular momentum rate at a given time.
            * NOTE: As the angular momentum can't be encoded using the SO3TrajectoryMessage, the EuclideanTrajectoryMessage is used.
            */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage getAngularMomentumTrajectory()
   {
      return angular_momentum_trajectory_;
   }


   public static Supplier<MomentumTrajectoryMessagePubSubType> getPubSubType()
   {
      return MomentumTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MomentumTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MomentumTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.angular_momentum_trajectory_.epsilonEquals(other.angular_momentum_trajectory_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MomentumTrajectoryMessage)) return false;

      MomentumTrajectoryMessage otherMyClass = (MomentumTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.angular_momentum_trajectory_.equals(otherMyClass.angular_momentum_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MomentumTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("angular_momentum_trajectory=");
      builder.append(this.angular_momentum_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
