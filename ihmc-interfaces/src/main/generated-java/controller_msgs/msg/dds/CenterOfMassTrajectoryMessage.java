package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * Request the whole-body controller to track a desired trajectory for the center of mass.
       */
public class CenterOfMassTrajectoryMessage extends Packet<CenterOfMassTrajectoryMessage> implements Settable<CenterOfMassTrajectoryMessage>, EpsilonComparable<CenterOfMassTrajectoryMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * List of center of mass trajectory waypoints. Each waypoint contains the center of mass position and velocity at a given time.
            */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage euclidean_trajectory_;

   public CenterOfMassTrajectoryMessage()
   {


      euclidean_trajectory_ = new controller_msgs.msg.dds.EuclideanTrajectoryMessage();

   }

   public CenterOfMassTrajectoryMessage(CenterOfMassTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(CenterOfMassTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


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
            * List of center of mass trajectory waypoints. Each waypoint contains the center of mass position and velocity at a given time.
            */
   public controller_msgs.msg.dds.EuclideanTrajectoryMessage getEuclideanTrajectory()
   {
      return euclidean_trajectory_;
   }


   public static Supplier<CenterOfMassTrajectoryMessagePubSubType> getPubSubType()
   {
      return CenterOfMassTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return CenterOfMassTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(CenterOfMassTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.euclidean_trajectory_.epsilonEquals(other.euclidean_trajectory_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof CenterOfMassTrajectoryMessage)) return false;

      CenterOfMassTrajectoryMessage otherMyClass = (CenterOfMassTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.euclidean_trajectory_.equals(otherMyClass.euclidean_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("CenterOfMassTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("euclidean_trajectory=");
      builder.append(this.euclidean_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
