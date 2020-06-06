package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message commands the controller to move in taskspace the chest to the desired orientation while going through the specified trajectory points.
       * A Hermite based curve (third order) is used to interpolate the orientations.
       * To execute a simple trajectory to reach a desired chest orientation, set only one trajectory point with zero velocity and its time to be equal to the desired trajectory time.
       */
public class ChestTrajectoryMessage extends Packet<ChestTrajectoryMessage> implements Settable<ChestTrajectoryMessage>, EpsilonComparable<ChestTrajectoryMessage>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   /**
            * The orientation trajectory information.
            */
   public controller_msgs.msg.dds.SO3TrajectoryMessage so3_trajectory_;

   public ChestTrajectoryMessage()
   {


      so3_trajectory_ = new controller_msgs.msg.dds.SO3TrajectoryMessage();

   }

   public ChestTrajectoryMessage(ChestTrajectoryMessage other)
   {
      this();
      set(other);
   }

   public void set(ChestTrajectoryMessage other)
   {

      sequence_id_ = other.sequence_id_;


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
            * The orientation trajectory information.
            */
   public controller_msgs.msg.dds.SO3TrajectoryMessage getSo3Trajectory()
   {
      return so3_trajectory_;
   }


   public static Supplier<ChestTrajectoryMessagePubSubType> getPubSubType()
   {
      return ChestTrajectoryMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ChestTrajectoryMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ChestTrajectoryMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.so3_trajectory_.epsilonEquals(other.so3_trajectory_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ChestTrajectoryMessage)) return false;

      ChestTrajectoryMessage otherMyClass = (ChestTrajectoryMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.so3_trajectory_.equals(otherMyClass.so3_trajectory_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ChestTrajectoryMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("so3_trajectory=");
      builder.append(this.so3_trajectory_);
      builder.append("}");
      return builder.toString();
   }
}
