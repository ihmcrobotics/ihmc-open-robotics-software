package ihmc_common_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This is part of the IHMC Common message package.
       * This class is used to build 1D trajectory messages including jointspace trajectory messages.
       * For 3D trajectory points look at:
       * - EuclideanTrajectoryMessage (translational),
       * - SO3TrajectoryPointMessage (rotational),
       * - SE3TrajectoryPointMessage (translational AND rotational).
       */
public class TrajectoryPoint1DMessage extends Packet<TrajectoryPoint1DMessage> implements Settable<TrajectoryPoint1DMessage>, EpsilonComparable<TrajectoryPoint1DMessage>
{
   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;
   /**
            * Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
            */
   public double time_;
   /**
            * Define the desired 1D position to be reached at this trajectory point.
            */
   public double position_;
   /**
            * Define the desired 1D velocity to be reached at this trajectory point.
            */
   public double velocity_;
   /**
            * Define the desired 1D acceleration to be reached at this trajectory point.
            */
   public double acceleration_;

   public TrajectoryPoint1DMessage()
   {
   }

   public TrajectoryPoint1DMessage(TrajectoryPoint1DMessage other)
   {
      this();
      set(other);
   }

   public void set(TrajectoryPoint1DMessage other)
   {
      sequence_id_ = other.sequence_id_;

      time_ = other.time_;

      position_ = other.position_;

      velocity_ = other.velocity_;

      acceleration_ = other.acceleration_;

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
            * Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
            */
   public void setTime(double time)
   {
      time_ = time;
   }
   /**
            * Time at which the trajectory point has to be reached. The time is relative to when the trajectory starts.
            */
   public double getTime()
   {
      return time_;
   }

   /**
            * Define the desired 1D position to be reached at this trajectory point.
            */
   public void setPosition(double position)
   {
      position_ = position;
   }
   /**
            * Define the desired 1D position to be reached at this trajectory point.
            */
   public double getPosition()
   {
      return position_;
   }

   /**
            * Define the desired 1D velocity to be reached at this trajectory point.
            */
   public void setVelocity(double velocity)
   {
      velocity_ = velocity;
   }
   /**
            * Define the desired 1D velocity to be reached at this trajectory point.
            */
   public double getVelocity()
   {
      return velocity_;
   }

   /**
            * Define the desired 1D acceleration to be reached at this trajectory point.
            */
   public void setAcceleration(double acceleration)
   {
      acceleration_ = acceleration;
   }
   /**
            * Define the desired 1D acceleration to be reached at this trajectory point.
            */
   public double getAcceleration()
   {
      return acceleration_;
   }


   public static Supplier<TrajectoryPoint1DMessagePubSubType> getPubSubType()
   {
      return TrajectoryPoint1DMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return TrajectoryPoint1DMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(TrajectoryPoint1DMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_, other.time_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.position_, other.position_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.velocity_, other.velocity_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.acceleration_, other.acceleration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof TrajectoryPoint1DMessage)) return false;

      TrajectoryPoint1DMessage otherMyClass = (TrajectoryPoint1DMessage) other;

      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;

      if(this.time_ != otherMyClass.time_) return false;

      if(this.position_ != otherMyClass.position_) return false;

      if(this.velocity_ != otherMyClass.velocity_) return false;

      if(this.acceleration_ != otherMyClass.acceleration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("TrajectoryPoint1DMessage {");
      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");
      builder.append("time=");
      builder.append(this.time_);      builder.append(", ");
      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");
      builder.append("velocity=");
      builder.append(this.velocity_);      builder.append(", ");
      builder.append("acceleration=");
      builder.append(this.acceleration_);
      builder.append("}");
      return builder.toString();
   }
}
