package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the IHMC whole-body controller API.
       * This message is used to build trajectory messages in taskspace.
       * It holds the only the translational information for one trajectory point (position & linear velocity).
       * Feel free to look at SO3TrajectoryPointMessage (rotational) and SE3TrajectoryPointMessage (rotational AND translational).
       */
public class EuclideanTrajectoryPointMessage extends Packet<EuclideanTrajectoryPointMessage> implements Settable<EuclideanTrajectoryPointMessage>, EpsilonComparable<EuclideanTrajectoryPointMessage>
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
            * Define the desired 3D position to be reached at this trajectory point. It is expressed in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D position_;

   /**
            * Define the desired 3D linear velocity to be reached at this trajectory point. It is expressed in world frame.
            */
   public us.ihmc.euclid.tuple3D.Vector3D linear_velocity_;

   public EuclideanTrajectoryPointMessage()
   {



      position_ = new us.ihmc.euclid.tuple3D.Point3D();

      linear_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();

   }

   public EuclideanTrajectoryPointMessage(EuclideanTrajectoryPointMessage other)
   {
      this();
      set(other);
   }

   public void set(EuclideanTrajectoryPointMessage other)
   {

      sequence_id_ = other.sequence_id_;


      time_ = other.time_;


      geometry_msgs.msg.dds.PointPubSubType.staticCopy(other.position_, position_);

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.linear_velocity_, linear_velocity_);
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
            * Define the desired 3D position to be reached at this trajectory point. It is expressed in world frame.
            */
   public us.ihmc.euclid.tuple3D.Point3D getPosition()
   {
      return position_;
   }



   /**
            * Define the desired 3D linear velocity to be reached at this trajectory point. It is expressed in world frame.
            */
   public us.ihmc.euclid.tuple3D.Vector3D getLinearVelocity()
   {
      return linear_velocity_;
   }


   public static Supplier<EuclideanTrajectoryPointMessagePubSubType> getPubSubType()
   {
      return EuclideanTrajectoryPointMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return EuclideanTrajectoryPointMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(EuclideanTrajectoryPointMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_, other.time_, epsilon)) return false;


      if (!this.position_.epsilonEquals(other.position_, epsilon)) return false;

      if (!this.linear_velocity_.epsilonEquals(other.linear_velocity_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof EuclideanTrajectoryPointMessage)) return false;

      EuclideanTrajectoryPointMessage otherMyClass = (EuclideanTrajectoryPointMessage) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if(this.time_ != otherMyClass.time_) return false;


      if (!this.position_.equals(otherMyClass.position_)) return false;

      if (!this.linear_velocity_.equals(otherMyClass.linear_velocity_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("EuclideanTrajectoryPointMessage {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("time=");
      builder.append(this.time_);      builder.append(", ");

      builder.append("position=");
      builder.append(this.position_);      builder.append(", ");

      builder.append("linear_velocity=");
      builder.append(this.linear_velocity_);
      builder.append("}");
      return builder.toString();
   }
}
