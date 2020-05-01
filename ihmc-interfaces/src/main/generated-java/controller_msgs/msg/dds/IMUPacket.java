package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * General purpose packet to carry IMU data. (need to be replace with the ROS IMUData)
       */
public class IMUPacket extends Packet<IMUPacket> implements Settable<IMUPacket>, EpsilonComparable<IMUPacket>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public us.ihmc.euclid.tuple4D.Quaternion orientation_;

   public us.ihmc.euclid.tuple3D.Vector3D angular_velocity_;

   public us.ihmc.euclid.tuple3D.Vector3D linear_acceleration_;

   public double time_;

   public IMUPacket()
   {


      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();

      angular_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();

      linear_acceleration_ = new us.ihmc.euclid.tuple3D.Vector3D();


   }

   public IMUPacket(IMUPacket other)
   {
      this();
      set(other);
   }

   public void set(IMUPacket other)
   {

      sequence_id_ = other.sequence_id_;


      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.angular_velocity_, angular_velocity_);

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.linear_acceleration_, linear_acceleration_);

      time_ = other.time_;

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



   public us.ihmc.euclid.tuple4D.Quaternion getOrientation()
   {
      return orientation_;
   }



   public us.ihmc.euclid.tuple3D.Vector3D getAngularVelocity()
   {
      return angular_velocity_;
   }



   public us.ihmc.euclid.tuple3D.Vector3D getLinearAcceleration()
   {
      return linear_acceleration_;
   }


   public void setTime(double time)
   {
      time_ = time;
   }
   public double getTime()
   {
      return time_;
   }


   public static Supplier<IMUPacketPubSubType> getPubSubType()
   {
      return IMUPacketPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return IMUPacketPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(IMUPacket other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon)) return false;

      if (!this.angular_velocity_.epsilonEquals(other.angular_velocity_, epsilon)) return false;

      if (!this.linear_acceleration_.epsilonEquals(other.linear_acceleration_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.time_, other.time_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof IMUPacket)) return false;

      IMUPacket otherMyClass = (IMUPacket) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.orientation_.equals(otherMyClass.orientation_)) return false;

      if (!this.angular_velocity_.equals(otherMyClass.angular_velocity_)) return false;

      if (!this.linear_acceleration_.equals(otherMyClass.linear_acceleration_)) return false;

      if(this.time_ != otherMyClass.time_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("IMUPacket {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("orientation=");
      builder.append(this.orientation_);      builder.append(", ");

      builder.append("angular_velocity=");
      builder.append(this.angular_velocity_);      builder.append(", ");

      builder.append("linear_acceleration=");
      builder.append(this.linear_acceleration_);      builder.append(", ");

      builder.append("time=");
      builder.append(this.time_);
      builder.append("}");
      return builder.toString();
   }
}
