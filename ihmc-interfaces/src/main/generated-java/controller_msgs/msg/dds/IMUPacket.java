package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * General purpose packet to carry IMU data. (need to be replace with the ROS IMUData)
 */
public class IMUPacket extends Packet<IMUPacket> implements Settable<IMUPacket>, EpsilonComparable<IMUPacket>
{
   public us.ihmc.euclid.tuple4D.Quaternion orientation_;
   public us.ihmc.euclid.tuple3D.Vector3D angular_velocity_;
   public us.ihmc.euclid.tuple3D.Vector3D linear_acceleration_;

   public IMUPacket()
   {
      orientation_ = new us.ihmc.euclid.tuple4D.Quaternion();
      angular_velocity_ = new us.ihmc.euclid.tuple3D.Vector3D();
      linear_acceleration_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public IMUPacket(IMUPacket other)
   {
      set(other);
   }

   public void set(IMUPacket other)
   {
      geometry_msgs.msg.dds.QuaternionPubSubType.staticCopy(other.orientation_, orientation_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.angular_velocity_, angular_velocity_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.linear_acceleration_, linear_acceleration_);
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

   @Override
   public boolean epsilonEquals(IMUPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.orientation_.epsilonEquals(other.orientation_, epsilon))
         return false;

      if (!this.angular_velocity_.epsilonEquals(other.angular_velocity_, epsilon))
         return false;

      if (!this.linear_acceleration_.epsilonEquals(other.linear_acceleration_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof IMUPacket))
         return false;

      IMUPacket otherMyClass = (IMUPacket) other;

      if (!this.orientation_.equals(otherMyClass.orientation_))
         return false;

      if (!this.angular_velocity_.equals(otherMyClass.angular_velocity_))
         return false;

      if (!this.linear_acceleration_.equals(otherMyClass.linear_acceleration_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("IMUPacket {");
      builder.append("orientation=");
      builder.append(this.orientation_);

      builder.append(", ");
      builder.append("angular_velocity=");
      builder.append(this.angular_velocity_);

      builder.append(", ");
      builder.append("linear_acceleration=");
      builder.append(this.linear_acceleration_);

      builder.append("}");
      return builder.toString();
   }
}