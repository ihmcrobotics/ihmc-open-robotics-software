package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class RawImuData extends Packet<RawImuData> implements Settable<RawImuData>, EpsilonComparable<RawImuData>
{
   public long timestamp_;
   public long packet_count_;
   public us.ihmc.euclid.tuple3D.Vector3D imu_rates_;
   public us.ihmc.euclid.tuple3D.Vector3D imu_deltas_;

   public RawImuData()
   {

      imu_rates_ = new us.ihmc.euclid.tuple3D.Vector3D();
      imu_deltas_ = new us.ihmc.euclid.tuple3D.Vector3D();
   }

   public RawImuData(RawImuData other)
   {
      set(other);
   }

   public void set(RawImuData other)
   {
      timestamp_ = other.timestamp_;

      packet_count_ = other.packet_count_;

      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.imu_rates_, imu_rates_);
      geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.imu_deltas_, imu_deltas_);
   }

   public long getTimestamp()
   {
      return timestamp_;
   }

   public void setTimestamp(long timestamp)
   {
      timestamp_ = timestamp;
   }

   public long getPacketCount()
   {
      return packet_count_;
   }

   public void setPacketCount(long packet_count)
   {
      packet_count_ = packet_count;
   }

   public us.ihmc.euclid.tuple3D.Vector3D getImuRates()
   {
      return imu_rates_;
   }

   public us.ihmc.euclid.tuple3D.Vector3D getImuDeltas()
   {
      return imu_deltas_;
   }

   @Override
   public boolean epsilonEquals(RawImuData other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestamp_, other.timestamp_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.packet_count_, other.packet_count_, epsilon))
         return false;

      if (!this.imu_rates_.epsilonEquals(other.imu_rates_, epsilon))
         return false;

      if (!this.imu_deltas_.epsilonEquals(other.imu_deltas_, epsilon))
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
      if (!(other instanceof RawImuData))
         return false;

      RawImuData otherMyClass = (RawImuData) other;

      if (this.timestamp_ != otherMyClass.timestamp_)
         return false;

      if (this.packet_count_ != otherMyClass.packet_count_)
         return false;

      if (!this.imu_rates_.equals(otherMyClass.imu_rates_))
         return false;

      if (!this.imu_deltas_.equals(otherMyClass.imu_deltas_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RawImuData {");
      builder.append("timestamp=");
      builder.append(this.timestamp_);

      builder.append(", ");
      builder.append("packet_count=");
      builder.append(this.packet_count_);

      builder.append(", ");
      builder.append("imu_rates=");
      builder.append(this.imu_rates_);

      builder.append(", ");
      builder.append("imu_deltas=");
      builder.append(this.imu_deltas_);

      builder.append("}");
      return builder.toString();
   }
}