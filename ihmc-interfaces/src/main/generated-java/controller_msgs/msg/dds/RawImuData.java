package controller_msgs.msg.dds;

/**
 * Definition of the class "RawImuData" defined in RawImuData_.idl.
 *
 * This file was automatically generated from RawImuData_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit RawImuData_.idl instead.
 */
public class RawImuData
{
   private long timestamp_;
   private long packet_count_;
   private us.ihmc.euclid.tuple3D.Vector3D imu_rates_;
   private us.ihmc.euclid.tuple3D.Vector3D imu_deltas_;

   public RawImuData()
   {
      imu_rates_ = new us.ihmc.euclid.tuple3D.Vector3D();
      imu_deltas_ = new us.ihmc.euclid.tuple3D.Vector3D();
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

   public long getPacket_count()
   {
      return packet_count_;
   }

   public void setPacket_count(long packet_count)
   {
      packet_count_ = packet_count;
   }

   public us.ihmc.euclid.tuple3D.Vector3D getImu_rates()
   {
      return imu_rates_;
   }

   public us.ihmc.euclid.tuple3D.Vector3D getImu_deltas()
   {
      return imu_deltas_;
   }

   @Override
   public boolean equals(java.lang.Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof RawImuData))
         return false;
      RawImuData otherMyClass = (RawImuData) other;
      boolean returnedValue = true;

      returnedValue &= this.timestamp_ == otherMyClass.timestamp_;

      returnedValue &= this.packet_count_ == otherMyClass.packet_count_;

      returnedValue &= this.imu_rates_.equals(otherMyClass.imu_rates_);

      returnedValue &= this.imu_deltas_.equals(otherMyClass.imu_deltas_);

      return returnedValue;
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