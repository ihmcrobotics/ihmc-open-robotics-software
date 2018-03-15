package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class SimulatedLidarScanPacket implements Settable<SimulatedLidarScanPacket>, EpsilonComparable<SimulatedLidarScanPacket>
{
   private us.ihmc.idl.IDLSequence.Float ranges_;
   private int sensor_id_;
   private controller_msgs.msg.dds.LidarScanParametersMessage lidar_scan_parameters_;

   public SimulatedLidarScanPacket()
   {
      ranges_ = new us.ihmc.idl.IDLSequence.Float(100, "type_5");

      lidar_scan_parameters_ = new controller_msgs.msg.dds.LidarScanParametersMessage();
   }

   public SimulatedLidarScanPacket(SimulatedLidarScanPacket other)
   {
      set(other);
   }

   public void set(SimulatedLidarScanPacket other)
   {
      ranges_.set(other.ranges_);
      sensor_id_ = other.sensor_id_;

      controller_msgs.msg.dds.LidarScanParametersMessagePubSubType.staticCopy(other.lidar_scan_parameters_, lidar_scan_parameters_);
   }

   public us.ihmc.idl.IDLSequence.Float getRanges()
   {
      return ranges_;
   }

   public int getSensorId()
   {
      return sensor_id_;
   }

   public void setSensorId(int sensor_id)
   {
      sensor_id_ = sensor_id;
   }

   public controller_msgs.msg.dds.LidarScanParametersMessage getLidarScanParameters()
   {
      return lidar_scan_parameters_;
   }

   @Override
   public boolean epsilonEquals(SimulatedLidarScanPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.ranges_, other.ranges_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sensor_id_, other.sensor_id_, epsilon))
         return false;

      if (!this.lidar_scan_parameters_.epsilonEquals(other.lidar_scan_parameters_, epsilon))
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
      if (!(other instanceof SimulatedLidarScanPacket))
         return false;

      SimulatedLidarScanPacket otherMyClass = (SimulatedLidarScanPacket) other;

      if (!this.ranges_.equals(otherMyClass.ranges_))
         return false;

      if (this.sensor_id_ != otherMyClass.sensor_id_)
         return false;

      if (!this.lidar_scan_parameters_.equals(otherMyClass.lidar_scan_parameters_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SimulatedLidarScanPacket {");
      builder.append("ranges=");
      builder.append(this.ranges_);

      builder.append(", ");
      builder.append("sensor_id=");
      builder.append(this.sensor_id_);

      builder.append(", ");
      builder.append("lidar_scan_parameters=");
      builder.append(this.lidar_scan_parameters_);

      builder.append("}");
      return builder.toString();
   }
}