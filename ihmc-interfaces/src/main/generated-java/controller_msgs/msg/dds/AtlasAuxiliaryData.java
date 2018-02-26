package controller_msgs.msg.dds;

/**
 * Definition of the class "AtlasAuxiliaryData" defined in AtlasAuxiliaryData_.idl.
 *
 * This file was automatically generated from AtlasAuxiliaryData_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit AtlasAuxiliaryData_.idl instead.
 */
public class AtlasAuxiliaryData
{
   private std_msgs.msg.dds.Header header_;
   private controller_msgs.msg.dds.ElectricJointData[] electric_joint_data_;
   private controller_msgs.msg.dds.RawImuData[] raw_imu_data_;
   private controller_msgs.msg.dds.BatteryState battery_state_;
   private controller_msgs.msg.dds.PumpState pump_state_;

   public AtlasAuxiliaryData()
   {
      header_ = new std_msgs.msg.dds.Header();
      electric_joint_data_ = new controller_msgs.msg.dds.ElectricJointData[6];
      for (int b = 0; b < electric_joint_data_.length; ++b)
      {
         electric_joint_data_[b] = new controller_msgs.msg.dds.ElectricJointData();
      }
      raw_imu_data_ = new controller_msgs.msg.dds.RawImuData[15];
      for (int d = 0; d < raw_imu_data_.length; ++d)
      {
         raw_imu_data_[d] = new controller_msgs.msg.dds.RawImuData();
      }
      battery_state_ = new controller_msgs.msg.dds.BatteryState();
      pump_state_ = new controller_msgs.msg.dds.PumpState();
   }

   public void set(AtlasAuxiliaryData other)
   {
      std_msgs.msg.dds.HeaderPubSubType.staticCopy(other.header_, header_);
      for (int f = 0; f < electric_joint_data_.length; ++f)
      {
         controller_msgs.msg.dds.ElectricJointDataPubSubType.staticCopy(other.electric_joint_data_[f], electric_joint_data_[f]);
      }

      for (int h = 0; h < raw_imu_data_.length; ++h)
      {
         controller_msgs.msg.dds.RawImuDataPubSubType.staticCopy(other.raw_imu_data_[h], raw_imu_data_[h]);
      }

      controller_msgs.msg.dds.BatteryStatePubSubType.staticCopy(other.battery_state_, battery_state_);
      controller_msgs.msg.dds.PumpStatePubSubType.staticCopy(other.pump_state_, pump_state_);
   }

   public std_msgs.msg.dds.Header getHeader()
   {
      return header_;
   }

   public controller_msgs.msg.dds.ElectricJointData[] getElectric_joint_data()
   {
      return electric_joint_data_;
   }

   public controller_msgs.msg.dds.RawImuData[] getRaw_imu_data()
   {
      return raw_imu_data_;
   }

   public controller_msgs.msg.dds.BatteryState getBattery_state()
   {
      return battery_state_;
   }

   public controller_msgs.msg.dds.PumpState getPump_state()
   {
      return pump_state_;
   }

   @Override
   public boolean equals(java.lang.Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof AtlasAuxiliaryData))
         return false;
      AtlasAuxiliaryData otherMyClass = (AtlasAuxiliaryData) other;
      boolean returnedValue = true;

      returnedValue &= this.header_.equals(otherMyClass.header_);

      for (int j = 0; j < electric_joint_data_.length; ++j)
      {
         returnedValue &= electric_joint_data_[j].equals(otherMyClass.electric_joint_data_[j]);
      }
      for (int l = 0; l < raw_imu_data_.length; ++l)
      {
         returnedValue &= raw_imu_data_[l].equals(otherMyClass.raw_imu_data_[l]);
      }
      returnedValue &= this.battery_state_.equals(otherMyClass.battery_state_);

      returnedValue &= this.pump_state_.equals(otherMyClass.pump_state_);

      return returnedValue;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasAuxiliaryData {");
      builder.append("header=");
      builder.append(this.header_);

      builder.append(", ");
      builder.append("electric_joint_data=");
      builder.append(java.util.Arrays.toString(this.electric_joint_data_));

      builder.append(", ");
      builder.append("raw_imu_data=");
      builder.append(java.util.Arrays.toString(this.raw_imu_data_));

      builder.append(", ");
      builder.append("battery_state=");
      builder.append(this.battery_state_);

      builder.append(", ");
      builder.append("pump_state=");
      builder.append(this.pump_state_);

      builder.append("}");
      return builder.toString();
   }
}