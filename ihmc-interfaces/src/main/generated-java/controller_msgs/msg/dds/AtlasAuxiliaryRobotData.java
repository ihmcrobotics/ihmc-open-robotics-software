package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Message specific to Atlas, it is reported by the IHMC state estimator.
 */
public class AtlasAuxiliaryRobotData implements Settable<AtlasAuxiliaryRobotData>, EpsilonComparable<AtlasAuxiliaryRobotData>
{
   private us.ihmc.idl.IDLSequence.Float electric_joint_temperatures_;
   private us.ihmc.idl.IDLSequence.Float electric_joint_currents_;
   private us.ihmc.idl.IDLSequence.Float electric_joint_enabled_array_;
   private float[] raw_imu_timestamps_;
   private float[] raw_imu_packet_counts_;
   private us.ihmc.euclid.tuple3D.Vector3D[] raw_imu_rates_;
   private us.ihmc.euclid.tuple3D.Vector3D[] raw_imu_deltas_;
   private boolean battery_charging_;
   private float battery_voltage_;
   private float battery_current_;
   private float remaining_battery_time_;
   private float remaining_amp_hours_;
   private float remaining_charge_percentage_;
   private long battery_cycle_count_;
   private float pump_inlet_pressure_;
   private float pump_supply_pressure_;
   private float air_sump_pressure_;
   private float pump_supply_temperature_;
   private float pump_rpm_;
   private float motor_temperature_;
   private float motor_driver_temperature_;

   public AtlasAuxiliaryRobotData()
   {
      electric_joint_temperatures_ = new us.ihmc.idl.IDLSequence.Float(6, "type_5");

      electric_joint_currents_ = new us.ihmc.idl.IDLSequence.Float(6, "type_5");

      electric_joint_enabled_array_ = new us.ihmc.idl.IDLSequence.Float(6, "type_5");

      raw_imu_timestamps_ = new float[15];

      raw_imu_packet_counts_ = new float[15];

      raw_imu_rates_ = new us.ihmc.euclid.tuple3D.Vector3D[15];
      for (int b = 0; b < raw_imu_rates_.length; ++b)
      {
         raw_imu_rates_[b] = new us.ihmc.euclid.tuple3D.Vector3D();
      }

      raw_imu_deltas_ = new us.ihmc.euclid.tuple3D.Vector3D[15];
      for (int d = 0; d < raw_imu_deltas_.length; ++d)
      {
         raw_imu_deltas_[d] = new us.ihmc.euclid.tuple3D.Vector3D();
      }
   }

   public AtlasAuxiliaryRobotData(AtlasAuxiliaryRobotData other)
   {
      set(other);
   }

   public void set(AtlasAuxiliaryRobotData other)
   {
      electric_joint_temperatures_.set(other.electric_joint_temperatures_);
      electric_joint_currents_.set(other.electric_joint_currents_);
      electric_joint_enabled_array_.set(other.electric_joint_enabled_array_);
      for (int f = 0; f < raw_imu_timestamps_.length; ++f)
      {
         raw_imu_timestamps_[f] = other.raw_imu_timestamps_[f];
      }

      for (int h = 0; h < raw_imu_packet_counts_.length; ++h)
      {
         raw_imu_packet_counts_[h] = other.raw_imu_packet_counts_[h];
      }

      for (int j = 0; j < raw_imu_rates_.length; ++j)
      {
         geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.raw_imu_rates_[j], raw_imu_rates_[j]);
      }

      for (int l = 0; l < raw_imu_deltas_.length; ++l)
      {
         geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.raw_imu_deltas_[l], raw_imu_deltas_[l]);
      }

      battery_charging_ = other.battery_charging_;

      battery_voltage_ = other.battery_voltage_;

      battery_current_ = other.battery_current_;

      remaining_battery_time_ = other.remaining_battery_time_;

      remaining_amp_hours_ = other.remaining_amp_hours_;

      remaining_charge_percentage_ = other.remaining_charge_percentage_;

      battery_cycle_count_ = other.battery_cycle_count_;

      pump_inlet_pressure_ = other.pump_inlet_pressure_;

      pump_supply_pressure_ = other.pump_supply_pressure_;

      air_sump_pressure_ = other.air_sump_pressure_;

      pump_supply_temperature_ = other.pump_supply_temperature_;

      pump_rpm_ = other.pump_rpm_;

      motor_temperature_ = other.motor_temperature_;

      motor_driver_temperature_ = other.motor_driver_temperature_;
   }

   public us.ihmc.idl.IDLSequence.Float getElectricJointTemperatures()
   {
      return electric_joint_temperatures_;
   }

   public us.ihmc.idl.IDLSequence.Float getElectricJointCurrents()
   {
      return electric_joint_currents_;
   }

   public us.ihmc.idl.IDLSequence.Float getElectricJointEnabledArray()
   {
      return electric_joint_enabled_array_;
   }

   public float[] getRawImuTimestamps()
   {
      return raw_imu_timestamps_;
   }

   public float[] getRawImuPacketCounts()
   {
      return raw_imu_packet_counts_;
   }

   public us.ihmc.euclid.tuple3D.Vector3D[] getRawImuRates()
   {
      return raw_imu_rates_;
   }

   public us.ihmc.euclid.tuple3D.Vector3D[] getRawImuDeltas()
   {
      return raw_imu_deltas_;
   }

   public boolean getBatteryCharging()
   {
      return battery_charging_;
   }

   public void setBatteryCharging(boolean battery_charging)
   {
      battery_charging_ = battery_charging;
   }

   public float getBatteryVoltage()
   {
      return battery_voltage_;
   }

   public void setBatteryVoltage(float battery_voltage)
   {
      battery_voltage_ = battery_voltage;
   }

   public float getBatteryCurrent()
   {
      return battery_current_;
   }

   public void setBatteryCurrent(float battery_current)
   {
      battery_current_ = battery_current;
   }

   public float getRemainingBatteryTime()
   {
      return remaining_battery_time_;
   }

   public void setRemainingBatteryTime(float remaining_battery_time)
   {
      remaining_battery_time_ = remaining_battery_time;
   }

   public float getRemainingAmpHours()
   {
      return remaining_amp_hours_;
   }

   public void setRemainingAmpHours(float remaining_amp_hours)
   {
      remaining_amp_hours_ = remaining_amp_hours;
   }

   public float getRemainingChargePercentage()
   {
      return remaining_charge_percentage_;
   }

   public void setRemainingChargePercentage(float remaining_charge_percentage)
   {
      remaining_charge_percentage_ = remaining_charge_percentage;
   }

   public long getBatteryCycleCount()
   {
      return battery_cycle_count_;
   }

   public void setBatteryCycleCount(long battery_cycle_count)
   {
      battery_cycle_count_ = battery_cycle_count;
   }

   public float getPumpInletPressure()
   {
      return pump_inlet_pressure_;
   }

   public void setPumpInletPressure(float pump_inlet_pressure)
   {
      pump_inlet_pressure_ = pump_inlet_pressure;
   }

   public float getPumpSupplyPressure()
   {
      return pump_supply_pressure_;
   }

   public void setPumpSupplyPressure(float pump_supply_pressure)
   {
      pump_supply_pressure_ = pump_supply_pressure;
   }

   public float getAirSumpPressure()
   {
      return air_sump_pressure_;
   }

   public void setAirSumpPressure(float air_sump_pressure)
   {
      air_sump_pressure_ = air_sump_pressure;
   }

   public float getPumpSupplyTemperature()
   {
      return pump_supply_temperature_;
   }

   public void setPumpSupplyTemperature(float pump_supply_temperature)
   {
      pump_supply_temperature_ = pump_supply_temperature;
   }

   public float getPumpRpm()
   {
      return pump_rpm_;
   }

   public void setPumpRpm(float pump_rpm)
   {
      pump_rpm_ = pump_rpm;
   }

   public float getMotorTemperature()
   {
      return motor_temperature_;
   }

   public void setMotorTemperature(float motor_temperature)
   {
      motor_temperature_ = motor_temperature;
   }

   public float getMotorDriverTemperature()
   {
      return motor_driver_temperature_;
   }

   public void setMotorDriverTemperature(float motor_driver_temperature)
   {
      motor_driver_temperature_ = motor_driver_temperature;
   }

   @Override
   public boolean epsilonEquals(AtlasAuxiliaryRobotData other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.electric_joint_temperatures_, other.electric_joint_temperatures_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.electric_joint_currents_, other.electric_joint_currents_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.electric_joint_enabled_array_, other.electric_joint_enabled_array_, epsilon))
         return false;

      for (int n = 0; n < raw_imu_timestamps_.length; ++n)
      {
         if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.raw_imu_timestamps_[n], other.raw_imu_timestamps_[n], epsilon))
            return false;
      }

      for (int p = 0; p < raw_imu_packet_counts_.length; ++p)
      {
         if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.raw_imu_packet_counts_[p], other.raw_imu_packet_counts_[p], epsilon))
            return false;
      }

      for (int r = 0; r < raw_imu_rates_.length; ++r)
      {
         if (!this.raw_imu_rates_[r].epsilonEquals(other.raw_imu_rates_[r], epsilon))
            return false;
      }

      for (int t = 0; t < raw_imu_deltas_.length; ++t)
      {
         if (!this.raw_imu_deltas_[t].epsilonEquals(other.raw_imu_deltas_[t], epsilon))
            return false;
      }

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.battery_charging_, other.battery_charging_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.battery_voltage_, other.battery_voltage_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.battery_current_, other.battery_current_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.remaining_battery_time_, other.remaining_battery_time_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.remaining_amp_hours_, other.remaining_amp_hours_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.remaining_charge_percentage_, other.remaining_charge_percentage_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.battery_cycle_count_, other.battery_cycle_count_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pump_inlet_pressure_, other.pump_inlet_pressure_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pump_supply_pressure_, other.pump_supply_pressure_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.air_sump_pressure_, other.air_sump_pressure_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pump_supply_temperature_, other.pump_supply_temperature_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pump_rpm_, other.pump_rpm_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.motor_temperature_, other.motor_temperature_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.motor_driver_temperature_, other.motor_driver_temperature_, epsilon))
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
      if (!(other instanceof AtlasAuxiliaryRobotData))
         return false;

      AtlasAuxiliaryRobotData otherMyClass = (AtlasAuxiliaryRobotData) other;

      if (!this.electric_joint_temperatures_.equals(otherMyClass.electric_joint_temperatures_))
         return false;

      if (!this.electric_joint_currents_.equals(otherMyClass.electric_joint_currents_))
         return false;

      if (!this.electric_joint_enabled_array_.equals(otherMyClass.electric_joint_enabled_array_))
         return false;

      for (int v = 0; v < raw_imu_timestamps_.length; ++v)
      {
         if (this.raw_imu_timestamps_[v] != otherMyClass.raw_imu_timestamps_[v])
            return false;
      }
      for (int x = 0; x < raw_imu_packet_counts_.length; ++x)
      {
         if (this.raw_imu_packet_counts_[x] != otherMyClass.raw_imu_packet_counts_[x])
            return false;
      }
      for (int z = 0; z < raw_imu_rates_.length; ++z)
      {
         if (!this.raw_imu_rates_[z].equals(otherMyClass.raw_imu_rates_[z]))
            return false;
      }
      for (int | =0; | <raw_imu_deltas_.length;
      ++ |)
      {
         if (!this.raw_imu_deltas_[ |].equals(otherMyClass.raw_imu_deltas_[ |]))return false;
      } if (this.battery_charging_ != otherMyClass.battery_charging_)
      return false;

      if (this.battery_voltage_ != otherMyClass.battery_voltage_)
         return false;

      if (this.battery_current_ != otherMyClass.battery_current_)
         return false;

      if (this.remaining_battery_time_ != otherMyClass.remaining_battery_time_)
         return false;

      if (this.remaining_amp_hours_ != otherMyClass.remaining_amp_hours_)
         return false;

      if (this.remaining_charge_percentage_ != otherMyClass.remaining_charge_percentage_)
         return false;

      if (this.battery_cycle_count_ != otherMyClass.battery_cycle_count_)
         return false;

      if (this.pump_inlet_pressure_ != otherMyClass.pump_inlet_pressure_)
         return false;

      if (this.pump_supply_pressure_ != otherMyClass.pump_supply_pressure_)
         return false;

      if (this.air_sump_pressure_ != otherMyClass.air_sump_pressure_)
         return false;

      if (this.pump_supply_temperature_ != otherMyClass.pump_supply_temperature_)
         return false;

      if (this.pump_rpm_ != otherMyClass.pump_rpm_)
         return false;

      if (this.motor_temperature_ != otherMyClass.motor_temperature_)
         return false;

      if (this.motor_driver_temperature_ != otherMyClass.motor_driver_temperature_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasAuxiliaryRobotData {");
      builder.append("electric_joint_temperatures=");
      builder.append(this.electric_joint_temperatures_);

      builder.append(", ");
      builder.append("electric_joint_currents=");
      builder.append(this.electric_joint_currents_);

      builder.append(", ");
      builder.append("electric_joint_enabled_array=");
      builder.append(this.electric_joint_enabled_array_);

      builder.append(", ");
      builder.append("raw_imu_timestamps=");
      builder.append(java.util.Arrays.toString(this.raw_imu_timestamps_));

      builder.append(", ");
      builder.append("raw_imu_packet_counts=");
      builder.append(java.util.Arrays.toString(this.raw_imu_packet_counts_));

      builder.append(", ");
      builder.append("raw_imu_rates=");
      builder.append(java.util.Arrays.toString(this.raw_imu_rates_));

      builder.append(", ");
      builder.append("raw_imu_deltas=");
      builder.append(java.util.Arrays.toString(this.raw_imu_deltas_));

      builder.append(", ");
      builder.append("battery_charging=");
      builder.append(this.battery_charging_);

      builder.append(", ");
      builder.append("battery_voltage=");
      builder.append(this.battery_voltage_);

      builder.append(", ");
      builder.append("battery_current=");
      builder.append(this.battery_current_);

      builder.append(", ");
      builder.append("remaining_battery_time=");
      builder.append(this.remaining_battery_time_);

      builder.append(", ");
      builder.append("remaining_amp_hours=");
      builder.append(this.remaining_amp_hours_);

      builder.append(", ");
      builder.append("remaining_charge_percentage=");
      builder.append(this.remaining_charge_percentage_);

      builder.append(", ");
      builder.append("battery_cycle_count=");
      builder.append(this.battery_cycle_count_);

      builder.append(", ");
      builder.append("pump_inlet_pressure=");
      builder.append(this.pump_inlet_pressure_);

      builder.append(", ");
      builder.append("pump_supply_pressure=");
      builder.append(this.pump_supply_pressure_);

      builder.append(", ");
      builder.append("air_sump_pressure=");
      builder.append(this.air_sump_pressure_);

      builder.append(", ");
      builder.append("pump_supply_temperature=");
      builder.append(this.pump_supply_temperature_);

      builder.append(", ");
      builder.append("pump_rpm=");
      builder.append(this.pump_rpm_);

      builder.append(", ");
      builder.append("motor_temperature=");
      builder.append(this.motor_temperature_);

      builder.append(", ");
      builder.append("motor_driver_temperature=");
      builder.append(this.motor_driver_temperature_);

      builder.append("}");
      return builder.toString();
   }
}