package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * Message specific to Atlas, it is reported by the IHMC state estimator.
       */
public class AtlasAuxiliaryRobotData extends Packet<AtlasAuxiliaryRobotData> implements Settable<AtlasAuxiliaryRobotData>, EpsilonComparable<AtlasAuxiliaryRobotData>
{

   /**
            * Unique ID used to identify this message, should preferably be consecutively increasing.
            */
   public long sequence_id_;

   public us.ihmc.idl.IDLSequence.Float  electric_joint_temperatures_;

   public us.ihmc.idl.IDLSequence.Float  electric_joint_currents_;

   public us.ihmc.idl.IDLSequence.Boolean  electric_joint_enabled_array_;

   public float[] raw_imu_timestamps_;

   public float[] raw_imu_packet_counts_;

   public us.ihmc.euclid.tuple3D.Vector3D[] raw_imu_rates_;

   public us.ihmc.euclid.tuple3D.Vector3D[] raw_imu_deltas_;

   public boolean battery_charging_;

   public float battery_voltage_;

   public float battery_current_;

   public float remaining_battery_time_;

   public float remaining_amp_hours_;

   public float remaining_charge_percentage_;

   public long battery_cycle_count_;

   public float pump_inlet_pressure_;

   public float pump_supply_pressure_;

   public float air_sump_pressure_;

   public float pump_supply_temperature_;

   public float pump_rpm_;

   public float motor_temperature_;

   public float motor_driver_temperature_;

   public AtlasAuxiliaryRobotData()
   {


      electric_joint_temperatures_ = new us.ihmc.idl.IDLSequence.Float (6, "type_5");


      electric_joint_currents_ = new us.ihmc.idl.IDLSequence.Float (6, "type_5");


      electric_joint_enabled_array_ = new us.ihmc.idl.IDLSequence.Boolean (6, "type_7");


      raw_imu_timestamps_ = new float[15];


      raw_imu_packet_counts_ = new float[15];


      raw_imu_rates_ = new us.ihmc.euclid.tuple3D.Vector3D[15];

      for(int i1 = 0; i1 < raw_imu_rates_.length; ++i1)
      {
          raw_imu_rates_[i1] = new us.ihmc.euclid.tuple3D.Vector3D();
      }

      raw_imu_deltas_ = new us.ihmc.euclid.tuple3D.Vector3D[15];

      for(int i3 = 0; i3 < raw_imu_deltas_.length; ++i3)
      {
          raw_imu_deltas_[i3] = new us.ihmc.euclid.tuple3D.Vector3D();
      }















   }

   public AtlasAuxiliaryRobotData(AtlasAuxiliaryRobotData other)
   {
      this();
      set(other);
   }

   public void set(AtlasAuxiliaryRobotData other)
   {

      sequence_id_ = other.sequence_id_;


      electric_joint_temperatures_.set(other.electric_joint_temperatures_);

      electric_joint_currents_.set(other.electric_joint_currents_);

      electric_joint_enabled_array_.set(other.electric_joint_enabled_array_);

      for(int i5 = 0; i5 < raw_imu_timestamps_.length; ++i5)
      {
            raw_imu_timestamps_[i5] = other.raw_imu_timestamps_[i5];

      }


      for(int i7 = 0; i7 < raw_imu_packet_counts_.length; ++i7)
      {
            raw_imu_packet_counts_[i7] = other.raw_imu_packet_counts_[i7];

      }


      for(int i9 = 0; i9 < raw_imu_rates_.length; ++i9)
      {
            geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.raw_imu_rates_[i9], raw_imu_rates_[i9]);}


      for(int i11 = 0; i11 < raw_imu_deltas_.length; ++i11)
      {
            geometry_msgs.msg.dds.Vector3PubSubType.staticCopy(other.raw_imu_deltas_[i11], raw_imu_deltas_[i11]);}


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



   public us.ihmc.idl.IDLSequence.Float  getElectricJointTemperatures()
   {
      return electric_joint_temperatures_;
   }



   public us.ihmc.idl.IDLSequence.Float  getElectricJointCurrents()
   {
      return electric_joint_currents_;
   }



   public us.ihmc.idl.IDLSequence.Boolean  getElectricJointEnabledArray()
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


   public void setBatteryCharging(boolean battery_charging)
   {
      battery_charging_ = battery_charging;
   }
   public boolean getBatteryCharging()
   {
      return battery_charging_;
   }


   public void setBatteryVoltage(float battery_voltage)
   {
      battery_voltage_ = battery_voltage;
   }
   public float getBatteryVoltage()
   {
      return battery_voltage_;
   }


   public void setBatteryCurrent(float battery_current)
   {
      battery_current_ = battery_current;
   }
   public float getBatteryCurrent()
   {
      return battery_current_;
   }


   public void setRemainingBatteryTime(float remaining_battery_time)
   {
      remaining_battery_time_ = remaining_battery_time;
   }
   public float getRemainingBatteryTime()
   {
      return remaining_battery_time_;
   }


   public void setRemainingAmpHours(float remaining_amp_hours)
   {
      remaining_amp_hours_ = remaining_amp_hours;
   }
   public float getRemainingAmpHours()
   {
      return remaining_amp_hours_;
   }


   public void setRemainingChargePercentage(float remaining_charge_percentage)
   {
      remaining_charge_percentage_ = remaining_charge_percentage;
   }
   public float getRemainingChargePercentage()
   {
      return remaining_charge_percentage_;
   }


   public void setBatteryCycleCount(long battery_cycle_count)
   {
      battery_cycle_count_ = battery_cycle_count;
   }
   public long getBatteryCycleCount()
   {
      return battery_cycle_count_;
   }


   public void setPumpInletPressure(float pump_inlet_pressure)
   {
      pump_inlet_pressure_ = pump_inlet_pressure;
   }
   public float getPumpInletPressure()
   {
      return pump_inlet_pressure_;
   }


   public void setPumpSupplyPressure(float pump_supply_pressure)
   {
      pump_supply_pressure_ = pump_supply_pressure;
   }
   public float getPumpSupplyPressure()
   {
      return pump_supply_pressure_;
   }


   public void setAirSumpPressure(float air_sump_pressure)
   {
      air_sump_pressure_ = air_sump_pressure;
   }
   public float getAirSumpPressure()
   {
      return air_sump_pressure_;
   }


   public void setPumpSupplyTemperature(float pump_supply_temperature)
   {
      pump_supply_temperature_ = pump_supply_temperature;
   }
   public float getPumpSupplyTemperature()
   {
      return pump_supply_temperature_;
   }


   public void setPumpRpm(float pump_rpm)
   {
      pump_rpm_ = pump_rpm;
   }
   public float getPumpRpm()
   {
      return pump_rpm_;
   }


   public void setMotorTemperature(float motor_temperature)
   {
      motor_temperature_ = motor_temperature;
   }
   public float getMotorTemperature()
   {
      return motor_temperature_;
   }


   public void setMotorDriverTemperature(float motor_driver_temperature)
   {
      motor_driver_temperature_ = motor_driver_temperature;
   }
   public float getMotorDriverTemperature()
   {
      return motor_driver_temperature_;
   }


   public static Supplier<AtlasAuxiliaryRobotDataPubSubType> getPubSubType()
   {
      return AtlasAuxiliaryRobotDataPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return AtlasAuxiliaryRobotDataPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(AtlasAuxiliaryRobotData other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.sequence_id_, other.sequence_id_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.electric_joint_temperatures_, other.electric_joint_temperatures_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsFloatSequence(this.electric_joint_currents_, other.electric_joint_currents_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBooleanSequence(this.electric_joint_enabled_array_, other.electric_joint_enabled_array_, epsilon)) return false;


      for(int i13 = 0; i13 < raw_imu_timestamps_.length; ++i13)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.raw_imu_timestamps_[i13], other.raw_imu_timestamps_[i13], epsilon)) return false;
      }


      for(int i15 = 0; i15 < raw_imu_packet_counts_.length; ++i15)
      {
                if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.raw_imu_packet_counts_[i15], other.raw_imu_packet_counts_[i15], epsilon)) return false;
      }


      for(int i17 = 0; i17 < raw_imu_rates_.length; ++i17)
      {
              if (!this.raw_imu_rates_[i17].epsilonEquals(other.raw_imu_rates_[i17], epsilon)) return false;
      }


      for(int i19 = 0; i19 < raw_imu_deltas_.length; ++i19)
      {
              if (!this.raw_imu_deltas_[i19].epsilonEquals(other.raw_imu_deltas_[i19], epsilon)) return false;
      }


      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.battery_charging_, other.battery_charging_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.battery_voltage_, other.battery_voltage_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.battery_current_, other.battery_current_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.remaining_battery_time_, other.remaining_battery_time_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.remaining_amp_hours_, other.remaining_amp_hours_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.remaining_charge_percentage_, other.remaining_charge_percentage_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.battery_cycle_count_, other.battery_cycle_count_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pump_inlet_pressure_, other.pump_inlet_pressure_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pump_supply_pressure_, other.pump_supply_pressure_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.air_sump_pressure_, other.air_sump_pressure_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pump_supply_temperature_, other.pump_supply_temperature_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.pump_rpm_, other.pump_rpm_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.motor_temperature_, other.motor_temperature_, epsilon)) return false;


      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.motor_driver_temperature_, other.motor_driver_temperature_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof AtlasAuxiliaryRobotData)) return false;

      AtlasAuxiliaryRobotData otherMyClass = (AtlasAuxiliaryRobotData) other;


      if(this.sequence_id_ != otherMyClass.sequence_id_) return false;


      if (!this.electric_joint_temperatures_.equals(otherMyClass.electric_joint_temperatures_)) return false;

      if (!this.electric_joint_currents_.equals(otherMyClass.electric_joint_currents_)) return false;

      if (!this.electric_joint_enabled_array_.equals(otherMyClass.electric_joint_enabled_array_)) return false;

      for(int i21 = 0; i21 < raw_imu_timestamps_.length; ++i21)
      {
                if(this.raw_imu_timestamps_[i21] != otherMyClass.raw_imu_timestamps_[i21]) return false;

      }

      for(int i23 = 0; i23 < raw_imu_packet_counts_.length; ++i23)
      {
                if(this.raw_imu_packet_counts_[i23] != otherMyClass.raw_imu_packet_counts_[i23]) return false;

      }

      for(int i25 = 0; i25 < raw_imu_rates_.length; ++i25)
      {
                if (!this.raw_imu_rates_[i25].equals(otherMyClass.raw_imu_rates_[i25])) return false;
      }

      for(int i27 = 0; i27 < raw_imu_deltas_.length; ++i27)
      {
                if (!this.raw_imu_deltas_[i27].equals(otherMyClass.raw_imu_deltas_[i27])) return false;
      }

      if(this.battery_charging_ != otherMyClass.battery_charging_) return false;


      if(this.battery_voltage_ != otherMyClass.battery_voltage_) return false;


      if(this.battery_current_ != otherMyClass.battery_current_) return false;


      if(this.remaining_battery_time_ != otherMyClass.remaining_battery_time_) return false;


      if(this.remaining_amp_hours_ != otherMyClass.remaining_amp_hours_) return false;


      if(this.remaining_charge_percentage_ != otherMyClass.remaining_charge_percentage_) return false;


      if(this.battery_cycle_count_ != otherMyClass.battery_cycle_count_) return false;


      if(this.pump_inlet_pressure_ != otherMyClass.pump_inlet_pressure_) return false;


      if(this.pump_supply_pressure_ != otherMyClass.pump_supply_pressure_) return false;


      if(this.air_sump_pressure_ != otherMyClass.air_sump_pressure_) return false;


      if(this.pump_supply_temperature_ != otherMyClass.pump_supply_temperature_) return false;


      if(this.pump_rpm_ != otherMyClass.pump_rpm_) return false;


      if(this.motor_temperature_ != otherMyClass.motor_temperature_) return false;


      if(this.motor_driver_temperature_ != otherMyClass.motor_driver_temperature_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("AtlasAuxiliaryRobotData {");

      builder.append("sequence_id=");
      builder.append(this.sequence_id_);      builder.append(", ");

      builder.append("electric_joint_temperatures=");
      builder.append(this.electric_joint_temperatures_);      builder.append(", ");

      builder.append("electric_joint_currents=");
      builder.append(this.electric_joint_currents_);      builder.append(", ");

      builder.append("electric_joint_enabled_array=");
      builder.append(this.electric_joint_enabled_array_);      builder.append(", ");

      builder.append("raw_imu_timestamps=");
      builder.append(java.util.Arrays.toString(this.raw_imu_timestamps_));      builder.append(", ");

      builder.append("raw_imu_packet_counts=");
      builder.append(java.util.Arrays.toString(this.raw_imu_packet_counts_));      builder.append(", ");

      builder.append("raw_imu_rates=");
      builder.append(java.util.Arrays.toString(this.raw_imu_rates_));      builder.append(", ");

      builder.append("raw_imu_deltas=");
      builder.append(java.util.Arrays.toString(this.raw_imu_deltas_));      builder.append(", ");

      builder.append("battery_charging=");
      builder.append(this.battery_charging_);      builder.append(", ");

      builder.append("battery_voltage=");
      builder.append(this.battery_voltage_);      builder.append(", ");

      builder.append("battery_current=");
      builder.append(this.battery_current_);      builder.append(", ");

      builder.append("remaining_battery_time=");
      builder.append(this.remaining_battery_time_);      builder.append(", ");

      builder.append("remaining_amp_hours=");
      builder.append(this.remaining_amp_hours_);      builder.append(", ");

      builder.append("remaining_charge_percentage=");
      builder.append(this.remaining_charge_percentage_);      builder.append(", ");

      builder.append("battery_cycle_count=");
      builder.append(this.battery_cycle_count_);      builder.append(", ");

      builder.append("pump_inlet_pressure=");
      builder.append(this.pump_inlet_pressure_);      builder.append(", ");

      builder.append("pump_supply_pressure=");
      builder.append(this.pump_supply_pressure_);      builder.append(", ");

      builder.append("air_sump_pressure=");
      builder.append(this.air_sump_pressure_);      builder.append(", ");

      builder.append("pump_supply_temperature=");
      builder.append(this.pump_supply_temperature_);      builder.append(", ");

      builder.append("pump_rpm=");
      builder.append(this.pump_rpm_);      builder.append(", ");

      builder.append("motor_temperature=");
      builder.append(this.motor_temperature_);      builder.append(", ");

      builder.append("motor_driver_temperature=");
      builder.append(this.motor_driver_temperature_);
      builder.append("}");
      return builder.toString();
   }
}
