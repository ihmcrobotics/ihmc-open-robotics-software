package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Definition of the class "PumpState" defined in PumpState_.idl.
 *
 * This file was automatically generated from PumpState_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit PumpState_.idl instead.
 */
public class PumpState implements Settable<PumpState>, EpsilonComparable<PumpState>
{
   private float pump_inlet_pressure_;
   private float pump_supply_pressure_;
   private float air_sump_pressure_;
   private float pump_supply_temperature_;
   private float pump_rpm_;
   private float motor_temperature_;
   private float motor_driver_temperature_;

   public PumpState()
   {

   }

   public PumpState(PumpState other)
   {
      set(other);
   }

   public void set(PumpState other)
   {
      pump_inlet_pressure_ = other.pump_inlet_pressure_;

      pump_supply_pressure_ = other.pump_supply_pressure_;

      air_sump_pressure_ = other.air_sump_pressure_;

      pump_supply_temperature_ = other.pump_supply_temperature_;

      pump_rpm_ = other.pump_rpm_;

      motor_temperature_ = other.motor_temperature_;

      motor_driver_temperature_ = other.motor_driver_temperature_;
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
   public boolean epsilonEquals(PumpState other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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
      if (!(other instanceof PumpState))
         return false;

      PumpState otherMyClass = (PumpState) other;

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

      builder.append("PumpState {");
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