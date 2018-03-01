package controller_msgs.msg.dds;

/**
 * Definition of the class "PumpState" defined in PumpState_.idl.
 *
 * This file was automatically generated from PumpState_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit PumpState_.idl instead.
 */
public class PumpState
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

   public float getPump_inlet_pressure()
   {
      return pump_inlet_pressure_;
   }

   public void setPump_inlet_pressure(float pump_inlet_pressure)
   {
      pump_inlet_pressure_ = pump_inlet_pressure;
   }

   public float getPump_supply_pressure()
   {
      return pump_supply_pressure_;
   }

   public void setPump_supply_pressure(float pump_supply_pressure)
   {
      pump_supply_pressure_ = pump_supply_pressure;
   }

   public float getAir_sump_pressure()
   {
      return air_sump_pressure_;
   }

   public void setAir_sump_pressure(float air_sump_pressure)
   {
      air_sump_pressure_ = air_sump_pressure;
   }

   public float getPump_supply_temperature()
   {
      return pump_supply_temperature_;
   }

   public void setPump_supply_temperature(float pump_supply_temperature)
   {
      pump_supply_temperature_ = pump_supply_temperature;
   }

   public float getPump_rpm()
   {
      return pump_rpm_;
   }

   public void setPump_rpm(float pump_rpm)
   {
      pump_rpm_ = pump_rpm;
   }

   public float getMotor_temperature()
   {
      return motor_temperature_;
   }

   public void setMotor_temperature(float motor_temperature)
   {
      motor_temperature_ = motor_temperature;
   }

   public float getMotor_driver_temperature()
   {
      return motor_driver_temperature_;
   }

   public void setMotor_driver_temperature(float motor_driver_temperature)
   {
      motor_driver_temperature_ = motor_driver_temperature;
   }

   @Override
   public boolean equals(java.lang.Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof PumpState))
         return false;
      PumpState otherMyClass = (PumpState) other;
      boolean returnedValue = true;

      returnedValue &= this.pump_inlet_pressure_ == otherMyClass.pump_inlet_pressure_;

      returnedValue &= this.pump_supply_pressure_ == otherMyClass.pump_supply_pressure_;

      returnedValue &= this.air_sump_pressure_ == otherMyClass.air_sump_pressure_;

      returnedValue &= this.pump_supply_temperature_ == otherMyClass.pump_supply_temperature_;

      returnedValue &= this.pump_rpm_ == otherMyClass.pump_rpm_;

      returnedValue &= this.motor_temperature_ == otherMyClass.motor_temperature_;

      returnedValue &= this.motor_driver_temperature_ == otherMyClass.motor_driver_temperature_;

      return returnedValue;
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