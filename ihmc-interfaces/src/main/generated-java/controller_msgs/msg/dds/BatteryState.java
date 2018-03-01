package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * Definition of the class "BatteryState" defined in BatteryState_.idl.
 *
 * This file was automatically generated from BatteryState_.idl by us.ihmc.idl.generator.IDLGenerator.
 * Do not update this file directly, edit BatteryState_.idl instead.
 */
public class BatteryState implements Settable<BatteryState>, EpsilonComparable<BatteryState>
{
   private boolean battery_charging_;
   private float battery_voltage_;
   private float battery_current_;
   private float remaining_battery_time_;
   private float remaining_amp_hours_;
   private float remaining_charge_percentage_;
   private long battery_cycle_count_;

   public BatteryState()
   {

   }

   public BatteryState(BatteryState other)
   {
      set(other);
   }

   public void set(BatteryState other)
   {
      battery_charging_ = other.battery_charging_;

      battery_voltage_ = other.battery_voltage_;

      battery_current_ = other.battery_current_;

      remaining_battery_time_ = other.remaining_battery_time_;

      remaining_amp_hours_ = other.remaining_amp_hours_;

      remaining_charge_percentage_ = other.remaining_charge_percentage_;

      battery_cycle_count_ = other.battery_cycle_count_;
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

   @Override
   public boolean epsilonEquals(BatteryState other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

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

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof BatteryState))
         return false;

      BatteryState otherMyClass = (BatteryState) other;

      if (this.battery_charging_ != otherMyClass.battery_charging_)
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

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("BatteryState {");
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

      builder.append("}");
      return builder.toString();
   }
}