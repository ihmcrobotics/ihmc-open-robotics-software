package controller_msgs.msg.dds;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class MultisenseParameterPacket implements Settable<MultisenseParameterPacket>, EpsilonComparable<MultisenseParameterPacket>
{
   private boolean initialize_;
   private double gain_;
   private double motor_speed_;
   private boolean led_enable_;
   private boolean flash_enable_;
   private double duty_cycle_;
   private boolean auto_exposure_;
   private boolean auto_white_balance_;

   public MultisenseParameterPacket()
   {

   }

   public MultisenseParameterPacket(MultisenseParameterPacket other)
   {
      set(other);
   }

   public void set(MultisenseParameterPacket other)
   {
      initialize_ = other.initialize_;

      gain_ = other.gain_;

      motor_speed_ = other.motor_speed_;

      led_enable_ = other.led_enable_;

      flash_enable_ = other.flash_enable_;

      duty_cycle_ = other.duty_cycle_;

      auto_exposure_ = other.auto_exposure_;

      auto_white_balance_ = other.auto_white_balance_;
   }

   public boolean getInitialize()
   {
      return initialize_;
   }

   public void setInitialize(boolean initialize)
   {
      initialize_ = initialize;
   }

   public double getGain()
   {
      return gain_;
   }

   public void setGain(double gain)
   {
      gain_ = gain;
   }

   public double getMotorSpeed()
   {
      return motor_speed_;
   }

   public void setMotorSpeed(double motor_speed)
   {
      motor_speed_ = motor_speed;
   }

   public boolean getLedEnable()
   {
      return led_enable_;
   }

   public void setLedEnable(boolean led_enable)
   {
      led_enable_ = led_enable;
   }

   public boolean getFlashEnable()
   {
      return flash_enable_;
   }

   public void setFlashEnable(boolean flash_enable)
   {
      flash_enable_ = flash_enable;
   }

   public double getDutyCycle()
   {
      return duty_cycle_;
   }

   public void setDutyCycle(double duty_cycle)
   {
      duty_cycle_ = duty_cycle;
   }

   public boolean getAutoExposure()
   {
      return auto_exposure_;
   }

   public void setAutoExposure(boolean auto_exposure)
   {
      auto_exposure_ = auto_exposure;
   }

   public boolean getAutoWhiteBalance()
   {
      return auto_white_balance_;
   }

   public void setAutoWhiteBalance(boolean auto_white_balance)
   {
      auto_white_balance_ = auto_white_balance;
   }

   @Override
   public boolean epsilonEquals(MultisenseParameterPacket other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.initialize_, other.initialize_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.gain_, other.gain_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.motor_speed_, other.motor_speed_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.led_enable_, other.led_enable_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.flash_enable_, other.flash_enable_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.duty_cycle_, other.duty_cycle_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.auto_exposure_, other.auto_exposure_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.auto_white_balance_, other.auto_white_balance_, epsilon))
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
      if (!(other instanceof MultisenseParameterPacket))
         return false;

      MultisenseParameterPacket otherMyClass = (MultisenseParameterPacket) other;

      if (this.initialize_ != otherMyClass.initialize_)
         return false;

      if (this.gain_ != otherMyClass.gain_)
         return false;

      if (this.motor_speed_ != otherMyClass.motor_speed_)
         return false;

      if (this.led_enable_ != otherMyClass.led_enable_)
         return false;

      if (this.flash_enable_ != otherMyClass.flash_enable_)
         return false;

      if (this.duty_cycle_ != otherMyClass.duty_cycle_)
         return false;

      if (this.auto_exposure_ != otherMyClass.auto_exposure_)
         return false;

      if (this.auto_white_balance_ != otherMyClass.auto_white_balance_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MultisenseParameterPacket {");
      builder.append("initialize=");
      builder.append(this.initialize_);

      builder.append(", ");
      builder.append("gain=");
      builder.append(this.gain_);

      builder.append(", ");
      builder.append("motor_speed=");
      builder.append(this.motor_speed_);

      builder.append(", ");
      builder.append("led_enable=");
      builder.append(this.led_enable_);

      builder.append(", ");
      builder.append("flash_enable=");
      builder.append(this.flash_enable_);

      builder.append(", ");
      builder.append("duty_cycle=");
      builder.append(this.duty_cycle_);

      builder.append(", ");
      builder.append("auto_exposure=");
      builder.append(this.auto_exposure_);

      builder.append(", ");
      builder.append("auto_white_balance=");
      builder.append(this.auto_white_balance_);

      builder.append("}");
      return builder.toString();
   }
}