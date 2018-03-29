package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;

public class MultisenseParameterPacket extends Packet<MultisenseParameterPacket>
{
   public boolean initialize;
   public double gain;
   public double motorSpeed;
   public boolean ledEnable;
   public boolean flashEnable;
   public double dutyCycle;
   public boolean autoExposure;
   public boolean autoWhiteBalance;

   public MultisenseParameterPacket()
   {
   }

   @Override
   public void set(MultisenseParameterPacket other)
   {
      this.initialize = other.initialize;
      this.gain = other.gain;
      this.flashEnable = other.flashEnable;
      this.motorSpeed = other.motorSpeed;
      this.ledEnable = other.ledEnable;
      this.dutyCycle = other.dutyCycle;
      this.autoExposure = other.autoExposure;
      this.autoWhiteBalance = other.autoWhiteBalance;
      setPacketInformation(other);
   }

   public boolean getInitialize()
   {
      return initialize;
   }

   public boolean getAutoExposure()
   {
      return autoExposure;
   }

   public double getGain()
   {
      return gain;
   }

   public void setGain(double gain)
   {
      this.gain = gain;
   }

   public double getMotorSpeed()
   {
      return motorSpeed;
   }

   public void setMotorSpeed(double motorSpeed)
   {
      this.motorSpeed = motorSpeed;
   }

   public boolean getLedEnable()
   {
      return ledEnable;
   }

   public void setLedEnable(boolean ledEnable)
   {
      this.ledEnable = ledEnable;
   }

   public boolean getFlashEnable()
   {
      return flashEnable;
   }

   public void setFlashEnable(boolean flashEnable)
   {
      this.flashEnable = flashEnable;
   }

   public double getDutyCycle()
   {
      return dutyCycle;
   }

   public boolean getAutoWhiteBalance()
   {
      return autoWhiteBalance;
   }

   public boolean equals(Object other)
   {
      if (other instanceof MultisenseParameterPacket)
      {
         MultisenseParameterPacket otherVideoPacket = (MultisenseParameterPacket) other;

         return epsilonEquals(otherVideoPacket, 0);
      }
      else
      {
         return false;
      }
   }

   @Override
   public boolean epsilonEquals(MultisenseParameterPacket other, double epsilon)
   {
      if (initialize)
         return other.getInitialize();
      double roundedGain = Math.round(getGain() * 100) / 100.0;
      double roundedSpeed = Math.round(getMotorSpeed() * 1000) / 1000.0;
      double roundedDutyCycle = Math.round(getDutyCycle() * 10) / 10.0;
      System.out.println("gain1: " + roundedGain + " gain 2 " + other.getGain());
      System.out.println("speed1: " + roundedSpeed + " speed 2 " + other.getMotorSpeed());
      System.out.println("led1: " + getLedEnable() + " led 2 " + other.getLedEnable());
      System.out.println("flash1: " + getFlashEnable() + " flash 2 " + other.getFlashEnable());
      System.out.println("dutyCycle 1 : " + roundedDutyCycle + " dutyCycle 2 " + other.getDutyCycle());
      System.out.println("auto exposure 1: " + getAutoExposure() + " auto exposure 2 " + other.getAutoExposure());
      System.out.println("auto white balance 1: " + getAutoWhiteBalance() + " auto white balance 2 " + other.getAutoWhiteBalance());

      return (other.getInitialize() == getInitialize()) && (other.getGain() == roundedGain) && (other.getMotorSpeed() == roundedSpeed)
            && (other.getDutyCycle() == roundedDutyCycle) && (other.getFlashEnable() == getFlashEnable()) && (other.getLedEnable() == getLedEnable())
            && (other.getAutoExposure() == getAutoExposure()) && (other.getAutoWhiteBalance() == getAutoWhiteBalance());
   }
}
