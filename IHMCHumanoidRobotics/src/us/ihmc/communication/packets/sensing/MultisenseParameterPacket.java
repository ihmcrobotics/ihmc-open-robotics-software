package us.ihmc.communication.packets.sensing;

import java.util.Random;

import us.ihmc.communication.packets.Packet;


public class MultisenseParameterPacket extends Packet<MultisenseParameterPacket>
{
   public boolean intialize;
   public double gain;
   public double motorSpeed;
   public boolean ledEnable;
   public boolean flashEnable;
   public double dutyCycle;
   public boolean autoExposure;
   public boolean autoWhiteBalance;


   public MultisenseParameterPacket(boolean initialize, double gain, double motorSpeed, double dutyCycle, boolean ledEnable,
                                    boolean flashEnable, boolean autoExposure, boolean autoWhiteBalance)
   {
      this.intialize = initialize;
      this.gain = gain;
      this.flashEnable = flashEnable;
      this.motorSpeed = motorSpeed;
      this.ledEnable = ledEnable;
      this.dutyCycle = dutyCycle;
      this.autoExposure = autoExposure;
      this.autoWhiteBalance = autoWhiteBalance;

   }
   
   public MultisenseParameterPacket()
   {
      
   }


   public boolean isInitialize()
   {
      return intialize;
   }

   public boolean isAutoExposure()
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

   public boolean isLedEnable()
   {
      return ledEnable;
   }

   public void setLedEnable(boolean ledEnable)
   {
      this.ledEnable = ledEnable;
   }

   public boolean isFlashEnable()
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

   public boolean isAutoWhiteBalance()
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
      if (intialize)
         return other.isInitialize();
      double roundedGain = Math.round(getGain() * 100) / 100.0;
      double roundedSpeed = Math.round(getMotorSpeed() * 1000) / 1000.0;
      double roundedDutyCycle = Math.round(getDutyCycle() * 10) / 10.0;
      System.out.println("gain1: " + roundedGain + " gain 2 " + other.getGain());
      System.out.println("speed1: " + roundedSpeed + " speed 2 " + other.getMotorSpeed());
      System.out.println("led1: " + isLedEnable() + " led 2 " + other.isLedEnable());
      System.out.println("flash1: " + isFlashEnable() + " flash 2 " + other.isFlashEnable());
      System.out.println("dutyCycle 1 : " + roundedDutyCycle + " dutyCycle 2 " + other.getDutyCycle());
      System.out.println("auto exposure 1: " + isAutoExposure() + " auto exposure 2 " + other.isAutoExposure());
      System.out.println("auto white balance 1: " + isAutoWhiteBalance() + " auto white balance 2 " + other.isAutoWhiteBalance());

      return (other.isInitialize() == isInitialize()) && (other.getGain() == roundedGain) && (other.getMotorSpeed() == roundedSpeed)
             && (other.getDutyCycle() == roundedDutyCycle) && (other.isFlashEnable() == isFlashEnable())
             && (other.isLedEnable() == isLedEnable()) && (other.isAutoExposure() == isAutoExposure()) && (other.isAutoWhiteBalance() == isAutoWhiteBalance());
   }

   public MultisenseParameterPacket(Random random)
   {
      this(true, random.nextDouble(), random.nextDouble(), random.nextDouble(), true, true, true, true);
   }
}
