package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;

public class BlackFlyParameterPacket extends Packet<BlackFlyParameterPacket>
{
   private static final boolean DEBUG = false;

   public static final byte ROBOT_SIDE_LEFT = 0;
   public static final byte ROBOT_SIDE_RIGHT = 1;

   public boolean autoExposure;

   public boolean autoGain;

   public boolean autoShutter;

   public double exposure;

   public double frameRate;

   public boolean fromUI;

   public double gain;

   public double shutter;

   public byte robotSide;

   public BlackFlyParameterPacket()
   {

   }

   @Override
   public void set(BlackFlyParameterPacket other)
   {
      setPacketInformation(other);
      this.fromUI = other.fromUI;
      this.exposure = other.exposure;
      this.shutter = other.shutter;
      this.gain = other.gain;
      this.frameRate = other.frameRate;
      this.autoExposure = other.autoExposure;
      this.autoGain = other.autoGain;
      this.autoShutter = other.autoShutter;
      this.robotSide = other.robotSide;
   }

   @Override
   public boolean epsilonEquals(BlackFlyParameterPacket other, double epsilon)
   {
      if (fromUI)
         return other.isFromUI();
      double roundedGain = Math.round(getGain() * 100) / 100.0;
      double roundedExposure = Math.round(getExposure() * 100) / 100.0;
      double roundedFrameRate = Math.round(getFrameRate() * 100) / 100.0;
      double roundedShutter = Math.round(getShutter() * 100) / 100.0;

      if (DEBUG)
      {
         System.out.println("gain1: " + roundedGain + " gain 2 " + other.getGain());
         System.out.println("speed1: " + roundedExposure + " speed 2 " + other.getExposure());
         System.out.println("led1: " + roundedFrameRate + " led 2 " + other.getFrameRate());
         System.out.println("flash1: " + roundedShutter + " flash 2 " + other.getShutter());
      }

      return (other.isFromUI() == isFromUI()) && (other.getGain() == roundedGain) && (other.getExposure() == roundedExposure)
            && (other.getFrameRate() == roundedFrameRate) && (other.getShutter() == roundedShutter);
   }

   public boolean equals(Object other)
   {
      if (other instanceof BlackFlyParameterPacket)
      {
         return epsilonEquals((BlackFlyParameterPacket) other, 0);
      }
      else
      {
         return false;
      }
   }

   public double getExposure()
   {
      return exposure;
   }

   public double getFrameRate()
   {
      return frameRate;
   }

   public double getGain()
   {
      return gain;
   }

   public double getShutter()
   {
      return shutter;
   }

   public byte getSide()
   {
      return robotSide;
   }

   public boolean isAutoExposure()
   {
      return autoExposure;
   }

   public boolean isAutoGain()
   {
      return autoGain;
   }

   public boolean isAutoShutter()
   {
      return autoShutter;
   }

   public boolean isFromUI()
   {
      return fromUI;
   }

   public void setAutoExposure(boolean autoExposure)
   {
      this.autoExposure = autoExposure;
   }

   public void setAutoGain(boolean autoGain)
   {
      this.autoGain = autoGain;
   }

   public void setAutoShutter(boolean autoShutter)
   {
      this.autoShutter = autoShutter;
   }

   @Override
   public String toString()
   {
      return " source(fromUI?)" + fromUI + " " + RobotSide.fromByte(robotSide).name() + " gain " + gain + " fps " + frameRate + " shutter " + shutter
            + " exposure " + exposure + " autoExposure/autoGain/autoShutter " + autoExposure + autoGain + autoShutter;
   }
}
