package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;

public class BlackFlyParameterPacket extends Packet<BlackFlyParameterPacket>
{
   private static final boolean DEBUG = false;

   public boolean autoExposure;

   public boolean autoGain;

   public boolean autoShutter;

   public double exposure;

   public double frameRate;

   public boolean fromUI;

   public double gain;

   public double shutter;

   public RobotSide side;

   public BlackFlyParameterPacket()
   {

   }

   public BlackFlyParameterPacket(boolean fromUI, double gain, double exposure, double frameRate, double shutter, boolean autoExPosure, boolean autoGain,
                                  boolean autoShutter, RobotSide side)
   {
      this.fromUI = fromUI;
      this.exposure = exposure;
      this.shutter = shutter;
      this.gain = gain;
      this.frameRate = frameRate;
      this.autoExposure = autoExPosure;
      this.autoGain = autoGain;
      this.autoShutter = autoShutter;
      this.side = side;
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

   public RobotSide getSide()
   {
      return side;
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
      return " source(fromUI?)" + fromUI + " " + side.name() + " gain " + gain + " fps " + frameRate + " shutter " + shutter + " exposure " + exposure
            + " autoExposure/autoGain/autoShutter " + autoExposure + autoGain + autoShutter;
   }
}
