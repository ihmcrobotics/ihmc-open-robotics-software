package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;

public class ManualHandControlPacket extends Packet<ManualHandControlPacket>
{
   public static final int VELOCITY = 0;
   public static final int POSITION = 1;

   public static enum HandType
   {
      IROBOT, ROBOTIQ
   }

   public RobotSide robotSide;
   public double index;
   public double middle;
   public double thumb;
   public double spread;
   public int controlType;

   public ManualHandControlPacket()
   {
      // Empty constructor for deserialization
   }

   @Override
   public void set(ManualHandControlPacket other)
   {
      robotSide = other.robotSide;
      index = other.index;
      middle = other.middle;
      thumb = other.thumb;
      spread = other.spread;
      controlType = other.controlType;
      setPacketInformation(other);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   @Override
   public String toString()
   {
      return robotSide.getSideNameFirstLetter() + " finger jodoubles";
   }

   @Override
   public boolean epsilonEquals(ManualHandControlPacket other, double epsilon)
   {
      boolean ret = getRobotSide().equals(other.getRobotSide());

      ret &= Math.abs(getIndex() - other.getIndex()) < epsilon;
      ret &= Math.abs(getMiddle() - other.getMiddle()) < epsilon;
      ret &= Math.abs(getThumb() - other.getThumb()) < epsilon;
      ret &= Math.abs(getSpread() - other.getSpread()) < epsilon;
      ret &= Math.abs(getControlType() - other.getControlType()) < epsilon;

      return ret;
   }

   public double[] getCommands(HandType hand)
   {
      if (hand == HandType.IROBOT)
         return new double[] {index, middle, thumb, 0, spread};
      else if (hand == HandType.ROBOTIQ)
         return new double[] {thumb, index, middle, spread};
      else
         return new double[] {index, middle, thumb, spread};
   }

   public double getIndex()
   {
      return index;
   }

   public double getMiddle()
   {
      return middle;
   }

   public double getThumb()
   {
      return thumb;
   }

   public double getSpread()
   {
      return spread;
   }

   public int getControlType()
   {
      return controlType;
   }

   public RobotSide getSide()
   {
      return robotSide;
   }
}
