package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.robotSide.RobotSide;


public class ManualHandControlPacket extends Packet<ManualHandControlPacket>
{
   public static final int VELOCITY = 0;
   public static final int POSITION = 1;

   public static enum HandType {IROBOT, ROBOTIQ}

   ;

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

   // joint values should be in the range [0,1]
   public ManualHandControlPacket(RobotSide robotSide, double index, double middle, double thumb, double spread, int type)
   {
      this.robotSide = robotSide;
      this.index = index;
      this.middle = middle;
      this.thumb = thumb;
      this.spread = spread;
      this.controlType = type;
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public String toString()
   {
      return robotSide.getSideNameFirstLetter() + " finger jodoubles";
   }

   public boolean epsilonEquals(ManualHandControlPacket other, double epsilon)
   {
      boolean ret = this.getRobotSide().equals(other.getRobotSide());

      ret &= Math.abs(this.getIndex() - other.getIndex()) < epsilon;
      ret &= Math.abs(this.getMiddle() - other.getMiddle()) < epsilon;
      ret &= Math.abs(this.getThumb() - other.getThumb()) < epsilon;
      ret &= Math.abs(this.getSpread() - other.getSpread()) < epsilon;
      ret &= Math.abs(this.getControlType() - other.getControlType()) < epsilon;

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

   public ManualHandControlPacket(Random random)
   {
      this.robotSide = RobotSide.generateRandomRobotSide(random);
      double[] angles = RandomNumbers.nextDoubleArray(random, 4, 0, 1);


      this.index = angles[0];
      this.middle = angles[1];
      this.thumb = angles[2];
      this.spread = angles[3];
      this.controlType = 0;

   }
}
