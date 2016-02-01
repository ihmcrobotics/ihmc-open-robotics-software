package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Arrays;
import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandJointAnglePacket extends Packet<HandJointAnglePacket>
{
   public RobotSide robotSide;
   public double[] indexJoints;
   public double[] middleJoints;
   public double[] thumbJoints;
   public boolean connected;
   public boolean calibrated;

   public HandJointAnglePacket()
   {
      // Empty constructor for deserialization
   }

   public HandJointAnglePacket(RobotSide robotSide, boolean connected, boolean calibrated, double[] indexJoints, double[] middleJoints, double[] thumbJoints)
   {
      this.robotSide = robotSide;
      this.indexJoints = indexJoints;
      this.middleJoints = middleJoints;
      this.thumbJoints = thumbJoints;
      this.connected = connected;
      this.calibrated = calibrated;
   }

   public void setAll(RobotSide robotSide, boolean connected, boolean calibrated, double[] indexJoints, double[] middleJoints, double[] thumbJoints)
   {
      this.robotSide = robotSide;
      if(this.indexJoints == null)
      {
         this.indexJoints = new double[indexJoints.length];
      }
      if(this.middleJoints == null)
      {
         this.middleJoints = new double[middleJoints.length];
      }
      if(this.thumbJoints == null)
      {
         this.thumbJoints = new double[thumbJoints.length];
      }
      System.arraycopy(indexJoints, 0, this.indexJoints, 0, indexJoints.length);
      System.arraycopy(middleJoints, 0, this.middleJoints, 0, middleJoints.length);
      System.arraycopy(thumbJoints, 0, this.thumbJoints, 0, thumbJoints.length);
      this.connected = connected;
      this.calibrated = calibrated;
   }

   public double getJointAngle(HandJointName jointName)
   {
      double[] source;
      switch (jointName.getFinger(robotSide))
      {
         case INDEX :
            source = indexJoints;

            break;

         case MIDDLE :
            source = middleJoints;

            break;

         default :
            source = thumbJoints;
      }

      int index = jointName.getHandJointAngleIndex();
      if (index == -1)
      {
         return 0;
      }

      return source[index];
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public double[] getIndexJoints()
   {
      return indexJoints;
   }

   public int getNumberOfIndexJoints()
   {
      return indexJoints.length;
   }

   public int getNumberOfMiddleJoints()
   {
      return middleJoints.length;
   }

   public int getNumberOfThumbJoints()
   {
      return thumbJoints.length;
   }

   public double[] getMiddleJoints()
   {
      return middleJoints;
   }

   public double[] getThumbJoints()
   {
      return thumbJoints;
   }

   public boolean isHandConnected()
   {
      return connected;
   }

   public boolean isCalibrated()
   {
      return calibrated;
   }

   @Override
   public String toString()
   {
      return "HandJointAnglePacket [robotSide=" + robotSide + ", indexJoints=" + Arrays.toString(indexJoints) + ", middleJoints="
            + Arrays.toString(middleJoints) + ", thumbJoints=" + Arrays.toString(thumbJoints) + ", connected=" + connected + ", calibrated=" + calibrated + "]";
   }

   public boolean epsilonEquals(HandJointAnglePacket other, double epsilon)
   {
      boolean ret = this.getRobotSide().equals(other.getRobotSide());

      ret &= other.indexJoints.length == indexJoints.length;
      for (int i = 0; i < indexJoints.length; i++)
      {
         ret &= Math.abs(this.getIndexJoints()[i] - other.getIndexJoints()[i]) < epsilon;
      }

      ret &= other.middleJoints.length == middleJoints.length;
      for (int i = 0; i < middleJoints.length; i++)
      {
         ret &= Math.abs(this.getMiddleJoints()[i] - other.getMiddleJoints()[i]) < epsilon;
      }

      ret &= other.thumbJoints.length == thumbJoints.length;
      for (int i = 0; i < thumbJoints.length; i++)
      {
         ret &= Math.abs(this.getThumbJoints()[i] - other.getThumbJoints()[i]) < epsilon;
      }

      ret &= this.connected == other.connected;
      
      ret &= this.calibrated == other.calibrated;

      return ret;
   }

   public HandJointAnglePacket(Random random)
   {
      int indexJointArraySize = 3;
      int middleJointArraySize = 3;
      int thumbJointArraySize = 2;
      double limit = Math.PI;

      double[][] fingers = new double[3][];
      fingers[0] = new double[indexJointArraySize];
      fingers[1] = new double[middleJointArraySize];
      fingers[2] = new double[thumbJointArraySize];

      for (int i = 0; i < fingers.length; i++)
      {
         for (int j = 0; j < fingers[i].length; j++)
         {
            fingers[i][j] = -limit + (random.nextDouble() * limit * 2);
         }
      }

      RobotSide side = (random.nextBoolean()) ? RobotSide.LEFT : RobotSide.RIGHT;

      this.robotSide = side;
      this.indexJoints = fingers[0];
      this.middleJoints = fingers[1];
      this.thumbJoints = fingers[2];
      this.connected = random.nextBoolean();
      this.calibrated = random.nextBoolean();
   }
}
