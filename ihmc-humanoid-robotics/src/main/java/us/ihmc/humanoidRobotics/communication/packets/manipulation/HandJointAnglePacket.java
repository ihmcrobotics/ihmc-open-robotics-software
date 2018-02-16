package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Arrays;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandJointAnglePacket extends Packet<HandJointAnglePacket>
{
   public byte robotSide;
   public double[] jointAngles;
   public boolean connected;
   public boolean calibrated;

   public HandJointAnglePacket()
   {
      // Empty constructor for deserialization
   }

   public void setAll(byte robotSide, boolean connected, boolean calibrated, double[] jointAngles)
   {
      this.robotSide = robotSide;
      if (this.jointAngles == null)
      {
         this.jointAngles = new double[jointAngles.length];
      }
      System.arraycopy(jointAngles, 0, this.jointAngles, 0, jointAngles.length);
      this.connected = connected;
      this.calibrated = calibrated;
   }

   @Override
   public void set(HandJointAnglePacket other)
   {
      setAll(other.robotSide, other.connected, other.calibrated, other.jointAngles);
      setPacketInformation(other);
   }

   public double getJointAngle(HandJointName jointName)
   {
      int index = jointName.getIndex(RobotSide.fromByte(robotSide));
      if (index == -1)
      {
         return 0;
      }

      return jointAngles[index];
   }

   public byte getRobotSide()
   {
      return robotSide;
   }

   public double[] getJointAngles()
   {
      return jointAngles;
   }

   public int getNumberOfJoints()
   {
      return jointAngles.length;
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
      return "HandJointAnglePacket [robotSide=" + robotSide + ", joints=" + Arrays.toString(jointAngles) + ", connected=" + connected + ", calibrated="
            + calibrated + "]";
   }

   @Override
   public boolean epsilonEquals(HandJointAnglePacket other, double epsilon)
   {
      boolean ret = robotSide == other.robotSide;

      ret &= other.jointAngles.length == jointAngles.length;
      for (int i = 0; i < jointAngles.length; i++)
      {
         ret &= Math.abs(jointAngles[i] - other.jointAngles[i]) < epsilon;
      }

      ret &= connected == other.connected;

      ret &= calibrated == other.calibrated;

      return ret;
   }
}
