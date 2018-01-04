package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import java.util.Arrays;
import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandJointAnglePacket extends Packet<HandJointAnglePacket>
{
   public RobotSide robotSide;
   public double[] jointAngles;
   public boolean connected;
   public boolean calibrated;

   public HandJointAnglePacket()
   {
      // Empty constructor for deserialization
   }

   public HandJointAnglePacket(RobotSide robotSide, boolean connected, boolean calibrated, double[] jointAngles)
   {
      setAll(robotSide, connected, calibrated, jointAngles);
   }

   public void setAll(RobotSide robotSide, boolean connected, boolean calibrated, double[] jointAngles)
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

   public void set(HandJointAnglePacket other)
   {
      setAll(other.robotSide, other.connected, other.calibrated, other.jointAngles);
   }

   public double getJointAngle(HandJointName jointName)
   {
      int index = jointName.getIndex(robotSide);
      if (index == -1)
      {
         return 0;
      }

      return jointAngles[index];
   }

   public RobotSide getRobotSide()
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
      boolean ret = getRobotSide().equals(other.getRobotSide());

      ret &= other.jointAngles.length == jointAngles.length;
      for (int i = 0; i < jointAngles.length; i++)
      {
         ret &= Math.abs(jointAngles[i] - other.jointAngles[i]) < epsilon;
      }

      ret &= connected == other.connected;

      ret &= calibrated == other.calibrated;

      return ret;
   }

   public HandJointAnglePacket(Random random)
   {
      double limit = Math.PI;

      double[] joints = new double[8];

      for (int i = 0; i < joints.length; i++)
      {
         joints[i] = -limit + random.nextDouble() * limit * 2;
      }

      RobotSide side = random.nextBoolean() ? RobotSide.LEFT : RobotSide.RIGHT;

      robotSide = side;
      jointAngles = joints;
      connected = random.nextBoolean();
      calibrated = random.nextBoolean();
   }
}
