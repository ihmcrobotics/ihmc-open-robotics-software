package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class HandJointAnglePacket extends Packet<HandJointAnglePacket>
{
   public static final byte ROBOT_SIDE_LEFT = 0;
   public static final byte ROBOT_SIDE_RIGHT = 1;

   public byte robotSide;
   public TDoubleArrayList jointAngles = new TDoubleArrayList();
   public boolean connected;
   public boolean calibrated;

   public HandJointAnglePacket()
   {
      // Empty constructor for deserialization
   }

   public void setAll(byte robotSide, boolean connected, boolean calibrated, TDoubleArrayList jointAngles)
   {
      this.robotSide = robotSide;
      MessageTools.copyData(jointAngles, this.jointAngles);
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

      return jointAngles.get(index);
   }

   public byte getRobotSide()
   {
      return robotSide;
   }

   public TDoubleArrayList getJointAngles()
   {
      return jointAngles;
   }

   public int getNumberOfJoints()
   {
      return jointAngles.size();
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
      return "HandJointAnglePacket [robotSide=" + robotSide + ", joints=" + jointAngles + ", connected=" + connected + ", calibrated="
            + calibrated + "]";
   }

   @Override
   public boolean epsilonEquals(HandJointAnglePacket other, double epsilon)
   {
      boolean ret = robotSide == other.robotSide;

      ret &= other.jointAngles.size() == jointAngles.size();
      for (int i = 0; i < jointAngles.size(); i++)
      {
         ret &= Math.abs(jointAngles.get(i) - other.jointAngles.get(i)) < epsilon;
      }

      ret &= connected == other.connected;

      ret &= calibrated == other.calibrated;

      return ret;
   }
}
