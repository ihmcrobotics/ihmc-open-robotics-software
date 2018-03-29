package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;

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

   @Override
   public void set(HandJointAnglePacket other)
   {
      robotSide = other.robotSide;
      MessageTools.copyData(other.jointAngles, jointAngles);
      connected = other.connected;
      calibrated = other.calibrated;
      
      setPacketInformation(other);
   }

   public byte getRobotSide()
   {
      return robotSide;
   }

   public TDoubleArrayList getJointAngles()
   {
      return jointAngles;
   }

   public boolean getConnected()
   {
      return connected;
   }

   public boolean getCalibrated()
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
