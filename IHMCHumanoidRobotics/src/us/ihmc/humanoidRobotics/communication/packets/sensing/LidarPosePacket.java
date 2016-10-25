package us.ihmc.humanoidRobotics.communication.packets.sensing;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.communication.packets.Packet;

public class LidarPosePacket extends Packet<LidarPosePacket>
{
   public Point3d position;
   public Quat4d orientation;

   public LidarPosePacket()
   {
   }

   public LidarPosePacket(Point3d position, Quat4d orientation)
   {
      this.position = position;
      this.orientation = orientation;
   }

   public LidarPosePacket(LidarPosePacket other)
   {
      if (other.position != null)
         position = new Point3d(other.position);
      else
         position = null;
      if (other.orientation != null)
         orientation = new Quat4d(other.orientation);
      else
         orientation = null;
   }

   public Point3d getPosition()
   {
      return position;
   }

   public Quat4d getOrientation()
   {
      return orientation;
   }

   @Override
   public boolean epsilonEquals(LidarPosePacket other, double epsilon)
   {
      try
      {
         boolean arePositionsEqual = position.epsilonEquals(other.position, epsilon);
         boolean areOrientationsEqual = orientation.epsilonEquals(other.orientation, epsilon);
         return arePositionsEqual && areOrientationsEqual;
      }
      catch (NullPointerException e)
      {
         return false;
      }
   }
}
