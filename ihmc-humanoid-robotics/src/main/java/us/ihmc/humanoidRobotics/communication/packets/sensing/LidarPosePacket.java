package us.ihmc.humanoidRobotics.communication.packets.sensing;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class LidarPosePacket extends Packet<LidarPosePacket>
{
   public Point3D position;
   public Quaternion orientation;

   public LidarPosePacket()
   {
   }

   public LidarPosePacket(Point3D position, Quaternion orientation)
   {
      this.position = position;
      this.orientation = orientation;
   }

   public LidarPosePacket(LidarPosePacket other)
   {
      if (other.position != null)
         position = new Point3D(other.position);
      else
         position = null;
      if (other.orientation != null)
         orientation = new Quaternion(other.orientation);
      else
         orientation = null;
   }

   public Point3D getPosition()
   {
      return position;
   }

   public Quaternion getOrientation()
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
