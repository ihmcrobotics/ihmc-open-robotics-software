package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class UIPositionCheckerPacket extends Packet<UIPositionCheckerPacket>
{
   public Point3D position = new Point3D();
   public Quaternion orientation = new Quaternion();

   public UIPositionCheckerPacket()
   {

   }

   @Override
   public void set(UIPositionCheckerPacket other)
   {
      position = new Point3D(other.position);
      orientation = new Quaternion(other.orientation);
      setPacketInformation(other);
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
   public boolean epsilonEquals(UIPositionCheckerPacket other, double epsilon)
   {
      if (orientation != null)
         return (position.epsilonEquals(other.getPosition(), (float) epsilon)) && orientation.epsilonEquals(other.getOrientation(), epsilon);
      return position.epsilonEquals(other.getPosition(), (float) epsilon);
   }
}