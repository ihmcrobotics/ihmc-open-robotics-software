package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class UIPositionCheckerPacket extends Packet<UIPositionCheckerPacket> implements VisualizablePacket
{
   public Point3D position = new Point3D();
   public Quaternion orientation;

   public UIPositionCheckerPacket()
   {

   }

   public UIPositionCheckerPacket(Point3D position)
   {
      this(position, null);
   }

   public UIPositionCheckerPacket(Point3D position, Quaternion orientation)
   {
      this.position = position;
      this.orientation = orientation;
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