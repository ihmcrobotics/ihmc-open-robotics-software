package us.ihmc.communication.packets;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

public class UIPositionCheckerPacket extends Packet<UIPositionCheckerPacket>
{
   public Point3D position = new Point3D();
   public Quaternion orientation;

   public UIPositionCheckerPacket()
   {

   }

   public UIPositionCheckerPacket(Point3DReadOnly position)
   {
      this(position, null);
   }

   public UIPositionCheckerPacket(Point3DReadOnly position, Quaternion orientation)
   {
      this.position = new Point3D(position);
      this.orientation = orientation;
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