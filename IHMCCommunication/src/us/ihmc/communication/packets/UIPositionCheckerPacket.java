package us.ihmc.communication.packets;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;

public class UIPositionCheckerPacket extends Packet<UIPositionCheckerPacket> implements VisualizablePacket
{
   public Point3d position = new Point3d();
   public Quat4d orientation;

   public UIPositionCheckerPacket()
   {

   }

   public UIPositionCheckerPacket(Point3d position)
   {
      this(position, null);
   }

   public UIPositionCheckerPacket(Point3d position, Quat4d orientation)
   {
      this.position = position;
      this.orientation = orientation;
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
   public boolean epsilonEquals(UIPositionCheckerPacket other, double epsilon)
   {
      if (orientation != null)
         return (position.epsilonEquals(other.getPosition(), (float) epsilon)) && orientation.epsilonEquals(other.getOrientation(), epsilon);
      return position.epsilonEquals(other.getPosition(), (float) epsilon);
   }
}