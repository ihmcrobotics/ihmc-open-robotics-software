package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class WallPosePacket extends Packet<WallPosePacket>
{
   public double cuttingRadius = 0.20;
   public Point3D centerPosition;
   public Quaternion centerOrientation;

   public WallPosePacket()
   {

   }

   @Override
   public void set(WallPosePacket other)
   {
      cuttingRadius = other.cuttingRadius;
      centerPosition = new Point3D(other.centerPosition);
      centerOrientation = new Quaternion(other.centerOrientation);
      setPacketInformation(other);
   }

   public double getCuttingRadius()
   {
      return cuttingRadius;
   }

   public Point3D getCenterPosition()
   {
      return centerPosition;
   }

   public Quaternion getCenterOrientation()
   {
      return centerOrientation;
   }

   public void setCuttingRadius(double cuttingRadius)
   {
      this.cuttingRadius = cuttingRadius;
   }

   public void setCenterPosition(Tuple3DReadOnly centerPosition)
   {
      this.centerPosition = new Point3D(centerPosition);
   }

   public void setCenterOrientation(QuaternionReadOnly centerOrientation)
   {
      this.centerOrientation = new Quaternion(centerOrientation);
   }

   @Override
   public boolean epsilonEquals(WallPosePacket other, double epsilon)
   {
      if (!MathTools.epsilonEquals(cuttingRadius, other.cuttingRadius, epsilon))
         return false;

      if (centerPosition == null ^ other.centerPosition == null)
         return false;
      if (centerPosition != null && !centerPosition.epsilonEquals(other.centerPosition, epsilon))
         return false;

      if (centerOrientation == null ^ other.centerOrientation == null)
         return false;
      if (centerOrientation != null && !centerOrientation.epsilonEquals(other.centerOrientation, epsilon))
         return false;

      return true;
   }
}
