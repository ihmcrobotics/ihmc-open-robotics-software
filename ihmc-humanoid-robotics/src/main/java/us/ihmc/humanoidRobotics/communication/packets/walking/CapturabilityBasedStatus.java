package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.RecyclingArrayListPubSub;

public class CapturabilityBasedStatus extends Packet<CapturabilityBasedStatus>
{
   public static final int MAXIMUM_NUMBER_OF_VERTICES = 8;

   public Point3D capturePoint2D = new Point3D();
   public Point3D desiredCapturePoint2D = new Point3D();

   public Point3D centerOfMass3D = new Point3D();

   public RecyclingArrayListPubSub<Point3D> leftFootSupportPolygon2D = new RecyclingArrayListPubSub<>(Point3D.class, Point3D::new, MAXIMUM_NUMBER_OF_VERTICES);
   public RecyclingArrayListPubSub<Point3D> rightFootSupportPolygon2D = new RecyclingArrayListPubSub<>(Point3D.class, Point3D::new, MAXIMUM_NUMBER_OF_VERTICES);

   public CapturabilityBasedStatus()
   {
      // Empty constructor for serialization
   }

   @Override
   public void set(CapturabilityBasedStatus other)
   {
      capturePoint2D.set(other.capturePoint2D);
      desiredCapturePoint2D.set(other.desiredCapturePoint2D);
      MessageTools.copyData(other.leftFootSupportPolygon2D, leftFootSupportPolygon2D);
      MessageTools.copyData(other.rightFootSupportPolygon2D, rightFootSupportPolygon2D);
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(CapturabilityBasedStatus other, double epsilon)
   {

      boolean ret = this.capturePoint2D.epsilonEquals(other.capturePoint2D, epsilon);
      ret &= this.desiredCapturePoint2D.epsilonEquals(other.desiredCapturePoint2D, epsilon);

      ret &= this.centerOfMass3D.epsilonEquals(other.centerOfMass3D, epsilon);

      if (!MessageTools.epsilonEquals(leftFootSupportPolygon2D, other.leftFootSupportPolygon2D, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(rightFootSupportPolygon2D, other.rightFootSupportPolygon2D, epsilon))
         return false;

      return ret;
   }
}
