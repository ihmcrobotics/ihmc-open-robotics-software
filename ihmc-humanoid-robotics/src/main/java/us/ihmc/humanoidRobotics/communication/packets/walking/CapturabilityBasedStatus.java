package us.ihmc.humanoidRobotics.communication.packets.walking;

import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.idl.PreallocatedList;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.robotSide.RobotSide;

public class CapturabilityBasedStatus extends Packet<CapturabilityBasedStatus>
{
   public static final int MAXIMUM_NUMBER_OF_VERTICES = 8;

   public Point2D capturePoint = new Point2D();
   public Point2D desiredCapturePoint = new Point2D();

   public Point3D centerOfMass = new Point3D();

   public PreallocatedList<Point2D> leftFootSupportPolygon = new PreallocatedList<>(Point2D.class, Point2D::new, MAXIMUM_NUMBER_OF_VERTICES);
   public PreallocatedList<Point2D> rightFootSupportPolygon = new PreallocatedList<>(Point2D.class, Point2D::new, MAXIMUM_NUMBER_OF_VERTICES);

   public CapturabilityBasedStatus()
   {
      // Empty constructor for serialization
   }

   @Override
   public void set(CapturabilityBasedStatus other)
   {
      capturePoint.set(other.capturePoint);
      desiredCapturePoint.set(other.desiredCapturePoint);
      MessageTools.copyData(other.leftFootSupportPolygon, leftFootSupportPolygon);
      MessageTools.copyData(other.rightFootSupportPolygon, rightFootSupportPolygon);
      setPacketInformation(other);
   }

   public void setSupportPolygon(RobotSide robotSide, FrameConvexPolygon2d footPolygon)
   {
      int numberOfVertices = footPolygon.getNumberOfVertices();

      if (numberOfVertices > MAXIMUM_NUMBER_OF_VERTICES)
      {
         numberOfVertices = MAXIMUM_NUMBER_OF_VERTICES;
      }

      if (robotSide == RobotSide.LEFT)
      {
         leftFootSupportPolygon.clear();
      }
      else
      {
         rightFootSupportPolygon.clear();
      }

      for (int i = 0; i < numberOfVertices; i++)
      {
         if (robotSide == RobotSide.LEFT)
         {
            footPolygon.getVertex(i, leftFootSupportPolygon.add());
         }
         else
         {
            footPolygon.getVertex(i, rightFootSupportPolygon.add());
         }
      }
   }

   public FramePoint2D getCapturePoint()
   {
      return new FramePoint2D(ReferenceFrame.getWorldFrame(), capturePoint);
   }

   public FramePoint2D getDesiredCapturePoint()
   {
      return new FramePoint2D(ReferenceFrame.getWorldFrame(), desiredCapturePoint);
   }

   public FrameConvexPolygon2d getFootSupportPolygon(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT && leftFootSupportPolygon.size() > 0)
         return new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), leftFootSupportPolygon.toArray());
      else if (rightFootSupportPolygon != null)
         return new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame(), rightFootSupportPolygon.toArray());
      else
         return new FrameConvexPolygon2d(ReferenceFrame.getWorldFrame());
   }

   public boolean isInDoubleSupport()
   {
      return leftFootSupportPolygon.size() != 0 & rightFootSupportPolygon.size() != 0;
   }

   public boolean isSupportFoot(RobotSide robotside)
   {
      if (robotside == RobotSide.LEFT)
         return leftFootSupportPolygon.size() != 0;
      else
         return rightFootSupportPolygon.size() != 0;
   }

   @Override
   public boolean epsilonEquals(CapturabilityBasedStatus other, double epsilon)
   {

      boolean ret = this.capturePoint.epsilonEquals(other.capturePoint, epsilon);
      ret &= this.desiredCapturePoint.epsilonEquals(other.desiredCapturePoint, epsilon);

      ret &= this.centerOfMass.epsilonEquals(other.centerOfMass, epsilon);

      if (!MessageTools.epsilonEquals(leftFootSupportPolygon, other.leftFootSupportPolygon, epsilon))
         return false;
      if (!MessageTools.epsilonEquals(rightFootSupportPolygon, other.rightFootSupportPolygon, epsilon))
         return false;

      return ret;
   }
}
