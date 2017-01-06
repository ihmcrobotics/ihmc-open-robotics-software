package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.FrameLine;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class FramePlane3d extends AbstractReferenceFrameHolder
{
   private ReferenceFrame referenceFrame;
   private Plane3d plane3d;

   private final RigidBodyTransform temporaryTransformToDesiredFrame = new RigidBodyTransform();
   private final Vector3d temporaryVector = new Vector3d();
   private final Point3d temporaryPoint = new Point3d();
   
   private final Point3d temporaryPointA = new Point3d();
   private final Point3d temporaryPointB = new Point3d();
   private final Point3d temporaryPointC = new Point3d();

   public FramePlane3d()
   {
      this(ReferenceFrame.getWorldFrame());
   }
   
   public FramePlane3d(ReferenceFrame referenceFrame, Plane3d plane3d)
   {
      this.referenceFrame = referenceFrame;
      this.plane3d = plane3d;
   }

   public FramePlane3d(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
      this.plane3d = new Plane3d();
   }

   public FramePlane3d(FramePlane3d framePlane3d)
   {
      this.referenceFrame = framePlane3d.referenceFrame;
      this.plane3d = new Plane3d(framePlane3d.plane3d);
   }

   public FramePlane3d(FrameVector normal, FramePoint point)
   {
      normal.checkReferenceFrameMatch(point);
      this.referenceFrame = normal.getReferenceFrame();
      this.plane3d = new Plane3d(point.getPoint(), normal.getVector());
   }

   public FramePlane3d(ReferenceFrame referenceFrame, Point3d point, Vector3d normal)
   {
      this.referenceFrame = referenceFrame;
      this.plane3d = new Plane3d(point, normal);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void getNormal(FrameVector normalToPack)
   {
      checkReferenceFrameMatch(normalToPack.getReferenceFrame());
      this.plane3d.getNormal(temporaryVector);
      normalToPack.set(temporaryVector);
   }

   public FrameVector getNormalCopy()
   {
      FrameVector returnVector = new FrameVector(this.referenceFrame);
      getNormal(returnVector);
      return returnVector;
   }
   
   public Vector3d getNormal()
   {
      return plane3d.getNormal();
   }

   public void setNormal(double x, double y, double z)
   {
      plane3d.setNormal(x, y, z);
   }

   public void setNormal(Vector3d normal)
   {
      plane3d.setNormal(normal);
   }

   public void getPoint(FramePoint pointToPack)
   {
      checkReferenceFrameMatch(pointToPack.getReferenceFrame());
      this.plane3d.getPoint(temporaryPoint);
      pointToPack.set(temporaryPoint);
   }

   public FramePoint getPointCopy()
   {
      FramePoint pointToReturn = new FramePoint(this.getReferenceFrame());
      this.getPoint(pointToReturn);
      return pointToReturn;
   }
   
   public Point3d getPoint()
   {
      return plane3d.getPoint();
   }

   public void setPoint(double x, double y, double z)
   {
      plane3d.setPoint(x, y, z);
   }

   public void setPoint(Point3d point)
   {
      plane3d.setPoint(point);
   }

   public void setPoints(FramePoint pointA, FramePoint pointB, FramePoint pointC)
   {
      pointA.checkReferenceFrameMatch(referenceFrame);
      pointB.checkReferenceFrameMatch(referenceFrame);
      pointC.checkReferenceFrameMatch(referenceFrame);

      pointA.get(temporaryPointA);
      pointB.get(temporaryPointB);
      pointC.get(temporaryPointC);
      
      plane3d.setPoints(temporaryPointA, temporaryPointB, temporaryPointC);
   }
   
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      if (desiredFrame != referenceFrame)
      {
         referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
         plane3d.applyTransform(temporaryTransformToDesiredFrame);
         referenceFrame = desiredFrame;
      }

      // otherwise: in the right frame already, so do nothing
   }

   public boolean isOnOrAbove(FramePoint pointToTest)
   {
      checkReferenceFrameMatch(pointToTest);

      return plane3d.isOnOrAbove(pointToTest.getPoint());
   }

   public boolean isOnOrBelow(FramePoint pointToTest)
   {
      checkReferenceFrameMatch(pointToTest);

      return plane3d.isOnOrBelow(pointToTest.getPoint());
   }
   
   public boolean isParallel(FramePlane3d otherPlane, double epsilon)
   {
      checkReferenceFrameMatch(otherPlane);
      return plane3d.isParallel(otherPlane.plane3d, epsilon);
   }
   
   public boolean isCoplanar(FramePlane3d otherPlane, double epsilon)
   {
      checkReferenceFrameMatch(otherPlane);
      return plane3d.isCoplanar(otherPlane.plane3d, epsilon);
   }

   public FramePoint orthogonalProjectionCopy(FramePoint point)
   {
      checkReferenceFrameMatch(point);
      FramePoint returnPoint = new FramePoint(point);
      orthogonalProjection(returnPoint);

      return returnPoint;
   }

   public void orthogonalProjection(FramePoint point)
   {
      checkReferenceFrameMatch(point);
      plane3d.orthogonalProjection(point.getPoint());
   }

   public double getZOnPlane(FramePoint2d xyPoint)
   {
      checkReferenceFrameMatch(xyPoint.getReferenceFrame());
      return plane3d.getZOnPlane(xyPoint.getX(), xyPoint.getY());
   }
   
   public double distance(FramePoint point)
   {
      checkReferenceFrameMatch(point);
      return plane3d.distance(point.getPoint());
   }

   public boolean epsilonEquals(FramePlane3d plane, double epsilon)
   {
      checkReferenceFrameMatch(plane.getReferenceFrame());

      return ((referenceFrame == plane.getReferenceFrame()) && (plane.plane3d.epsilonEquals(this.plane3d, epsilon)));
   }

   public FramePlane3d applyTransformCopy(RigidBodyTransform transformation)
   {
      FramePlane3d returnPlane = new FramePlane3d(this);
      returnPlane.applyTransform(transformation);

      return returnPlane;
   }

   public void applyTransform(RigidBodyTransform transformation)
   {
      plane3d.applyTransform(transformation);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double pointX, double pointY, double pointZ, double normalX, double normalY, double normalZ)
   {
      this.referenceFrame = referenceFrame;
      plane3d.setPoint(pointX, pointY, pointZ);
      plane3d.setNormal(normalX, normalY, normalZ);
   }
   
   public void getIntersectionWithLine(FramePoint pointToPack, FrameLine line)
   {
	   checkReferenceFrameMatch(line.getReferenceFrame());
	   checkReferenceFrameMatch(pointToPack.getReferenceFrame());
	   
	   Point3d intersectionToPack = new Point3d();
	   plane3d.getIntersectionWithLine(intersectionToPack, line.getPoint(), line.getNormalizedVector());
	   pointToPack.set(intersectionToPack);
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();
      builder.append("ReferenceFrame = " + referenceFrame + ", " + plane3d.toString());

      return builder.toString();
   }
}
