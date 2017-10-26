package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * FrameLineSegment whose aim is to be used in 3D robot controllers. Keep GC-free!
 * 
 * Create one as a final field in your controller. Do not instantiate every tick!
 * 
 * @author dcalvert
 */
public class FrameLineSegment extends FrameGeometryObject<FrameLineSegment, LineSegment3D>
{
   private final LineSegment3D lineSegment3d;

   public FrameLineSegment()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameLineSegment(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new LineSegment3D());
      lineSegment3d = getGeometryObject();
   }

   public FrameLineSegment(ReferenceFrame referenceFrame, Point3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint)
   {
      super(referenceFrame, new LineSegment3D(firstEndpoint, secondEndpoint));
      lineSegment3d = getGeometryObject();
   }

   public void setFirstEndpoint(Point3D firstEndpoint)
   {
      lineSegment3d.setFirstEndpoint(firstEndpoint);
   }

   public void setFirstEndpoint(FramePoint3D firstEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      lineSegment3d.setFirstEndpoint(firstEndpoint.getPoint());
   }

   public void setSecondEndpoint(Point3D secondEndpoint)
   {
      lineSegment3d.setSecondEndpoint(secondEndpoint);
   }

   public void setSecondEndpoint(FramePoint3D secondEndpoint)
   {
      checkReferenceFrameMatch(secondEndpoint);
      lineSegment3d.setSecondEndpoint(secondEndpoint.getPoint());
   }

   public void set(Point3D firstEndpoint, Point3D secondEndpoint)
   {
      setFirstEndpoint(firstEndpoint);
      setSecondEndpoint(secondEndpoint);
   }

   public void set(FramePoint3D firstEndpoint, FramePoint3D secondEndpoint)
   {
      setFirstEndpoint(firstEndpoint);
      setSecondEndpoint(secondEndpoint);
   }

   public void setIncludingFrame(FramePoint3D firstEndpoint, FramePoint3D secondEndpoint)
   {
      firstEndpoint.checkReferenceFrameMatch(secondEndpoint);
      setToZero(firstEndpoint.getReferenceFrame());
      set(firstEndpoint, secondEndpoint);
   }

   public void set(Point3D firstEndpoint, Vector3D fromFirstToSecondEndpoint)
   {
      lineSegment3d.set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   public void set(FramePoint3D firstEndpoint, FrameVector3D fromFirstToSecondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(fromFirstToSecondEndpoint);
      lineSegment3d.set(firstEndpoint.getPoint(), fromFirstToSecondEndpoint.getVector());
   }

   public void setIncludingFrame(FramePoint3D firstEndpoint, FrameVector3D fromFirstToSecondEndpoint)
   {
      firstEndpoint.checkReferenceFrameMatch(fromFirstToSecondEndpoint);
      setToZero(firstEndpoint.getReferenceFrame());
      set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   public double getDistance(FramePoint3D framePoint)
   {
      checkReferenceFrameMatch(framePoint);
      return lineSegment3d.distance(framePoint.getPoint());
   }

   public void orthogonalProjection(FramePoint3D pointToProject, FramePoint3D projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      checkReferenceFrameMatch(projectionToPack);
      lineSegment3d.orthogonalProjection(pointToProject.getPoint(), projectionToPack.getPoint());
   }

   public void pointBetweenEndPointsGivenPercantage(double percentage, FramePoint3D pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      lineSegment3d.pointBetweenEndpointsGivenPercentage(percentage, pointToPack.getPoint());
   }

   public double length()
   {
      return lineSegment3d.length();
   }

   public boolean isBetweenEndpoints(FramePoint3D point, double epsilon)
   {
      checkReferenceFrameMatch(point);
      return lineSegment3d.isBetweenEndpoints(point.getPoint(), epsilon);
   }

   public void getMidpoint(FramePoint3D midpointToPack)
   {
      checkReferenceFrameMatch(midpointToPack);
      lineSegment3d.midpoint(midpointToPack.getPoint());
   }

   public void getDirection(boolean normalize, FrameVector3D directionToPack)
   {
      checkReferenceFrameMatch(directionToPack);
      lineSegment3d.getDirection(normalize, directionToPack.getVector());
   }

   public boolean firstEndpointContainsNaN()
   {
      return lineSegment3d.firstEndpointContainsNaN();
   }
   
   public boolean secondEndpointContainsNaN()
   {
      return lineSegment3d.secondEndpointContainsNaN();
   }

   public Point3D getFirstEndpoint()
   {
      return lineSegment3d.getFirstEndpoint();
   }

   public void getFirstEndpoint(FramePoint3D firstEndpointToPack)
   {
      checkReferenceFrameMatch(firstEndpointToPack);
      firstEndpointToPack.set(getFirstEndpoint());
   }

   public void getFirstEndpointIncludingFrame(FramePoint3D firstEndpointToPack)
   {
      firstEndpointToPack.setIncludingFrame(referenceFrame, getFirstEndpoint());
   }

   public Point3D getSecondEndpoint()
   {
      return lineSegment3d.getSecondEndpoint();
   }

   public void getSecondEndpoint(FramePoint3D secondEndpointToPack)
   {
      checkReferenceFrameMatch(secondEndpointToPack);
      secondEndpointToPack.set(getSecondEndpoint());
   }

   public void getSecondEndpointIncludingFrame(FramePoint3D secondEndpointToPack)
   {
      secondEndpointToPack.setIncludingFrame(referenceFrame, getSecondEndpoint());
   }

   public void get(FramePoint3D firstEndpointToPack, FramePoint3D secondEndpointToPack)
   {
      getFirstEndpoint(firstEndpointToPack);
      getSecondEndpoint(secondEndpointToPack);
   }

   public void getIncludingFrame(FramePoint3D firstEndpointToPack, FramePoint3D secondEndpointToPack)
   {
      getFirstEndpointIncludingFrame(firstEndpointToPack);
      getSecondEndpointIncludingFrame(secondEndpointToPack);
   }

   public LineSegment3D getLineSegment3d()
   {
      return lineSegment3d;
   }

   public void getIncludingFrame(FrameLineSegment2d lineSegment2DToPack)
   {
      lineSegment2DToPack.setToZero(getReferenceFrame());
      lineSegment2DToPack.set(getReferenceFrame(), lineSegment3d.getFirstEndpointX(), lineSegment3d.getFirstEndpointY(), lineSegment3d.getSecondEndpointX(), lineSegment3d.getSecondEndpointY());
   }
}
