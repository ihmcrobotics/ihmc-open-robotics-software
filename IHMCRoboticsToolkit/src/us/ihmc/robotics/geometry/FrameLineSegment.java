package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * FrameLineSegment whose aim is to be used in 3D robot controllers. Keep GC-free!
 * 
 * Create one as a final field in your controller. Do not instantiate every tick!
 * 
 * @author dcalvert
 */
public class FrameLineSegment extends AbstractFrameObject<FrameLineSegment, LineSegment3D>
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

   public void setFirstEndpoint(FramePoint firstEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      lineSegment3d.setFirstEndpoint(firstEndpoint.getPoint());
   }

   public void setSecondEndpoint(Point3D secondEndpoint)
   {
      lineSegment3d.setSecondEndpoint(secondEndpoint);
   }

   public void setSecondEndpoint(FramePoint secondEndpoint)
   {
      checkReferenceFrameMatch(secondEndpoint);
      lineSegment3d.setSecondEndpoint(secondEndpoint.getPoint());
   }

   public void set(Point3D firstEndpoint, Point3D secondEndpoint)
   {
      setFirstEndpoint(firstEndpoint);
      setSecondEndpoint(secondEndpoint);
   }

   public void set(FramePoint firstEndpoint, FramePoint secondEndpoint)
   {
      setFirstEndpoint(firstEndpoint);
      setSecondEndpoint(secondEndpoint);
   }

   public void setIncludingFrame(FramePoint firstEndpoint, FramePoint secondEndpoint)
   {
      firstEndpoint.checkReferenceFrameMatch(secondEndpoint);
      setToZero(firstEndpoint.getReferenceFrame());
      set(firstEndpoint, secondEndpoint);
   }

   public void set(Point3D firstEndpoint, Vector3D fromFirstToSecondEndpoint)
   {
      lineSegment3d.set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   public void set(FramePoint firstEndpoint, FrameVector fromFirstToSecondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(fromFirstToSecondEndpoint);
      lineSegment3d.set(firstEndpoint.getPoint(), fromFirstToSecondEndpoint.getVector());
   }

   public void setIncludingFrame(FramePoint firstEndpoint, FrameVector fromFirstToSecondEndpoint)
   {
      firstEndpoint.checkReferenceFrameMatch(fromFirstToSecondEndpoint);
      setToZero(firstEndpoint.getReferenceFrame());
      set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   public double getDistance(FramePoint framePoint)
   {
      checkReferenceFrameMatch(framePoint);
      return lineSegment3d.distance(framePoint.getPoint());
   }

   public void orthogonalProjection(FramePoint pointToProject, FramePoint projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      checkReferenceFrameMatch(projectionToPack);
      lineSegment3d.orthogonalProjection(pointToProject.getPoint(), projectionToPack.getPoint());
   }

   public void pointBetweenEndPointsGivenPercantage(double percentage, FramePoint pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      lineSegment3d.pointBetweenEndPointsGivenPercentage(percentage, pointToPack.getPoint());
   }

   public double length()
   {
      return lineSegment3d.length();
   }

   public boolean isBetweenEndpoints(FramePoint point, double epsilon)
   {
      checkReferenceFrameMatch(point);
      return lineSegment3d.isBetweenEndpoints(point.getPoint(), epsilon);
   }

   public void getMidpoint(FramePoint midpointToPack)
   {
      checkReferenceFrameMatch(midpointToPack);
      lineSegment3d.midpoint(midpointToPack.getPoint());
   }

   public void getDirection(boolean normalize, FrameVector directionToPack)
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

   public void getFirstEndpoint(FramePoint firstEndpointToPack)
   {
      checkReferenceFrameMatch(firstEndpointToPack);
      firstEndpointToPack.set(getFirstEndpoint());
   }

   public void getFirstEndpointIncludingFrame(FramePoint firstEndpointToPack)
   {
      firstEndpointToPack.setIncludingFrame(referenceFrame, getFirstEndpoint());
   }

   public Point3D getSecondEndpoint()
   {
      return lineSegment3d.getSecondEndpoint();
   }

   public void getSecondEndpoint(FramePoint secondEndpointToPack)
   {
      checkReferenceFrameMatch(secondEndpointToPack);
      secondEndpointToPack.set(getSecondEndpoint());
   }

   public void getSecondEndpointIncludingFrame(FramePoint secondEndpointToPack)
   {
      secondEndpointToPack.setIncludingFrame(referenceFrame, getSecondEndpoint());
   }

   public void get(FramePoint firstEndpointToPack, FramePoint secondEndpointToPack)
   {
      getFirstEndpoint(firstEndpointToPack);
      getSecondEndpoint(secondEndpointToPack);
   }

   public void getIncludingFrame(FramePoint firstEndpointToPack, FramePoint secondEndpointToPack)
   {
      getFirstEndpointIncludingFrame(firstEndpointToPack);
      getSecondEndpointIncludingFrame(secondEndpointToPack);
   }

   public LineSegment3D getLineSegment3d()
   {
      return lineSegment3d;
   }
}
