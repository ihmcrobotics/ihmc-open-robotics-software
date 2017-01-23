package us.ihmc.robotics.geometry;

import javax.vecmath.Point3d;

import us.ihmc.robotics.geometry.transformables.TransformableLineSegment3d;
import us.ihmc.robotics.geometry.transformables.TransformablePoint3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * FrameLineSegment whose aim is to be used in 3D robot controllers. Keep GC-free!
 * 
 * Create one as a final field in your controller. Do not instantiate every tick!
 * 
 * @author dcalvert
 */
public class FrameLineSegment extends AbstractFrameObject<FrameLineSegment, TransformableLineSegment3d>
{
   /** Actual startPoint */
   private final Point3d startPoint;
   /** Actual endPoint */
   private final Point3d endPoint;
   
   /** For temporary type-conversion! Not actual value */
   private final LineSegment3d lineSegment3d = new LineSegment3d();
   
   public FrameLineSegment(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new TransformableLineSegment3d());
      
      startPoint = getGeometryObject().getStartPoint();
      endPoint = getGeometryObject().getEndPoint();
   }
   
   public double getDistance(FramePoint framePoint)
   {
      checkReferenceFrameMatch(framePoint);
      
      putValuesIntoLineSegment3d();
      return lineSegment3d.distance(framePoint.getPoint());
   }
   
   public void getPointAlongPercentageOfLineSegment(double percentage, FramePoint pointToPack)
   {
      if (percentage < 0.0 || percentage > 1.0)
      {
         throw new RuntimeException("Percentage must be between 0.0 and 1.0. Was: " + percentage);
      }
      
      checkReferenceFrameMatch(pointToPack);
      
      pointToPack.set(endPoint);
      pointToPack.sub(startPoint);
      pointToPack.scale(percentage);
      pointToPack.add(startPoint);
   }
   
   public void getProjectionOntoLineSegment(FramePoint pointToProject, FramePoint projectedPointToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      checkReferenceFrameMatch(projectedPointToPack);
      
      putValuesIntoLineSegment3d();
      lineSegment3d.orthogonalProjection(pointToProject.getPoint(), projectedPointToPack.getPoint());
   }
   
   public double getLength()
   {
      putValuesIntoLineSegment3d();
      return lineSegment3d.length();
   }
   
   public boolean isBetweenEndpoints(FramePoint point, double epsilon)
   {
      checkReferenceFrameMatch(point);
      putValuesIntoLineSegment3d();
      return lineSegment3d.isBetweenEndpoints(point.getPoint(), epsilon);
   }
   
   public void getMidpoint(FramePoint midpointToPack)
   {
      checkReferenceFrameMatch(midpointToPack);
      
      midpointToPack.setX((startPoint.getX() + endPoint.getX()) / 2.0);
      midpointToPack.setY((startPoint.getY() + endPoint.getY()) / 2.0);
      midpointToPack.setZ((startPoint.getZ() + endPoint.getZ()) / 2.0);
   }
   
   public void getFrameVector(FrameVector startToEndVector)
   {
      checkReferenceFrameMatch(startToEndVector);
      
      startToEndVector.setX(endPoint.getX() - startPoint.getX());
      startToEndVector.setY(endPoint.getY() - startPoint.getY());
      startToEndVector.setZ(endPoint.getZ() - startPoint.getZ());
   }
   
   public void set(FramePoint startPoint, FrameVector segmentVector)
   {
      checkReferenceFrameMatch(startPoint);
      checkReferenceFrameMatch(segmentVector);
      
      setStartPointWithoutChecks(startPoint.getPoint());
      
      endPoint.set(this.startPoint);
      endPoint.add(segmentVector.getVector());
   }
   
   public void setStartPoint(FramePoint startPoint)
   {
      checkReferenceFrameMatch(startPoint);
      
      setWithoutChecks(startPoint.getPoint(), endPoint);
   }
   
   public void setEndPoint(FramePoint endPoint)
   {
      checkReferenceFrameMatch(endPoint);
      
      setWithoutChecks(startPoint, endPoint.getPoint());
   }
   
   public void set(FramePoint startPoint, FramePoint endPoint)
   {
      checkReferenceFrameMatch(startPoint);
      checkReferenceFrameMatch(endPoint);
      
      setWithoutChecks(startPoint.getPoint(), endPoint.getPoint());
   }
   
   public void setStartPointWithoutChecks(Point3d startPoint)
   {
      setWithoutChecks(startPoint, endPoint);
   }
   
   public void setEndPointWithoutChecks(Point3d endPoint)
   {
      setWithoutChecks(startPoint, endPoint);
   }
   
   public void setWithoutChecks(Point3d startPoint, Point3d endPoint)
   {
      this.startPoint.set(startPoint.getX(), startPoint.getY(), startPoint.getZ());
      this.endPoint.set(endPoint.getX(), endPoint.getY(), endPoint.getZ());
   }
   
   public TransformablePoint3d getStartPointUnsafe()
   {
      return getGeometryObject().getStartPoint();
   }
   
   public TransformablePoint3d getEndPointUnsafe()
   {
      return getGeometryObject().getEndPoint();
   }
   
   public LineSegment3d getTemporaryLineSegment3d()
   {
      putValuesIntoLineSegment3d();
      return lineSegment3d;
   }
   
   private void putValuesIntoLineSegment3d()
   {
      lineSegment3d.getFirstEndpoint().set(startPoint.getX(), startPoint.getY(), startPoint.getZ());
      lineSegment3d.getSecondEndpoint().set(endPoint.getX(), endPoint.getY(), endPoint.getZ());
   }
   
   private void getValuesFromLineSegment3d()
   {
      lineSegment3d.getFirstEndpoint().get(startPoint);
      lineSegment3d.getSecondEndpoint().get(endPoint);
   }
}
