package us.ihmc.robotics.geometry;

import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * @author Twan Koolen
 */
public class FrameLineSegment2d extends FrameGeometry2d<FrameLineSegment2d, LineSegment2d>
{
   protected final LineSegment2d lineSegment;

   private final Point2d tempPoint2d = new Point2d();
   private final Vector2d tempVector2d = new Vector2d();

   public FrameLineSegment2d()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameLineSegment2d(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new LineSegment2d());
      
      this.lineSegment = this.getGeometryObject();
   }

   public FrameLineSegment2d(ReferenceFrame referenceFrame, LineSegment2d lineSegment2d)
   {
      super(referenceFrame, lineSegment2d);
      
      this.lineSegment = this.getGeometryObject();
   }

   public FrameLineSegment2d(ReferenceFrame referenceFrame, Point2d[] endpoints)
   {
      this(referenceFrame);
      this.lineSegment.set(endpoints);
   }

   public FrameLineSegment2d(ReferenceFrame referenceFrame, Point2d firstEndpoint, Point2d secondEndpoint)
   {
      this(referenceFrame, new LineSegment2d(firstEndpoint, secondEndpoint));
   }

   public FrameLineSegment2d(FramePoint2d[] endpoints)
   {
      this(endpoints[0].getReferenceFrame(), new LineSegment2d(endpoints[0].getPointCopy(), endpoints[1].getPointCopy()));
      
      endpoints[0].checkReferenceFrameMatch(endpoints[1]);
   }

   public FrameLineSegment2d(FramePoint2d firstEndpoint, FramePoint2d secondEndpoint)
   {
      this(firstEndpoint.getReferenceFrame(), new LineSegment2d(firstEndpoint.getPointCopy(), secondEndpoint.getPointCopy()));
      firstEndpoint.checkReferenceFrameMatch(secondEndpoint);
   }

   public FrameLineSegment2d(FrameLineSegment2d frameLineSegment2d)
   {
      this(frameLineSegment2d.getReferenceFrame(), new LineSegment2d(frameLineSegment2d.lineSegment));
   }

   public void set(FramePoint2d firstEndpoint, FramePoint2d secondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(secondEndpoint);
      this.lineSegment.set(firstEndpoint.getPoint(), secondEndpoint.getPoint());
   }

   public void setByProjectionOntoXYPlane(FramePoint firstEndpoint, FramePoint secondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(secondEndpoint);
      
      this.lineSegment.set(firstEndpoint.getX(), firstEndpoint.getY(), secondEndpoint.getX(), secondEndpoint.getY());
   }

   public void set(FramePoint2d firstEndpoint, FrameVector2d vectorToSecondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(vectorToSecondEndpoint);
      this.lineSegment.set(firstEndpoint.getPoint(), vectorToSecondEndpoint.getVector());
   }

   public void setIncludingFrame(FramePoint2d firstEndpoint, FramePoint2d secondEndpoint)
   {
      this.referenceFrame = firstEndpoint.getReferenceFrame();
      set(firstEndpoint, secondEndpoint);
   }

   public void setIncludingFrame(FramePoint2d[] endpoints)
   {
      setIncludingFrame(endpoints[0], endpoints[1]);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, Point2d firstEndpoint, Point2d secondEndpoint)
   {
      this.referenceFrame = referenceFrame;
      this.lineSegment.set(firstEndpoint, secondEndpoint);
   }

   public void setFirstEndpoint(FramePoint2d firstEndpointToPack)
   {
      firstEndpointToPack.checkReferenceFrameMatch(referenceFrame);
      lineSegment.set(firstEndpointToPack.getPoint(), lineSegment.endpoints[1]);
   }

   public void setSecondEndpoint(FramePoint2d secondEndpointToPack)
   {
      secondEndpointToPack.checkReferenceFrameMatch(referenceFrame);
      lineSegment.set(lineSegment.endpoints[0], secondEndpointToPack.getPoint());
   }

   public FramePoint2d getFirstEndpointCopy()
   {
      return new FramePoint2d(referenceFrame, lineSegment.getFirstEndpointCopy());
   }

   public FramePoint2d getSecondEndpointCopy()
   {
      return new FramePoint2d(referenceFrame, lineSegment.getSecondEndpointCopy());
   }

   public void getFirstEndpoint(FramePoint2d firstEndpointToPack)
   {
      firstEndpointToPack.setIncludingFrame(referenceFrame, lineSegment.endpoints[0]);
   }

   public void getSecondEndpoint(FramePoint2d secondEndpointToPack)
   {
      secondEndpointToPack.setIncludingFrame(referenceFrame, lineSegment.endpoints[1]);
   }

   public void getFirstEndpoint(Point2d firstEndpointToPack)
   {
      firstEndpointToPack.set(lineSegment.endpoints[0]);
   }

   public void getSecondEndpoint(Point2d secondEndpointToPack)
   {
      secondEndpointToPack.set(lineSegment.endpoints[1]);
   }

   public void getFrameVector(FrameVector2d vectorToPack)
   {
      vectorToPack.setToZero(referenceFrame);
      vectorToPack.sub(lineSegment.getEndpoints()[1], lineSegment.getEndpoints()[0]);
   }

   public void set(FramePoint2d[] endpoints)
   {
      checkReferenceFrameMatch(endpoints[0]);
      checkReferenceFrameMatch(endpoints[1]);

      this.lineSegment.set(endpoints[0].getPoint(), endpoints[1].getPoint());
   }

   public void set(ReferenceFrame referenceFrame, double firstEndpointX, double firstEndpointY, double secondEndpointX, double secondEndpointY)
   {
      checkReferenceFrameMatch(referenceFrame);
      this.lineSegment.set(firstEndpointX, firstEndpointY, secondEndpointX, secondEndpointY);
   }

   @Override
   public void set(FrameLineSegment2d lineSegment)
   {
      checkReferenceFrameMatch(lineSegment);
      this.lineSegment.set(lineSegment.lineSegment);
   }

   // TODO change to setIncludingFrame
   public void setAndChangeFrame(FrameLineSegment2d lineSegment)
   {
      this.referenceFrame = lineSegment.referenceFrame;
      this.lineSegment.set(lineSegment.lineSegment);
   }

   public void set(ReferenceFrame referenceFrame, LineSegment2d lineSegment2d)
   {
      this.referenceFrame = referenceFrame;
      this.lineSegment.set(lineSegment2d);
   }

   public void flipDirection()
   {
      this.lineSegment.flipDirection();
   }

   public LineSegment2d getLineSegment2d()
   {
      return this.lineSegment;
   }

   public LineSegment2d getLineSegment2dCopy()
   {
      return new LineSegment2d(lineSegment);
   }

   public FramePoint2d[] getEndFramePointsCopy()
   {
      FramePoint2d[] endFramePoints = new FramePoint2d[2];
      endFramePoints[0] = new FramePoint2d(referenceFrame, lineSegment.endpoints[0]);
      endFramePoints[1] = new FramePoint2d(referenceFrame, lineSegment.endpoints[1]);

      return endFramePoints;
   }

   public double length()
   {
      return lineSegment.length();
   }

   public FramePoint2d midpoint()
   {
      double x = (lineSegment.endpoints[0].getX() + lineSegment.endpoints[1].getX()) / 2.0;
      double y = (lineSegment.endpoints[0].getY() + lineSegment.endpoints[1].getY()) / 2.0;

      return new FramePoint2d(referenceFrame, x, y);
   }

   public double dotProduct(FrameLineSegment2d frameLineSegment2d)
   {
      checkReferenceFrameMatch(frameLineSegment2d);

      return lineSegment.dotProduct(frameLineSegment2d.lineSegment);
   }

   public boolean isBetweenEndpoints(FramePoint2d point2d, double epsilon)
   {
      checkReferenceFrameMatch(point2d);

      return lineSegment.isBetweenEndpoints(point2d.getPoint(), epsilon);
   }

   public double percentageAlongLineSegment(FramePoint2d point2d)
   {
      checkReferenceFrameMatch(point2d);

      return lineSegment.percentageAlongLineSegment(point2d.getPoint());
   }

   @Override
   public void applyTransformAndProjectToXYPlane(RigidBodyTransform transform)
   {
      lineSegment.applyTransformAndProjectToXYPlane(transform);
   }

   @Override
   public FrameLineSegment2d applyTransformCopy(RigidBodyTransform transform)
   {
      FrameLineSegment2d copy = new FrameLineSegment2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   @Override
   public FrameLineSegment2d applyTransformAndProjectToXYPlaneCopy(RigidBodyTransform transform)
   {
      FrameLineSegment2d copy = new FrameLineSegment2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

   private RigidBodyTransform temporaryTransformToDesiredFrame;

   @Override
   public void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // this is in the correct frame already
      if (desiredFrame == referenceFrame)
         return;

      if (temporaryTransformToDesiredFrame == null)
         temporaryTransformToDesiredFrame = new RigidBodyTransform();

      referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);

      applyTransformAndProjectToXYPlane(temporaryTransformToDesiredFrame);
      referenceFrame = desiredFrame;
   }

   @Override
   public FrameLineSegment2d changeFrameAndProjectToXYPlaneCopy(ReferenceFrame desiredFrame)
   {
      FrameLineSegment2d copy = new FrameLineSegment2d(this);
      copy.changeFrameAndProjectToXYPlane(desiredFrame);
      return copy;
   }

   @Override
   public String toString()
   {
      return "" + lineSegment;
   }

   @Override
   public void orthogonalProjection(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);
      lineSegment.orthogonalProjection(point.getPoint());
   }

   public void orthogonalProjection(FramePoint2d point, FramePoint2d projectedPointToPack)
   {
      checkReferenceFrameMatch(point);
      projectedPointToPack.setToZero(referenceFrame);
      lineSegment.orthogonalProjection(point.getPoint(), projectedPointToPack.getPoint());
   }

   @Override
   public FramePoint2d orthogonalProjectionCopy(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);
      Point2d projected = lineSegment.orthogonalProjectionCopy(point.getPoint());

      return new FramePoint2d(point.getReferenceFrame(), projected);
   }

   @Override
   public FramePoint2d intersectionWith(FrameLine2d line)
   {
      checkReferenceFrameMatch(line);
      Point2d intersection = this.lineSegment.intersectionWith(line.line);
      if (intersection == null)
      {
         return null;
      }
      return new FramePoint2d(line.getReferenceFrame(), intersection);
   }

   /**
    * Find the intersection point between this FrameLineSegment2d and the argument FrameLine2d and store it in the FramePoint2d argument.
    * @param intersectionPointToPack argument in which the intersect point is stored
    * @param line used to find the intersection with this line segment
    * @return true if successful, false otherwise.
    */
   public boolean intersectionWith(FramePoint2d intersectionPointToPack, FrameLine2d line)
   {
      checkReferenceFrameMatch(line);
      Point2d intersection = this.lineSegment.intersectionWith(line.line);

      if (intersection == null)
         return false;

      intersectionPointToPack.setIncludingFrame(line.getReferenceFrame(), intersection);

      return true;
   }

   @Override
   public FramePoint2d intersectionWith(FrameLineSegment2d secondLineSegment)
   {
      checkReferenceFrameMatch(secondLineSegment);
      Point2d intersection = this.lineSegment.intersectionWith(secondLineSegment.lineSegment);
      if (intersection == null)
      {
         return null;
      }
      return new FramePoint2d(secondLineSegment.getReferenceFrame(), intersection);
   }

   @Override
   public FramePoint2d[] intersectionWith(FrameConvexPolygon2d convexPolygon)
   {
      checkReferenceFrameMatch(convexPolygon);
      Point2d[] intersection = this.lineSegment.intersectionWith(convexPolygon.convexPolygon);
      FramePoint2d[] ret = new FramePoint2d[intersection.length];
      for (int i = 0; i < intersection.length; i++)
      {
         ret[i] = new FramePoint2d(convexPolygon.referenceFrame, intersection[i]);
      }

      return ret;
   }

   @Override
   public double distance(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);

      return this.lineSegment.distance(point.getPoint());
   }

   @Override
   public double distance(FrameLine2d line)
   {
      checkReferenceFrameMatch(line);

      return this.lineSegment.distance(line.line);
   }

   @Override
   public double distance(FrameLineSegment2d secondLineSegment)
   {
      checkReferenceFrameMatch(secondLineSegment);

      return this.lineSegment.distance(secondLineSegment.lineSegment);
   }

   @Override
   public double distance(FrameConvexPolygon2d convexPolygon)
   {
      checkReferenceFrameMatch(convexPolygon);

      return this.lineSegment.distance(convexPolygon.convexPolygon);
   }
   
   public boolean isPointOnLeftSideOfLineSegment(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);

      point.get(tempPoint2d);
      return this.lineSegment.isPointOnLeftSideOfLineSegment(tempPoint2d);
   }

   public boolean isPointOnRightSideOfLineSegment(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);

      return this.lineSegment.isPointOnRightSideOfLineSegment(point.getPointCopy());
   }

   public FrameLineSegment2d shiftToLeftCopy(double distanceToShift)
   {
      return new FrameLineSegment2d(referenceFrame, lineSegment.shiftToLeftCopy(distanceToShift));
   }

   public FrameLineSegment2d shiftToRightCopy(double distanceToShift)
   {
      return new FrameLineSegment2d(referenceFrame, lineSegment.shiftToRightCopy(distanceToShift));
   }

   public void shiftToLeft(double distanceToShift)
   {
      lineSegment.shiftToLeft(distanceToShift);
   }

   public void shiftToRight(double distanceToShift)
   {
      lineSegment.shiftToRight(distanceToShift);
   }

   public FramePoint2d pointBetweenEndPointsGivenParameter(double parameter)
   {
      return new FramePoint2d(referenceFrame, lineSegment.pointBetweenEndPointsGivenParameter(parameter));
   }

   public void pointBetweenEndPointsGivenParameter(FramePoint2d framePoint2dToPack, double parameter)
   {
      framePoint2dToPack.setIncludingFrame(referenceFrame, lineSegment.pointBetweenEndPointsGivenParameter(parameter));
   }
   
   public void getClosestPointOnLineSegment(FramePoint2d framePoint2dToPack, FramePoint2d point)
   {
      checkReferenceFrameMatch(point);
      
      lineSegment.getClosestPointOnLineSegment(tempPoint2d, point.getPoint());
      framePoint2dToPack.setIncludingFrame(referenceFrame, tempPoint2d);
   }
   
   public void getPerpendicularBisector(FrameVector2d perpendicularBisectorToPack, double bisectorLengthDesired)
   {
      lineSegment.getPerpendicularBisector(tempVector2d, bisectorLengthDesired);
      perpendicularBisectorToPack.setIncludingFrame(referenceFrame, tempVector2d);
   }

   public static FrameLineSegment2d generateRandomFrameLineSegment2d(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin, double yMax)
   {
      FramePoint2d randomPoint1 = FramePoint2d.generateRandomFramePoint2d(random, zUpFrame, xMin, xMax, yMin, yMax);
      FramePoint2d randomPoint2 = FramePoint2d.generateRandomFramePoint2d(random, zUpFrame, xMin, xMax, yMin, yMax);

      return new FrameLineSegment2d(randomPoint1, randomPoint2);
   }

   @Override
   public void setToNaN()
   {
      lineSegment.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }
}
