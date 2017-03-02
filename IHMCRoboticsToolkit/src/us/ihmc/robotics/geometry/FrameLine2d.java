package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * @author Twan Koolen
 */
public class FrameLine2d extends FrameGeometry2d<FrameLine2d, Line2d>
{
   private RigidBodyTransform temporaryTransformToDesiredFrame;
   protected final Line2d line;

   public FrameLine2d()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameLine2d(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, new Line2d(new Point2D(), new Vector2D(1.0, 0.0)));
   }

   public FrameLine2d(ReferenceFrame referenceFrame, Line2d line2d)
   {
      super(referenceFrame, line2d);
      this.line = getGeometryObject();
   }

   public FrameLine2d(ReferenceFrame referenceFrame, Point2DReadOnly point2d, Vector2DReadOnly vector2d)
   {
      this(referenceFrame, new Line2d(point2d, vector2d));
   }

   public FrameLine2d(ReferenceFrame referenceFrame, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      this(referenceFrame, new Line2d(firstPointOnLine, secondPointOnLine));
   }

   public FrameLine2d(FramePoint2d framePoint2d, FrameVector2d frameVector2d)
   {
      this(framePoint2d.getReferenceFrame(), new Line2d(framePoint2d.getPointCopy(), frameVector2d.getVectorCopy()));
      framePoint2d.checkReferenceFrameMatch(frameVector2d);
   }

   public FrameLine2d(FramePoint2d firstPointOnLine, FramePoint2d secondPointOnLine)
   {
      this(firstPointOnLine.getReferenceFrame(), new Line2d(firstPointOnLine.getPointCopy(), secondPointOnLine.getPointCopy()));
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
   }

   public FrameLine2d(FrameLine2d frameLine2d)
   {
      this(frameLine2d.getReferenceFrame(), new Line2d(frameLine2d.line));
   }

   public FrameLine2d(FrameLineSegment2d frameLineSegment2d)
   {
      this(frameLineSegment2d.referenceFrame, frameLineSegment2d.lineSegment.endpoints[0], frameLineSegment2d.lineSegment.endpoints[1]);
   }

   public Line2d getLine2d()
   {
      return line;
   }

   public Line2d getLine2dCopy()
   {
      return new Line2d(line);
   }

   public void getPoint2d(Point2DBasics origin)
   {
      line.getPoint(origin);
   }

   public void getFramePoint2d(FramePoint2d framePoint2d)
   {
      framePoint2d.setToZero(referenceFrame);
      line.getPoint(framePoint2d.tuple);
   }

   public void getNormalizedFrameVector(Vector2DBasics vector2dToPack)
   {
      vector2dToPack.set(line.normalizedVector);
   }

   public void getNormalizedFrameVector(FrameVector2d frameVector2dToPack)
   {
      frameVector2dToPack.setToZero(referenceFrame);
      frameVector2dToPack.set(line.normalizedVector);
   }

   public FramePoint2d getFramePoint2dGivenParameter(double t)
   {
      return new FramePoint2d(referenceFrame, line.getPointGivenParameter(t));
   }

   public double getParameterGivenPointEpsilon(FramePoint2d point, double epsilon)
   {
      return line.getParameterGivenPointEpsilon(point.getPoint(), epsilon);
   }

   public boolean containsEpsilon(FramePoint2d framePoint, double epsilon)
   {
      framePoint.checkReferenceFrameMatch(referenceFrame);

      return line.containsEpsilon(framePoint.getPointCopy(), epsilon);
   }

   public void negateDirection()
   {
      line.negateDirection();
   }

   public FrameLine2d negateDirectionCopy()
   {
      FrameLine2d ret = new FrameLine2d(this.referenceFrame, line.negateDirectionCopy());

      return ret;
   }

   public void rotate(double radians)
   {
      this.line.rotate(radians);
   }
   
   public void setPoint(FramePoint2d framePoint2d)
   {
      checkReferenceFrameMatch(framePoint2d);
      
      line.getPoint().set(framePoint2d.getPoint());
   }
   
   public void setVector(FrameVector2d frameVector2d)
   {
      checkReferenceFrameMatch(frameVector2d);
      
      line.getNormalizedVector().set(frameVector2d.getVector());
   }

   @Deprecated
   public void setFramePoint2d(FramePoint2d framePoint2d)
   {
      setPoint(framePoint2d);
   }

   public void setByProjectionOntoXYPlane(FramePoint startPoint, FramePoint endPoint)
   {
      checkReferenceFrameMatch(startPoint);
      checkReferenceFrameMatch(endPoint);
      
      line.getPoint().set(startPoint.getX(), startPoint.getY());
      line.getNormalizedVector().set(endPoint.getX() - startPoint.getX(), endPoint.getY() - startPoint.getY());
   }
   
   public void set(FramePoint2d endpoint0, FramePoint2d endpoint1)
   {
      checkReferenceFrameMatch(endpoint0);
      checkReferenceFrameMatch(endpoint1);
      this.line.set(endpoint0.getPoint(), endpoint1.getPoint());
   }

   public void set(FramePoint2d startPoint, FrameVector2d vector)
   {
      checkReferenceFrameMatch(startPoint);
      checkReferenceFrameMatch(vector);
      this.line.set(startPoint.getPoint(), vector.getVector());
   }

   public void set(FramePoint2d[] endpoints)
   {
      checkReferenceFrameMatch(endpoints[0]);
      checkReferenceFrameMatch(endpoints[1]);

      this.line.set(endpoints[0].getPoint(), endpoints[1].getPoint());
   }

   @Override
   public void setIncludingFrame(FrameLine2d otherFrameLine2d)
   {
      this.referenceFrame = otherFrameLine2d.referenceFrame;
      this.line.set(otherFrameLine2d.line);
   }

   public void setIncludingFrame(FramePoint2d endpoint0, FramePoint2d endpoint1)
   {
      endpoint0.checkReferenceFrameMatch(endpoint1);
      this.referenceFrame = endpoint0.referenceFrame;
      this.line.set(endpoint0.getPoint(), endpoint1.getPoint());
   }

   public void setIncludingFrame(FramePoint2d startPoint, FrameVector2d vector)
   {
      startPoint.checkReferenceFrameMatch(vector);
      this.referenceFrame = startPoint.getReferenceFrame();
      this.line.set(startPoint.getPoint(), vector.getVector());
   }

   public void set(ReferenceFrame referenceFrame, double pointX, double pointY, double vectorX, double vectorY)
   {
      checkReferenceFrameMatch(referenceFrame);
      this.line.set(pointX, pointY, vectorX, vectorY);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double pointX, double pointY, double vectorX, double vectorY)
   {
      this.referenceFrame = referenceFrame;
      this.line.set(pointX, pointY, vectorX, vectorY);
   }

   @Override
   public void set(FrameLine2d frameLine2d)
   {
      checkReferenceFrameMatch(frameLine2d);
      this.line.set(frameLine2d.line);
   }

   public FrameLine2d interiorBisector(FrameLine2d secondLine)
   {
      this.checkReferenceFrameMatch(secondLine);
      ReferenceFrame referenceFrame = this.referenceFrame;
      Line2d bisectorLine2d = this.line.interiorBisector(secondLine.line);

      if (bisectorLine2d == null)
      {
         return null;
      }
      else
      {
         return new FrameLine2d(referenceFrame, bisectorLine2d);
      }
   }

   public FrameVector2d perpendicularFrameVector()
   {
      return new FrameVector2d(referenceFrame, line.perpendicularVector());
   }

   public static FrameLine2d perpendicularLineThroughPoint(FrameLine2d line, FramePoint2d point)
   {
      line.checkReferenceFrameMatch(point);
      ReferenceFrame referenceFrame = line.referenceFrame;
      Line2d perpLine2d = line.line.perpendicularLineThroughPoint(point.getPointCopy());

      return new FrameLine2d(referenceFrame, perpLine2d);
   }

   @Override
   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      line.applyTransformAndProjectToXYPlane(temporaryTransformToDesiredFrame);
   }

   @Override
   public FrameLine2d applyTransformCopy(Transform transform)
   {
      FrameLine2d copy = new FrameLine2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   @Override
   public FrameLine2d applyTransformAndProjectToXYPlaneCopy(Transform transform)
   {
      FrameLine2d copy = new FrameLine2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

   @Override
   public void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      if (desiredFrame == referenceFrame)
         return; // this is in the correct frame already

      if (temporaryTransformToDesiredFrame == null)
         temporaryTransformToDesiredFrame = new RigidBodyTransform();

      referenceFrame.getTransformToDesiredFrame(temporaryTransformToDesiredFrame, desiredFrame);
      applyTransformAndProjectToXYPlane(temporaryTransformToDesiredFrame);
      referenceFrame = desiredFrame;
   }

   @Override
   public FrameLine2d changeFrameAndProjectToXYPlaneCopy(ReferenceFrame desiredFrame)
   {
      FrameLine2d copy = new FrameLine2d(this);
      copy.changeFrameAndProjectToXYPlane(desiredFrame);
      return copy;
   }

   /**
    * applyTransformCopy
    * Use of this method is discouraged. Only use it to speed up computation
    * FrameLine2ds en masse, and only when the desired reference frame is not a
    * parent or child of the current reference frame.
    *
    * @param transform Transform3D
    * @param newFrame ReferenceFrame
    * @param requirePlanarTransform boolean
    * @return FrameConvexPolygon2d
    */
   public FrameLine2d applyTransformCopy(Transform transform, ReferenceFrame newFrame)
   {
      return new FrameLine2d(newFrame, this.line.applyTransformCopy(transform));
   }

   public FrameLine2d applyTransformAndProjectToXYPlaneCopy(Transform transform, ReferenceFrame newFrame)
   {
      return new FrameLine2d(newFrame, this.line.applyTransformAndProjectToXYPlaneCopy(transform));
   }

   @Override
   public String toString()
   {
      return "" + this.line;
   }

   @Override
   public void orthogonalProjection(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);
      Point2D projected = line.orthogonalProjectionCopy(point.getPoint());
      point.set(projected.getX(), projected.getY());
   }

   @Override
   public FramePoint2d orthogonalProjectionCopy(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);
      Point2D projected = line.orthogonalProjectionCopy(point.getPoint());

      return new FramePoint2d(point.getReferenceFrame(), projected);
   }
   
   public void getIntersectionWithLine(FrameLine2d line, FramePoint2d intersectionToPack)
   {
      checkReferenceFrameMatch(line);
      checkReferenceFrameMatch(intersectionToPack);
      
      this.line.intersectionWith(line.getLine2d(), intersectionToPack.getPoint());
   }

   @Override
   public FramePoint2d intersectionWith(FrameLine2d secondLine)
   {
      checkReferenceFrameMatch(secondLine);
      Point2D intersection = this.line.intersectionWith(secondLine.line);

      return new FramePoint2d(secondLine.getReferenceFrame(), intersection);
   }

   @Override
   public FramePoint2d intersectionWith(FrameLineSegment2d lineSegment)
   {
      checkReferenceFrameMatch(lineSegment);
      Point2D intersection = this.line.intersectionWith(lineSegment.lineSegment);

      return new FramePoint2d(lineSegment.getReferenceFrame(), intersection);
   }

   @Override
   public FramePoint2d[] intersectionWith(FrameConvexPolygon2d convexPolygon)
   {
      checkReferenceFrameMatch(convexPolygon);
      Point2D[] intersection = this.line.intersectionWith(convexPolygon.convexPolygon);
      if (intersection == null)
      {
         return null;
      }

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

      return this.line.distance(point.getPoint());
   }

   @Override
   public double distance(FrameLine2d secondLine)
   {
      checkReferenceFrameMatch(secondLine);

      return this.line.distance(secondLine.line);
   }

   @Override
   public double distance(FrameLineSegment2d lineSegment)
   {
      checkReferenceFrameMatch(lineSegment);

      return this.line.distance(lineSegment.lineSegment);
   }

   @Override
   public double distance(FrameConvexPolygon2d convexPolygon)
   {
      checkReferenceFrameMatch(convexPolygon);

      return this.line.distance(convexPolygon.convexPolygon);
   }

   public boolean isPointOnLine(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);

      return this.line.isPointOnLine(point.getPoint());
   }

   public boolean areLinesPerpendicular(FrameLine2d line)
   {
      checkReferenceFrameMatch(line);

      return this.line.areLinesPerpendicular(line.getLine2d());
   }

   public boolean isPointOnLeftSideOfLine(FramePoint2d point)
   {
      return isPointOnSideOfLine(point, RobotSide.LEFT);
   }

   public boolean isPointOnRightSideOfLine(FramePoint2d point)
   {
      return isPointOnSideOfLine(point, RobotSide.RIGHT);
   }

   public boolean isPointOnSideOfLine(FramePoint2d point, RobotSide side)
   {
      checkReferenceFrameMatch(point);

      return line.isPointOnSideOfLine(point.tuple, side);
   }
   
   public boolean isPointInFrontOfLine(FrameVector2d frontDirection, FramePoint2d framePoint)
   {
      checkReferenceFrameMatch(frontDirection);
      checkReferenceFrameMatch(framePoint);
      
      return line.isPointInFrontOfLine(frontDirection.getVector(), framePoint.getPoint());
   }

   /**
    * isPointInFrontOfLine
    * returns whether the point is in front of the line or not. The front
    * direction is defined as the positive x-direction
    *
    * @param point FramePoint2d
    * @return boolean
    */
   public boolean isPointInFrontOfLine(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);

      return this.line.isPointInFrontOfLine(point.getPointCopy());
   }

   public boolean isPointBehindLine(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);

      return this.line.isPointBehindLine(point.getPointCopy());
   }

   public void setParallelLineThroughPoint(FramePoint2d point)
   {
      checkReferenceFrameMatch(point);
      this.line.setParallelLineThroughPoint(point.getPointCopy());
   }

   public void shiftToLeft(double distanceToShift)
   {
      this.line.shiftToLeft(distanceToShift);
   }

   public void shiftToRight(double distanceToShift)
   {
      this.line.shiftToRight(distanceToShift);
   }

   public static FrameLine2d generateRandomFrameLine2d(Random random, ReferenceFrame zUpFrame, double xMin, double xMax, double yMin, double yMax)
   {
      FramePoint2d randomPoint = FramePoint2d.generateRandomFramePoint2d(random, zUpFrame, xMin, xMax, yMin, yMax);
      FrameVector2d randomVector = FrameVector2d.generateRandomFrameVector2d(random, zUpFrame);

      return new FrameLine2d(randomPoint, randomVector);
   }
}
