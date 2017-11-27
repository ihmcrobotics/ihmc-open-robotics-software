package us.ihmc.robotics.geometry;

import java.util.Random;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * @author Twan Koolen
 */
public class FrameLine2d extends FrameGeometryObject<FrameLine2d, Line2D>
{
   private RigidBodyTransform temporaryTransformToDesiredFrame;
   protected final Line2D line;

   public FrameLine2d()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameLine2d(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, new Line2D(new Point2D(), new Vector2D(1.0, 0.0)));
   }

   public FrameLine2d(ReferenceFrame referenceFrame, Line2D line2d)
   {
      super(referenceFrame, line2d);
      this.line = getGeometryObject();
   }

   public FrameLine2d(ReferenceFrame referenceFrame, Point2DReadOnly point2d, Vector2DReadOnly vector2d)
   {
      this(referenceFrame, new Line2D(point2d, vector2d));
   }

   public FrameLine2d(ReferenceFrame referenceFrame, Point2DReadOnly firstPointOnLine, Point2DReadOnly secondPointOnLine)
   {
      this(referenceFrame, new Line2D(firstPointOnLine, secondPointOnLine));
   }

   public FrameLine2d(FramePoint2D framePoint2d, FrameVector2D frameVector2d)
   {
      this(framePoint2d.getReferenceFrame(), new Line2D(framePoint2d, frameVector2d));
      framePoint2d.checkReferenceFrameMatch(frameVector2d);
   }

   public FrameLine2d(FramePoint2D firstPointOnLine, FramePoint2D secondPointOnLine)
   {
      this(firstPointOnLine.getReferenceFrame(), new Line2D(firstPointOnLine, secondPointOnLine));
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
   }

   public FrameLine2d(FrameLine2d frameLine2d)
   {
      this(frameLine2d.getReferenceFrame(), new Line2D(frameLine2d.line));
   }

   public FrameLine2d(FrameLineSegment2d frameLineSegment2d)
   {
      this(frameLineSegment2d.getReferenceFrame(), frameLineSegment2d.lineSegment.getFirstEndpoint(), frameLineSegment2d.lineSegment.getSecondEndpoint());
   }

   public Line2D getLine2d()
   {
      return line;
   }

   public Line2D getLine2dCopy()
   {
      return new Line2D(line);
   }

   public void getPoint2d(Point2DBasics origin)
   {
      line.getPoint(origin);
   }

   public void getFramePoint2d(FramePoint2D framePoint2d)
   {
      framePoint2d.setToZero(referenceFrame);
      line.getPoint(framePoint2d);
   }

   public void getNormalizedFrameVector(Vector2DBasics vector2dToPack)
   {
      vector2dToPack.set(line.getDirection());
   }

   public void getNormalizedFrameVector(FrameVector2D frameVector2dToPack)
   {
      frameVector2dToPack.setToZero(referenceFrame);
      frameVector2dToPack.set(line.getDirection());
   }

   public FramePoint2D getFramePoint2dGivenParameter(double t)
   {
      return new FramePoint2D(referenceFrame, line.pointOnLineGivenParameter(t));
   }

   public double getParameterGivenPointEpsilon(FramePoint2D point, double epsilon)
   {
      return line.parameterGivenPointOnLine(point.getPoint(), epsilon);
   }

   public boolean containsEpsilon(FramePoint2D framePoint, double epsilon)
   {
      framePoint.checkReferenceFrameMatch(referenceFrame);

      return line.isPointOnLine(framePoint, epsilon);
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

   public void setPoint(FramePoint2D framePoint2d)
   {
      checkReferenceFrameMatch(framePoint2d);

      line.setPoint(framePoint2d.getPoint());
   }

   public void setVector(FrameVector2D frameVector2d)
   {
      checkReferenceFrameMatch(frameVector2d);

      line.setDirection(frameVector2d);
   }

   @Deprecated
   public void setFramePoint2d(FramePoint2D framePoint2d)
   {
      setPoint(framePoint2d);
   }

   public void setByProjectionOntoXYPlane(FramePoint3D startPoint, FramePoint3D endPoint)
   {
      checkReferenceFrameMatch(startPoint);
      checkReferenceFrameMatch(endPoint);

      double x = startPoint.getX();
      double y = startPoint.getY();
      double dx = endPoint.getX() - x;
      double dy = endPoint.getY() - y;
      line.set(x, y, dx, dy);
   }

   public void set(FramePoint2D endpoint0, FramePoint2D endpoint1)
   {
      checkReferenceFrameMatch(endpoint0);
      checkReferenceFrameMatch(endpoint1);
      this.line.set(endpoint0.getPoint(), endpoint1.getPoint());
   }

   public void set(FramePoint2D startPoint, FrameVector2D vector)
   {
      checkReferenceFrameMatch(startPoint);
      checkReferenceFrameMatch(vector);
      this.line.set(startPoint, vector);
   }

   public void set(FramePoint2D[] endpoints)
   {
      checkReferenceFrameMatch(endpoints[0]);
      checkReferenceFrameMatch(endpoints[1]);

      this.line.set(endpoints[0].getPoint(), endpoints[1].getPoint());
   }

   public void setIncludingFrame(FramePoint2D endpoint0, FramePoint2D endpoint1)
   {
      endpoint0.checkReferenceFrameMatch(endpoint1);
      this.referenceFrame = endpoint0.getReferenceFrame();
      this.line.set(endpoint0.getPoint(), endpoint1.getPoint());
   }

   public void setIncludingFrame(FramePoint2D startPoint, FrameVector2D vector)
   {
      startPoint.checkReferenceFrameMatch(vector);
      this.referenceFrame = startPoint.getReferenceFrame();
      this.line.set(startPoint, vector);
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

   public FrameLine2d interiorBisector(FrameLine2d secondLine)
   {
      this.checkReferenceFrameMatch(secondLine);
      ReferenceFrame referenceFrame = this.referenceFrame;
      Line2D bisectorLine2d = this.line.interiorBisector(secondLine.line);

      if (bisectorLine2d == null)
      {
         return null;
      }
      else
      {
         return new FrameLine2d(referenceFrame, bisectorLine2d);
      }
   }

   public FrameVector2D perpendicularFrameVector()
   {
      return new FrameVector2D(referenceFrame, line.perpendicularVector());
   }

   public static FrameLine2d perpendicularLineThroughPoint(FrameLine2d line, FramePoint2D point)
   {
      line.checkReferenceFrameMatch(point);
      ReferenceFrame referenceFrame = line.referenceFrame;
      Line2D perpLine2d = line.line.perpendicularLineThroughPoint(point);

      return new FrameLine2d(referenceFrame, perpLine2d);
   }

   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      line.applyTransformAndProjectToXYPlane(temporaryTransformToDesiredFrame);
   }

   public FrameLine2d applyTransformCopy(Transform transform)
   {
      FrameLine2d copy = new FrameLine2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   public FrameLine2d applyTransformAndProjectToXYPlaneCopy(Transform transform)
   {
      FrameLine2d copy = new FrameLine2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

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

   public boolean orthogonalProjection(FramePoint2D point)
   {
      checkReferenceFrameMatch(point);
      return line.orthogonalProjection(point);
   }

   public FramePoint2D orthogonalProjectionCopy(FramePoint2D point)
   {
      checkReferenceFrameMatch(point);
      Point2D projected = line.orthogonalProjectionCopy(point.getPoint());

      return new FramePoint2D(point.getReferenceFrame(), projected);
   }

   public void getIntersectionWithLine(FrameLine2d line, FramePoint2D intersectionToPack)
   {
      checkReferenceFrameMatch(line);
      checkReferenceFrameMatch(intersectionToPack);

      this.line.intersectionWith(line.getLine2d(), intersectionToPack);
   }

   public FramePoint2D intersectionWith(FrameLine2d secondLine)
   {
      checkReferenceFrameMatch(secondLine);
      Point2D intersection = this.line.intersectionWith(secondLine.line);

      return new FramePoint2D(secondLine.getReferenceFrame(), intersection);
   }

   public FramePoint2D intersectionWith(FrameLineSegment2d lineSegment)
   {
      checkReferenceFrameMatch(lineSegment);
      Point2D intersection = this.line.intersectionWith(lineSegment.lineSegment);

      return new FramePoint2D(lineSegment.getReferenceFrame(), intersection);
   }

   public FramePoint2D[] intersectionWith(FrameConvexPolygon2d convexPolygon)
   {
      checkReferenceFrameMatch(convexPolygon);
      Point2D[] intersection = this.line.intersectionWith(convexPolygon.convexPolygon);
      if (intersection == null)
      {
         return null;
      }

      FramePoint2D[] ret = new FramePoint2D[intersection.length];
      for (int i = 0; i < intersection.length; i++)
      {
         ret[i] = new FramePoint2D(convexPolygon.getReferenceFrame(), intersection[i]);
      }

      return ret;
   }

   public double distance(FramePoint2D point)
   {
      checkReferenceFrameMatch(point);

      return this.line.distance(point.getPoint());
   }

   public boolean isPointOnLine(FramePoint2D point)
   {
      checkReferenceFrameMatch(point);

      return this.line.isPointOnLine(point.getPoint());
   }

   public boolean areLinesPerpendicular(FrameLine2d line)
   {
      checkReferenceFrameMatch(line);

      return this.line.areLinesPerpendicular(line.getLine2d());
   }

   public boolean isPointOnLeftSideOfLine(FramePoint2D point)
   {
      return isPointOnSideOfLine(point, RobotSide.LEFT);
   }

   public boolean isPointOnRightSideOfLine(FramePoint2D point)
   {
      return isPointOnSideOfLine(point, RobotSide.RIGHT);
   }

   public boolean isPointOnSideOfLine(FramePoint2D point, RobotSide side)
   {
      checkReferenceFrameMatch(point);

      return line.isPointOnSideOfLine(point, side == RobotSide.LEFT);
   }

   public boolean isPointInFrontOfLine(FrameVector2D frontDirection, FramePoint2D framePoint)
   {
      checkReferenceFrameMatch(frontDirection);
      checkReferenceFrameMatch(framePoint);

      return line.isPointInFrontOfLine(frontDirection, framePoint);
   }

   /**
    * isPointInFrontOfLine
    * returns whether the point is in front of the line or not. The front
    * direction is defined as the positive x-direction
    *
    * @param point FramePoint2d
    * @return boolean
    */
   public boolean isPointInFrontOfLine(FramePoint2D point)
   {
      checkReferenceFrameMatch(point);

      return this.line.isPointInFrontOfLine(point);
   }

   public boolean isPointBehindLine(FramePoint2D point)
   {
      checkReferenceFrameMatch(point);

      return this.line.isPointBehindLine(point);
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
      FramePoint2D randomPoint = EuclidFrameRandomTools.nextFramePoint2D(random, zUpFrame, xMin, xMax, yMin, yMax);
      FrameVector2D randomVector = EuclidFrameRandomTools.nextFrameVector2D(random, zUpFrame);

      return new FrameLine2d(randomPoint, randomVector);
   }
}
