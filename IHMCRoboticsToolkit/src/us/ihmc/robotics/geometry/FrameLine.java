package us.ihmc.robotics.geometry;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameLine extends AbstractFrameObject<FrameLine, Line3d>
{
   private final Line3d line;

   public FrameLine()
   {
      this(ReferenceFrame.getWorldFrame());
   }
   
   public FrameLine(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, new Line3d());
   }
   
   public FrameLine(FramePoint point, FrameVector vector)
   {
      this(point.getReferenceFrame(), new Line3d(point.getGeometryObject(), vector.getGeometryObject()));
      point.checkReferenceFrameMatch(vector);
   }

   public FrameLine(ReferenceFrame referenceFrame, Point3d point, Vector3d vector)
   {
      this(referenceFrame, new Line3d(point, vector));
   }

   public FrameLine(FrameLine frameLine)
   {
      this(frameLine.getReferenceFrame(), new Line3d(frameLine.getPoint(), frameLine.getNormalizedVector()));
   }

   public FrameLine(ReferenceFrame referenceFrame, Line3d line)
   {
      super(referenceFrame, line);
      this.line = getGeometryObject();
   }

   public FrameVector getFrameNormalizedVectorCopy()
   {
      return new FrameVector(referenceFrame, line.getNormalizedVector());
   }

   public Point3d getPoint()
   {
      return line.getPoint();
   }

   public Vector3d getNormalizedVector()
   {
      return line.getNormalizedVector();
   }

   public Point3d getPointCopy()
   {
      return new Point3d(line.getPoint());
   }

   public Vector3d getNormalizedVectorCopy()
   {
      return new Vector3d(line.getNormalizedVector());
   }

   @Override
   public boolean epsilonEquals(FrameLine otherLine, double epsilon)
   {
      checkReferenceFrameMatch(otherLine);

      return line.epsilonEquals(otherLine.getGeometryObject(), epsilon);
   }

   public void setPoint(FramePoint point)
   {
      checkReferenceFrameMatch(point);

      line.setPoint(point.getPoint());
   }
   
   public void setPointWithoutChecks(Point3d point)
   {
      line.setPoint(point);
   }

   public void setVector(FrameVector vector)
   {
      checkReferenceFrameMatch(vector);
      
      line.setVector(vector.getVector());
   }

   public void setVectorWithoutChecks(Vector3d vector)
   {
      line.setVector(vector);
   }
   
   public void projectOntoXYPlane(FrameLine2d lineToPack)
   {
      lineToPack.set(getReferenceFrame(), line.getPoint().getX(), line.getPoint().getY(), line.getNormalizedVector().getX(), line.getNormalizedVector().getY());
   }
   
   public void projectOntoXYPlane(Line2d lineToPack)
   {
      lineToPack.set(line.getPoint().getX(), line.getPoint().getY(), line.getNormalizedVector().getX(), line.getNormalizedVector().getY());
   }
   
   public void projectOntoXYPlane(Point2d pointToPack, Vector2d normalizedVectorToPack)
   {
      pointToPack.set(line.getPoint().getX(), line.getPoint().getY());
      normalizedVectorToPack.set(line.getNormalizedVector().getX(), line.getNormalizedVector().getY());
      if (GeometryTools.isZero(normalizedVectorToPack, 1e-12))
      {
         normalizedVectorToPack.set(Double.NaN, Double.NaN);
      }
      else
      {
         normalizedVectorToPack.normalize();
      }
   }
}
