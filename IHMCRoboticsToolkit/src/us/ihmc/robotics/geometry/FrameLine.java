package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameLine extends AbstractFrameObject<FrameLine, Line3D>
{
   private final Line3D line;

   public FrameLine()
   {
      this(ReferenceFrame.getWorldFrame());
   }
   
   public FrameLine(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, new Line3D());
   }
   
   public FrameLine(FramePoint point, FrameVector vector)
   {
      this(point.getReferenceFrame(), new Line3D(point.getGeometryObject(), vector.getGeometryObject()));
      point.checkReferenceFrameMatch(vector);
   }

   public FrameLine(ReferenceFrame referenceFrame, Point3DReadOnly point, Vector3DReadOnly vector)
   {
      this(referenceFrame, new Line3D(point, vector));
   }

   public FrameLine(FrameLine frameLine)
   {
      this(frameLine.getReferenceFrame(), new Line3D(frameLine.getPoint(), frameLine.getNormalizedVector()));
   }

   public FrameLine(ReferenceFrame referenceFrame, Line3D line)
   {
      super(referenceFrame, line);
      this.line = getGeometryObject();
   }

   public FrameVector getFrameNormalizedVectorCopy()
   {
      return new FrameVector(referenceFrame, line.getDirection());
   }

   public Point3D getPoint()
   {
      return line.getPoint();
   }

   public Vector3D getNormalizedVector()
   {
      return line.getDirection();
   }

   public Point3D getPointCopy()
   {
      return new Point3D(line.getPoint());
   }

   public Vector3D getNormalizedVectorCopy()
   {
      return new Vector3D(line.getDirection());
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
   
   public void setPointWithoutChecks(Point3DReadOnly point)
   {
      line.setPoint(point);
   }

   public void setVector(FrameVector vector)
   {
      checkReferenceFrameMatch(vector);
      
      line.setDirection(vector.getVector());
   }

   public void setVectorWithoutChecks(Vector3DReadOnly vector)
   {
      line.setDirection(vector);
   }
   
   public void projectOntoXYPlane(FrameLine2d lineToPack)
   {
      lineToPack.set(getReferenceFrame(), line.getPoint().getX(), line.getPoint().getY(), line.getDirection().getX(), line.getDirection().getY());
   }
   
   public void projectOntoXYPlane(Line2d lineToPack)
   {
      lineToPack.set(line.getPoint().getX(), line.getPoint().getY(), line.getDirection().getX(), line.getDirection().getY());
   }
   
   public void projectOntoXYPlane(Point2DBasics pointToPack, Vector2DBasics normalizedVectorToPack)
   {
      pointToPack.set(line.getPoint().getX(), line.getPoint().getY());
      normalizedVectorToPack.set(line.getDirection().getX(), line.getDirection().getY());
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
