package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameLine3D extends FrameGeometryObject<FrameLine3D, Line3D>
{
   private final Line3D line3D;

   public FrameLine3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameLine3D(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Line3D());
      line3D = getGeometryObject();
   }

   public FrameLine3D(FramePoint point, FrameVector vector)
   {
      this(point.getReferenceFrame(), point.getGeometryObject(), vector.getGeometryObject());
      point.checkReferenceFrameMatch(vector);
   }

   public FrameLine3D(ReferenceFrame referenceFrame, Point3DReadOnly point, Vector3DReadOnly vector)
   {
      super(referenceFrame, new Line3D(point, vector));
      line3D = getGeometryObject();
   }

   public FrameLine3D(FrameLine3D frameLine)
   {
      this(frameLine.getReferenceFrame(), frameLine.getGeometryObject());
   }

   public FrameLine3D(ReferenceFrame referenceFrame, Line3D line)
   {
      super(referenceFrame, new Line3D(line));
      line3D = getGeometryObject();
   }

   public FrameVector getFrameNormalizedVectorCopy()
   {
      return new FrameVector(referenceFrame, line3D.getDirection());
   }

   public Point3DReadOnly getPoint()
   {
      return line3D.getPoint();
   }

   public Vector3DReadOnly getDirection()
   {
      return line3D.getDirection();
   }

   public Point3D getPointCopy()
   {
      return new Point3D(line3D.getPoint());
   }

   public Vector3D getDirectionCopy()
   {
      return new Vector3D(line3D.getDirection());
   }

   public void setPoint(double pointOnLineX, double pointOnLineY, double pointOnLineZ)
   {
      line3D.setPoint(pointOnLineX, pointOnLineY, pointOnLineZ);
   }

   public void setPoint(Point3DReadOnly pointOnLine)
   {
      line3D.setPoint(pointOnLine);
   }

   public void setPoint(FramePoint pointOnLine)
   {
      checkReferenceFrameMatch(pointOnLine);
      line3D.setPoint(pointOnLine.getPoint());
   }

   public void setDirection(double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      line3D.setDirection(lineDirectionX, lineDirectionY, lineDirectionZ);
   }

   public void setDirection(Vector3DReadOnly lineDirection)
   {
      line3D.setDirection(lineDirection);
   }

   public void setDirection(FrameVector lineDirection)
   {
      checkReferenceFrameMatch(lineDirection);
      line3D.setDirection(lineDirection.getVector());
   }

   public void projectOntoXYPlane(Line2D lineToPack)
   {
      lineToPack.set(line3D.getPointX(), line3D.getPointY(), line3D.getDirectionX(), line3D.getDirectionY());
   }

   public void projectOntoXYPlane(FrameLine2d lineToPack)
   {
      lineToPack.set(getReferenceFrame(), line3D.getPointX(), line3D.getPointY(), line3D.getDirectionX(), line3D.getDirectionY());
   }
}
