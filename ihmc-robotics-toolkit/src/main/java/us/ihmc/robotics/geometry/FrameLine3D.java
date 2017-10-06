package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
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

   public FrameLine3D(double pointOnLineX, double pointOnLineY, double pointOnLineZ, double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      super(new Line3D(pointOnLineX, pointOnLineY, pointOnLineZ, lineDirectionX, lineDirectionY, lineDirectionZ));
      line3D = getGeometryObject();
   }

   public FrameLine3D(ReferenceFrame referenceFrame, double pointOnLineX, double pointOnLineY, double pointOnLineZ, double lineDirectionX,
                      double lineDirectionY, double lineDirectionZ)
   {
      super(referenceFrame, new Line3D(pointOnLineX, pointOnLineY, pointOnLineZ, lineDirectionX, lineDirectionY, lineDirectionZ));
      line3D = getGeometryObject();
   }

   public FrameLine3D(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      super(new Line3D(pointOnLine, lineDirection));
      line3D = getGeometryObject();
   }

   public FrameLine3D(ReferenceFrame referenceFrame, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      super(referenceFrame, new Line3D(pointOnLine, lineDirection));
      line3D = getGeometryObject();
   }

   public FrameLine3D(Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      super(new Line3D(firstPointOnLine, secondPointOnLine));
      line3D = getGeometryObject();
   }

   public FrameLine3D(ReferenceFrame referenceFrame, Point3DReadOnly firstPointOnLine, Point3DReadOnly secondPointOnLine)
   {
      super(referenceFrame, new Line3D(firstPointOnLine, secondPointOnLine));
      line3D = getGeometryObject();
   }

   public FrameLine3D(FramePoint3D pointOnLine, FrameVector3D lineDirection)
   {
      this(pointOnLine.getReferenceFrame(), pointOnLine.getGeometryObject(), lineDirection.getGeometryObject());
      pointOnLine.checkReferenceFrameMatch(lineDirection);
   }

   public FrameLine3D(FramePoint3D firstPointOnLine, FramePoint3D secondPointOnLine)
   {
      this(firstPointOnLine.getReferenceFrame(), firstPointOnLine.getGeometryObject(), secondPointOnLine.getGeometryObject());
      firstPointOnLine.checkReferenceFrameMatch(secondPointOnLine);
   }

   public FrameLine3D(FrameLine3D other)
   {
      this(other.getReferenceFrame(), other.getGeometryObject());
   }

   public FrameLine3D(ReferenceFrame referenceFrame, Line3D line)
   {
      super(referenceFrame, new Line3D(line));
      line3D = getGeometryObject();
   }

   public FrameVector3D getFrameNormalizedVectorCopy()
   {
      return new FrameVector3D(referenceFrame, line3D.getDirection());
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

   public void setPoint(FramePoint3D pointOnLine)
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

   public void setDirection(FrameVector3D lineDirection)
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
