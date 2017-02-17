package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
//since they contain YoVariables.
public class YoFrameLine2d extends AbstractReferenceFrameHolder
{
   private final DoubleYoVariable pointX, pointY, vectorX, vectorY; // This is where the data is stored. All operations must act on these numbers.
   private final ReferenceFrame referenceFrame;
   protected FrameLine2d frameLine; // This is only for assistance. The data is stored in the YoVariables, not in here!

   public YoFrameLine2d(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      pointX = new DoubleYoVariable(namePrefix + "PointX" + nameSuffix, registry);
      pointY = new DoubleYoVariable(namePrefix + "PointY" + nameSuffix, registry);
      vectorX = new DoubleYoVariable(namePrefix + "VectorX" + nameSuffix, registry);
      vectorY = new DoubleYoVariable(namePrefix + "VectorY" + nameSuffix, registry);

      this.referenceFrame = frame;
   }

   public YoFrameLine2d(DoubleYoVariable pointX, DoubleYoVariable pointY, DoubleYoVariable vectorX, DoubleYoVariable vectorY, ReferenceFrame frame)
   {
      this.pointX = pointX;
      this.pointY = pointY;
      this.vectorX = vectorX;
      this.vectorY = vectorY;

      this.referenceFrame = frame;
   }

   public double getPointX()
   {
      return pointX.getDoubleValue();
   }

   public double getPointY()
   {
      return pointY.getDoubleValue();
   }

   public double getVectorX()
   {
      return vectorX.getDoubleValue();
   }

   public double getVectorY()
   {
      return vectorY.getDoubleValue();
   }

   public DoubleYoVariable getYoPointX()
   {
      return pointX;
   }

   public DoubleYoVariable getYoPointY()
   {
      return pointY;
   }

   public DoubleYoVariable getYoVectorX()
   {
      return vectorX;
   }

   public DoubleYoVariable getYoVectorY()
   {
      return vectorY;
   }

   @Override
   public String toString()
   {
      return "(" + pointX.getDoubleValue() + ", " + pointY.getDoubleValue() + "), (" + vectorX.getDoubleValue() + ", " + vectorY.getDoubleValue() + ")-" + referenceFrame;
   }

   public void setFrameLine2d(FrameLine2d frameLine2d)
   {
      if (frameLine2d == null)
      {
         pointX.set(Double.NaN);
         pointY.set(Double.NaN);
         vectorX.set(Double.NaN);
         vectorY.set(Double.NaN);

         return;
      }

      frameLine2d.checkReferenceFrameMatch(referenceFrame);

      Line2d line = frameLine2d.getLine2d();

      pointX.set(line.getPoint().getX());
      pointY.set(line.getPoint().getY());
      vectorX.set(line.getNormalizedVector().getX());
      vectorY.set(line.getNormalizedVector().getY());
   }

   public FrameLine2d getFrameLine2d()
   {
      putYoValuesIntoFrameLine();

      return new FrameLine2d(frameLine);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void orthogonalProjection(FramePoint2d point)
   {
      putYoValuesIntoFrameLine();
      frameLine.orthogonalProjection(point);
   }

   public FramePoint2d orthogonalProjectionCopy(FramePoint2d point)
   {
      putYoValuesIntoFrameLine();

      return frameLine.orthogonalProjectionCopy(point);
   }

   public FramePoint2d intersectionWith(FrameLine2d line)
   {
      putYoValuesIntoFrameLine();

      return frameLine.intersectionWith(line);
   }

   public FramePoint2d intersectionWith(FrameLineSegment2d secondLineSegment)
   {
      putYoValuesIntoFrameLine();

      return frameLine.intersectionWith(secondLineSegment);
   }

   public FramePoint2d[] intersectionWith(FrameConvexPolygon2d convexPolygon)
   {
      putYoValuesIntoFrameLine();

      return frameLine.intersectionWith(convexPolygon);
   }

   public double distance(FramePoint2d point)
   {
      putYoValuesIntoFrameLine();

      return frameLine.distance(point);
   }

   public double distance(FrameLine2d line)
   {
      putYoValuesIntoFrameLine();

      return frameLine.distance(line);
   }

   public double distance(FrameLineSegment2d secondLineSegment)
   {
      putYoValuesIntoFrameLine();

      return frameLine.distance(secondLineSegment);
   }

   public double distance(FrameConvexPolygon2d convexPolygon)
   {
      putYoValuesIntoFrameLine();

      return frameLine.distance(convexPolygon);
   }

   protected void putYoValuesIntoFrameLine()
   {
      if (frameLine == null)
      {
         frameLine = new FrameLine2d(referenceFrame, new Point2D(pointX.getDoubleValue(), pointY.getDoubleValue()), new Vector2D(vectorX.getDoubleValue(),
               vectorY.getDoubleValue()));
      }
      else
      {
         frameLine.set(referenceFrame, pointX.getDoubleValue(), pointY.getDoubleValue(), vectorX.getDoubleValue(), vectorY.getDoubleValue());
      }
   }
}
