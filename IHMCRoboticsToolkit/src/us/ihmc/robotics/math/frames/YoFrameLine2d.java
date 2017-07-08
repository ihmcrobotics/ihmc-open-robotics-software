package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
//since they contain YoVariables.
public class YoFrameLine2d implements ReferenceFrameHolder
{
   private final YoDouble pointX, pointY, vectorX, vectorY; // This is where the data is stored. All operations must act on these numbers.
   private final ReferenceFrame referenceFrame;
   protected FrameLine2d frameLine; // This is only for assistance. The data is stored in the YoVariables, not in here!

   public YoFrameLine2d(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      pointX = new YoDouble(namePrefix + "PointX" + nameSuffix, registry);
      pointY = new YoDouble(namePrefix + "PointY" + nameSuffix, registry);
      vectorX = new YoDouble(namePrefix + "VectorX" + nameSuffix, registry);
      vectorY = new YoDouble(namePrefix + "VectorY" + nameSuffix, registry);

      this.referenceFrame = frame;
   }

   public YoFrameLine2d(YoDouble pointX, YoDouble pointY, YoDouble vectorX, YoDouble vectorY, ReferenceFrame frame)
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

   public YoDouble getYoPointX()
   {
      return pointX;
   }

   public YoDouble getYoPointY()
   {
      return pointY;
   }

   public YoDouble getYoVectorX()
   {
      return vectorX;
   }

   public YoDouble getYoVectorY()
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

      Line2D line = frameLine2d.getLine2d();

      pointX.set(line.getPoint().getX());
      pointY.set(line.getPoint().getY());
      vectorX.set(line.getDirection().getX());
      vectorY.set(line.getDirection().getY());
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

   public void orthogonalProjection(FramePoint2D point)
   {
      putYoValuesIntoFrameLine();
      frameLine.orthogonalProjection(point);
   }

   public FramePoint2D orthogonalProjectionCopy(FramePoint2D point)
   {
      putYoValuesIntoFrameLine();

      return frameLine.orthogonalProjectionCopy(point);
   }

   public FramePoint2D intersectionWith(FrameLine2d line)
   {
      putYoValuesIntoFrameLine();

      return frameLine.intersectionWith(line);
   }

   public FramePoint2D intersectionWith(FrameLineSegment2d secondLineSegment)
   {
      putYoValuesIntoFrameLine();

      return frameLine.intersectionWith(secondLineSegment);
   }

   public FramePoint2D[] intersectionWith(FrameConvexPolygon2d convexPolygon)
   {
      putYoValuesIntoFrameLine();

      return frameLine.intersectionWith(convexPolygon);
   }

   public double distance(FramePoint2D point)
   {
      putYoValuesIntoFrameLine();

      return frameLine.distance(point);
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
