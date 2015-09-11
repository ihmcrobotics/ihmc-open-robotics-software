package us.ihmc.robotics.math.frames;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
//since they contain YoVariables.
public class YoFrameLine2d extends ReferenceFrameHolder
{
   private final DoubleYoVariable x0, y0, vx, vy; // This is where the data is stored. All operations must act on these numbers.
   private final ReferenceFrame referenceFrame;
   protected FrameLine2d frameLine; // This is only for assistance. The data is stored in the YoVariables, not in here!

   public YoFrameLine2d(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      x0 = new DoubleYoVariable(namePrefix + "X0" + nameSuffix, registry);
      y0 = new DoubleYoVariable(namePrefix + "Y0" + nameSuffix, registry);
      vx = new DoubleYoVariable(namePrefix + "VX" + nameSuffix, registry);
      vy = new DoubleYoVariable(namePrefix + "VY" + nameSuffix, registry);

      this.referenceFrame = frame;
   }

   public YoFrameLine2d(DoubleYoVariable x0Variable, DoubleYoVariable y0Variable, DoubleYoVariable vxVariable, DoubleYoVariable vyVariable, ReferenceFrame frame)
   {
      this.x0 = x0Variable;
      this.y0 = y0Variable;
      this.vx = vxVariable;
      this.vy = vyVariable;

      this.referenceFrame = frame;
   }

   public double getX0()
   {
      return x0.getDoubleValue();
   }

   public double getY0()
   {
      return y0.getDoubleValue();
   }

   public double getVx()
   {
      return vx.getDoubleValue();
   }

   public double getVy()
   {
      return vy.getDoubleValue();
   }

   public DoubleYoVariable getYoX0()
   {
      return x0;
   }

   public DoubleYoVariable getYoY0()
   {
      return y0;
   }

   public DoubleYoVariable getYoVx()
   {
      return vx;
   }

   public DoubleYoVariable getYoVy()
   {
      return vy;
   }

   @Override
   public String toString()
   {
      return "(" + x0.getDoubleValue() + ", " + y0.getDoubleValue() + "), (" + vx.getDoubleValue() + ", " + vy.getDoubleValue() + ")-" + referenceFrame;
   }

   public void setFrameLine2d(FrameLine2d frameLine2d)
   {
      if (frameLine2d == null)
      {
         x0.set(Double.NaN);
         y0.set(Double.NaN);
         vx.set(Double.NaN);
         vy.set(Double.NaN);

         return;
      }

      frameLine2d.checkReferenceFrameMatch(referenceFrame);

      Line2d line = frameLine2d.getLine2d();

      x0.set(line.getPoint().x);
      y0.set(line.getPoint().y);
      vx.set(line.getNormalizedVector().x);
      vy.set(line.getNormalizedVector().y);
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
         frameLine = new FrameLine2d(referenceFrame, new Point2d(x0.getDoubleValue(), y0.getDoubleValue()), new Vector2d(vx.getDoubleValue(),
               vy.getDoubleValue()));
      }
      else
      {
         frameLine.set(referenceFrame, x0.getDoubleValue(), y0.getDoubleValue(), vx.getDoubleValue(), vy.getDoubleValue());
      }
   }
}
