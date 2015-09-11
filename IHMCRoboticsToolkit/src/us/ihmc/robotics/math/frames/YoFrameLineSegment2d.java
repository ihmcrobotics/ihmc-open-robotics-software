package us.ihmc.robotics.math.frames;

import javax.vecmath.Point2d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

//Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
//since they contain YoVariables.
public class YoFrameLineSegment2d extends ReferenceFrameHolder
{
   private final DoubleYoVariable x0, y0, x1, y1; // This is where the data is stored. All operations must act on these numbers.
   private final ReferenceFrame referenceFrame;
   protected FrameLineSegment2d frameLineSegment; // This is only for assistance. The data is stored in the YoVariables, not in here!

   public YoFrameLineSegment2d(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(namePrefix, nameSuffix, "", frame, registry);
   }

   public YoFrameLineSegment2d(String namePrefix, String nameSuffix, String description, ReferenceFrame frame, YoVariableRegistry registry)
   {
      x0 = new DoubleYoVariable(namePrefix + "X0" + nameSuffix, description, registry);
      y0 = new DoubleYoVariable(namePrefix + "Y0" + nameSuffix, description, registry);
      x1 = new DoubleYoVariable(namePrefix + "X1" + nameSuffix, description, registry);
      y1 = new DoubleYoVariable(namePrefix + "Y1" + nameSuffix, description, registry);

      this.referenceFrame = frame;
      frameLineSegment = new FrameLineSegment2d(referenceFrame);
   }

   public YoFrameLineSegment2d(DoubleYoVariable x0Variable, DoubleYoVariable y0Variable, DoubleYoVariable x1Variable, DoubleYoVariable y1Variable,
         ReferenceFrame frame)
   {
      this.x0 = x0Variable;
      this.y0 = y0Variable;
      this.x1 = x1Variable;
      this.y1 = y1Variable;

      this.referenceFrame = frame;
      frameLineSegment = new FrameLineSegment2d(referenceFrame);
   }

   @Override
   public String toString()
   {
      return "(" + x0.getDoubleValue() + ", " + y0.getDoubleValue() + "), (" + x1.getDoubleValue() + ", " + y1.getDoubleValue() + ")-" + referenceFrame;
   }

   public double getX0()
   {
      return x0.getDoubleValue();
   }

   public double getX1()
   {
      return x1.getDoubleValue();
   }

   public double getY0()
   {
      return y0.getDoubleValue();
   }

   public double getY1()
   {
      return y1.getDoubleValue();
   }

   public DoubleYoVariable getYoX0()
   {
      return x0;
   }

   public DoubleYoVariable getYoX1()
   {
      return x1;
   }

   public DoubleYoVariable getYoY0()
   {
      return y0;
   }

   public DoubleYoVariable getYoY1()
   {
      return y1;
   }

   public void setFrameLineSegment2d(FrameLineSegment2d lineSegment)
   {
      if (lineSegment == null)
      {
         x0.set(Double.NaN);
         y0.set(Double.NaN);
         x1.set(Double.NaN);
         y1.set(Double.NaN);

         return;
      }

      lineSegment.checkReferenceFrameMatch(referenceFrame);

      this.frameLineSegment.set(lineSegment);
      getYoValuesFromFrameLineSegment();
   }

   public void set(FramePoint2d endpoint0, FramePoint2d endpoint1)
   {
      referenceFrame.checkReferenceFrameMatch(endpoint0.getReferenceFrame());
      referenceFrame.checkReferenceFrameMatch(endpoint1.getReferenceFrame());
      frameLineSegment.set(endpoint0, endpoint1);
      getYoValuesFromFrameLineSegment();
   }

   public void set(FramePoint2d endpoint0, FrameVector2d fromPoint0ToPoint1)
   {
      referenceFrame.checkReferenceFrameMatch(endpoint0.getReferenceFrame());
      referenceFrame.checkReferenceFrameMatch(fromPoint0ToPoint1.getReferenceFrame());
      frameLineSegment.set(endpoint0, fromPoint0ToPoint1);
      getYoValuesFromFrameLineSegment();
   }

   public FrameLineSegment2d getFrameLineSegment2d()
   {
      putYoValuesIntoFrameLineSegment();

      return new FrameLineSegment2d(frameLineSegment);
   }

   public FramePoint2d midpoint()
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.midpoint();
   }

   public void getFirstEndPoint(FramePoint2d firstEndPointToPack)
   {
      putYoValuesIntoFrameLineSegment();

      frameLineSegment.getFirstEndPoint(firstEndPointToPack);
   }

   public void getSecondEndPoint(FramePoint2d secondEndPointToPack)
   {
      putYoValuesIntoFrameLineSegment();

      frameLineSegment.getSecondEndPoint(secondEndPointToPack);
   }

   public void getFirstEndPoint(Point2d firstEndPointToPack)
   {
      frameLineSegment.getFirstEndPoint(firstEndPointToPack);
   }

   public void getSecondEndPoint(Point2d secondEndPointToPack)
   {
      putYoValuesIntoFrameLineSegment();

      frameLineSegment.getSecondEndPoint(secondEndPointToPack);
   }

   public double length()
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.length();
   }

   public boolean isBetweenEndpoints(FramePoint2d point, double epsilon)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.isBetweenEndpoints(point, epsilon);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void orthogonalProjection(FramePoint2d point)
   {
      putYoValuesIntoFrameLineSegment();
      frameLineSegment.orthogonalProjection(point);
   }

   public FramePoint2d orthogonalProjectionCopy(FramePoint2d point)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.orthogonalProjectionCopy(point);
   }

   public FramePoint2d intersectionWith(FrameLine2d line)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.intersectionWith(line);
   }

   public FramePoint2d intersectionWith(FrameLineSegment2d secondLineSegment)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.intersectionWith(secondLineSegment);
   }

   public FramePoint2d[] intersectionWith(FrameConvexPolygon2d convexPolygon)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.intersectionWith(convexPolygon);
   }

   public double distance(FramePoint2d point)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.distance(point);
   }

   public double distance(FrameLine2d line)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.distance(line);
   }

   public double distance(FrameLineSegment2d secondLineSegment)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.distance(secondLineSegment);
   }

   public double distance(FrameConvexPolygon2d convexPolygon)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.distance(convexPolygon);
   }

   public FramePoint2d pointBetweenEndPointsGivenParameter(double parameter)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.pointBetweenEndPointsGivenParameter(parameter);
   }

   public void setToNaN()
   {
      x0.set(Double.NaN);
      y0.set(Double.NaN);
      x1.set(Double.NaN);
      y1.set(Double.NaN);
   }

   public boolean containsNaN()
   {
      return Double.isNaN(getX0()) || Double.isNaN(getY0()) || Double.isNaN(getX1()) || Double.isNaN(getY1());
   }

   private void putYoValuesIntoFrameLineSegment()
   {
      frameLineSegment.set(referenceFrame, x0.getDoubleValue(), y0.getDoubleValue(), x1.getDoubleValue(), y1.getDoubleValue());
   }

   private void getYoValuesFromFrameLineSegment()
   {
      LineSegment2d lineSegment2d = frameLineSegment.getLineSegment2d();
      x0.set(lineSegment2d.getX0());
      y0.set(lineSegment2d.getY0());
      x1.set(lineSegment2d.getX1());
      y1.set(lineSegment2d.getY1());
   }

   public DoubleYoVariable[] getDoubleYoVariables()
   {
      return new DoubleYoVariable[] { x0, y0, x1, y1 };
   }
}
