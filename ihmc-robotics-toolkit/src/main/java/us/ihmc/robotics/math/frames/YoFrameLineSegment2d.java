package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
 * since they contain YoVariables.
 */
public class YoFrameLineSegment2d implements ReferenceFrameHolder
{
   /** This is where the data is stored. All operations must act on these numbers. */
   private final YoDouble firstEndpointX, firstEndpointY, secondEndpointX, secondEndpointY;
   private final ReferenceFrame referenceFrame;
   /** This is only for assistance. The data is stored in the YoVariables, not in here! */
   protected FrameLineSegment2D frameLineSegment;

   public YoFrameLineSegment2d(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(namePrefix, nameSuffix, "", frame, registry);
   }

   public YoFrameLineSegment2d(String namePrefix, String nameSuffix, String description, ReferenceFrame frame, YoVariableRegistry registry)
   {
      firstEndpointX = new YoDouble(namePrefix + "FirstEndpointX" + nameSuffix, description, registry);
      firstEndpointY = new YoDouble(namePrefix + "FirstEndpointY" + nameSuffix, description, registry);
      secondEndpointX = new YoDouble(namePrefix + "SecondEndpointX" + nameSuffix, description, registry);
      secondEndpointY = new YoDouble(namePrefix + "SecondEndpointY" + nameSuffix, description, registry);

      this.referenceFrame = frame;
      frameLineSegment = new FrameLineSegment2D(referenceFrame);
   }

   public YoFrameLineSegment2d(YoDouble firstEndpointX, YoDouble firstEndpointY, YoDouble secondEndpointX, YoDouble secondEndpointY,
         ReferenceFrame frame)
   {
      this.firstEndpointX = firstEndpointX;
      this.firstEndpointY = firstEndpointY;
      this.secondEndpointX = secondEndpointX;
      this.secondEndpointY = secondEndpointY;

      this.referenceFrame = frame;
      frameLineSegment = new FrameLineSegment2D(referenceFrame);
   }

   @Override
   public String toString()
   {
      return "(" + firstEndpointX.getDoubleValue() + ", " + firstEndpointY.getDoubleValue() + "), (" + secondEndpointX.getDoubleValue() + ", " + secondEndpointY.getDoubleValue() + ")-" + referenceFrame;
   }

   public double getFirstEndpointX()
   {
      return firstEndpointX.getDoubleValue();
   }

   public double getFirstEndpointY()
   {
      return firstEndpointY.getDoubleValue();
   }

   public double getSecondEndpointX()
   {
      return secondEndpointX.getDoubleValue();
   }

   public double getSecondEndpointY()
   {
      return secondEndpointY.getDoubleValue();
   }

   public YoDouble getYoFirstEndpointX()
   {
      return firstEndpointX;
   }

   public YoDouble getYoFirstEndpointY()
   {
      return firstEndpointY;
   }

   public YoDouble getYoSecondEndpointX()
   {
      return secondEndpointX;
   }

   public YoDouble getYoSecondEndpointY()
   {
      return secondEndpointY;
   }

   public void setFrameLineSegment2d(FrameLineSegment2D lineSegment)
   {
      if (lineSegment == null)
      {
         firstEndpointX.set(Double.NaN);
         firstEndpointY.set(Double.NaN);
         secondEndpointX.set(Double.NaN);
         secondEndpointY.set(Double.NaN);

         return;
      }

      lineSegment.checkReferenceFrameMatch(referenceFrame);

      this.frameLineSegment.set(lineSegment);
      getYoValuesFromFrameLineSegment();
   }

   public void set(FramePoint2D firstEndpoint, FramePoint2D secondEndpoint)
   {
      referenceFrame.checkReferenceFrameMatch(firstEndpoint.getReferenceFrame());
      referenceFrame.checkReferenceFrameMatch(secondEndpoint.getReferenceFrame());
      frameLineSegment.set(firstEndpoint, secondEndpoint);
      getYoValuesFromFrameLineSegment();
   }

   public void set(FramePoint2D firstEndpoint, FrameVector2D vectorToSecondEndpoint)
   {
      referenceFrame.checkReferenceFrameMatch(firstEndpoint.getReferenceFrame());
      referenceFrame.checkReferenceFrameMatch(vectorToSecondEndpoint.getReferenceFrame());
      frameLineSegment.set(firstEndpoint, vectorToSecondEndpoint);
      getYoValuesFromFrameLineSegment();
   }
   
   public void set(Point2DReadOnly firstEndpoint, Point2DReadOnly secondEndpoint)
   {
      frameLineSegment.set(frameLineSegment.getReferenceFrame(), firstEndpoint, secondEndpoint);
      getYoValuesFromFrameLineSegment();
   }

   public FrameLineSegment2D getFrameLineSegment2d()
   {
      putYoValuesIntoFrameLineSegment();
      return new FrameLineSegment2D(frameLineSegment);
   }
   
   public void getFrameLineSegment2d(FrameLineSegment2D lineSegmentToPack)
   {
      putYoValuesIntoFrameLineSegment();
      lineSegmentToPack.setIncludingFrame(frameLineSegment);
   }

   public FramePoint2DBasics midpoint()
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.midpoint();
   }

   public void midpoint(FramePoint2D midpoint)
   {
      putYoValuesIntoFrameLineSegment();
      frameLineSegment.midpoint(midpoint);
   }

   public void getFirstEndPoint(FramePoint2D firstEndpointToPack)
   {
      putYoValuesIntoFrameLineSegment();

      firstEndpointToPack.setIncludingFrame(frameLineSegment.getFirstEndpoint());
   }

   public void getSecondEndPoint(FramePoint2D secondEndpointToPack)
   {
      putYoValuesIntoFrameLineSegment();

      secondEndpointToPack.setIncludingFrame(frameLineSegment.getSecondEndpoint());
   }

   public void getFirstEndPoint(Point2D firstEndpointToPack)
   {
      putYoValuesIntoFrameLineSegment();
      
      firstEndpointToPack.set(frameLineSegment.getFirstEndpoint());
   }

   public void getSecondEndPoint(Point2D secondEndpointToPack)
   {
      putYoValuesIntoFrameLineSegment();

      secondEndpointToPack.set(frameLineSegment.getSecondEndpoint());
   }

   public double length()
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.length();
   }

   public boolean isBetweenEndpoints(FramePoint2D point, double epsilon)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.isBetweenEndpoints(point, epsilon);
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public void orthogonalProjection(FramePoint2D point)
   {
      putYoValuesIntoFrameLineSegment();
      frameLineSegment.orthogonalProjection(point);
   }

   public FramePoint2D orthogonalProjectionCopy(FramePoint2D point)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.orthogonalProjectionCopy(point);
   }

   public FramePoint2D intersectionWith(FrameLine2D line)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.intersectionWith(line);
   }

   public FramePoint2D intersectionWith(FrameLineSegment2D secondLineSegment)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.intersectionWith(secondLineSegment);
   }

   public FramePoint2D[] intersectionWith(FrameConvexPolygon2d convexPolygon)
   {
      putYoValuesIntoFrameLineSegment();

      return convexPolygon.intersectionWith(frameLineSegment);
   }

   public double distance(FramePoint2D point)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.distance(point);
   }

   public FramePoint2DBasics pointBetweenEndPointsGivenParameter(double parameter)
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.pointBetweenEndpointsGivenPercentage(parameter);
   }
   
   public boolean areEndpointsTheSame()
   {
      return firstEndpointX.getDoubleValue() == secondEndpointX.getDoubleValue() && firstEndpointY.getDoubleValue() == secondEndpointY.getDoubleValue();
   }

   public void setToNaN()
   {
      firstEndpointX.set(Double.NaN);
      firstEndpointY.set(Double.NaN);
      secondEndpointX.set(Double.NaN);
      secondEndpointY.set(Double.NaN);
   }

   public boolean containsNaN()
   {
      return Double.isNaN(getFirstEndpointX()) || Double.isNaN(getFirstEndpointY()) || Double.isNaN(getSecondEndpointX()) || Double.isNaN(getSecondEndpointY());
   }

   private void putYoValuesIntoFrameLineSegment()
   {
      frameLineSegment.set(referenceFrame, firstEndpointX.getDoubleValue(), firstEndpointY.getDoubleValue(), secondEndpointX.getDoubleValue(), secondEndpointY.getDoubleValue());
   }

   private void getYoValuesFromFrameLineSegment()
   {
      firstEndpointX.set(frameLineSegment.getFirstEndpointX());
      firstEndpointY.set(frameLineSegment.getFirstEndpointY());
      secondEndpointX.set(frameLineSegment.getSecondEndpointX());
      secondEndpointY.set(frameLineSegment.getSecondEndpointY());
   }

   public YoDouble[] getDoubleYoVariables()
   {
      return new YoDouble[] { firstEndpointX, firstEndpointY, secondEndpointX, secondEndpointY };
   }
}
