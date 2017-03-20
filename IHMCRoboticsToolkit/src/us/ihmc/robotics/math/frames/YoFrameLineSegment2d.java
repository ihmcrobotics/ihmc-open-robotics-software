package us.ihmc.robotics.math.frames;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AbstractReferenceFrameHolder;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Note: You should only make these once at the initialization of a controller. You shouldn't make any on the fly
 * since they contain YoVariables.
 */
public class YoFrameLineSegment2d extends AbstractReferenceFrameHolder
{
   /** This is where the data is stored. All operations must act on these numbers. */
   private final DoubleYoVariable firstEndpointX, firstEndpointY, secondEndpointX, secondEndpointY;
   private final ReferenceFrame referenceFrame;
   /** This is only for assistance. The data is stored in the YoVariables, not in here! */
   protected FrameLineSegment2d frameLineSegment;

   public YoFrameLineSegment2d(String namePrefix, String nameSuffix, ReferenceFrame frame, YoVariableRegistry registry)
   {
      this(namePrefix, nameSuffix, "", frame, registry);
   }

   public YoFrameLineSegment2d(String namePrefix, String nameSuffix, String description, ReferenceFrame frame, YoVariableRegistry registry)
   {
      firstEndpointX = new DoubleYoVariable(namePrefix + "FirstEndpointX" + nameSuffix, description, registry);
      firstEndpointY = new DoubleYoVariable(namePrefix + "FirstEndpointY" + nameSuffix, description, registry);
      secondEndpointX = new DoubleYoVariable(namePrefix + "SecondEndpointX" + nameSuffix, description, registry);
      secondEndpointY = new DoubleYoVariable(namePrefix + "SecondEndpointY" + nameSuffix, description, registry);

      this.referenceFrame = frame;
      frameLineSegment = new FrameLineSegment2d(referenceFrame);
   }

   public YoFrameLineSegment2d(DoubleYoVariable firstEndpointX, DoubleYoVariable firstEndpointY, DoubleYoVariable secondEndpointX, DoubleYoVariable secondEndpointY,
         ReferenceFrame frame)
   {
      this.firstEndpointX = firstEndpointX;
      this.firstEndpointY = firstEndpointY;
      this.secondEndpointX = secondEndpointX;
      this.secondEndpointY = secondEndpointY;

      this.referenceFrame = frame;
      frameLineSegment = new FrameLineSegment2d(referenceFrame);
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

   public DoubleYoVariable getYoFirstEndpointX()
   {
      return firstEndpointX;
   }

   public DoubleYoVariable getYoFirstEndpointY()
   {
      return firstEndpointY;
   }

   public DoubleYoVariable getYoSecondEndpointX()
   {
      return secondEndpointX;
   }

   public DoubleYoVariable getYoSecondEndpointY()
   {
      return secondEndpointY;
   }

   public void setFrameLineSegment2d(FrameLineSegment2d lineSegment)
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

   public void set(FramePoint2d firstEndpoint, FramePoint2d secondEndpoint)
   {
      referenceFrame.checkReferenceFrameMatch(firstEndpoint.getReferenceFrame());
      referenceFrame.checkReferenceFrameMatch(secondEndpoint.getReferenceFrame());
      frameLineSegment.set(firstEndpoint, secondEndpoint);
      getYoValuesFromFrameLineSegment();
   }

   public void set(FramePoint2d firstEndpoint, FrameVector2d vectorToSecondEndpoint)
   {
      referenceFrame.checkReferenceFrameMatch(firstEndpoint.getReferenceFrame());
      referenceFrame.checkReferenceFrameMatch(vectorToSecondEndpoint.getReferenceFrame());
      frameLineSegment.set(firstEndpoint, vectorToSecondEndpoint);
      getYoValuesFromFrameLineSegment();
   }

   public FrameLineSegment2d getFrameLineSegment2d()
   {
      putYoValuesIntoFrameLineSegment();
      return new FrameLineSegment2d(frameLineSegment);
   }
   
   public void getFrameLineSegment2d(FrameLineSegment2d lineSegmentToPack)
   {
      putYoValuesIntoFrameLineSegment();
      lineSegmentToPack.setAndChangeFrame(frameLineSegment);
   }

   public FramePoint2d midpoint()
   {
      putYoValuesIntoFrameLineSegment();

      return frameLineSegment.midpoint();
   }

   public void getFirstEndPoint(FramePoint2d firstEndpointToPack)
   {
      putYoValuesIntoFrameLineSegment();

      frameLineSegment.getFirstEndpoint(firstEndpointToPack);
   }

   public void getSecondEndPoint(FramePoint2d secondEndpointToPack)
   {
      putYoValuesIntoFrameLineSegment();

      frameLineSegment.getSecondEndpoint(secondEndpointToPack);
   }

   public void getFirstEndPoint(Point2D firstEndpointToPack)
   {
      putYoValuesIntoFrameLineSegment();
      
      frameLineSegment.getFirstEndpoint(firstEndpointToPack);
   }

   public void getSecondEndPoint(Point2D secondEndpointToPack)
   {
      putYoValuesIntoFrameLineSegment();

      frameLineSegment.getSecondEndpoint(secondEndpointToPack);
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
   
   public boolean areEndpointsTheSame()
   {
      return LineSegment2d.areEndpointsTheSame(getFirstEndpointX(), getFirstEndpointY(), getSecondEndpointX(), getSecondEndpointY());
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
      LineSegment2d lineSegment2d = frameLineSegment.getLineSegment2d();
      firstEndpointX.set(lineSegment2d.getFirstEndpointX());
      firstEndpointY.set(lineSegment2d.getFirstEndpointY());
      secondEndpointX.set(lineSegment2d.getSecondEndpointX());
      secondEndpointY.set(lineSegment2d.getSecondEndpointY());
   }

   public DoubleYoVariable[] getDoubleYoVariables()
   {
      return new DoubleYoVariable[] { firstEndpointX, firstEndpointY, secondEndpointX, secondEndpointY };
   }
}
