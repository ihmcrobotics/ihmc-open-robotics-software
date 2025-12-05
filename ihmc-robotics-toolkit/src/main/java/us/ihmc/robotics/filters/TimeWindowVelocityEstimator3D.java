package us.ihmc.robotics.filters;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class TimeWindowVelocityEstimator3D extends YoFrameVector3D
{
   private final Point3D[] positionMeasures;
   private final double dt;
   private int writeIndex;

   private final YoFramePoint3D windowStart;
   private final YoDouble windowDuration;
   private final YoInteger measures;

   public TimeWindowVelocityEstimator3D(String namePrefix, ReferenceFrame referenceFrame, double windowDuration, double controlDt, YoRegistry registry)
   {
      this(namePrefix, "", referenceFrame, windowDuration, controlDt, registry);
   }

   public TimeWindowVelocityEstimator3D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, double windowDuration, double controlDt, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.windowStart  = new YoFramePoint3D(namePrefix + "_WindowStart", nameSuffix, referenceFrame, registry);
      this.windowDuration = new YoDouble(namePrefix + "_WindowDuration", registry);
      this.measures = new YoInteger(namePrefix + "_Measures", registry);

      this.windowDuration.set(windowDuration);
      this.dt = controlDt;

      int maxSize = computeMaxSize();
      positionMeasures = new Point3D[2 * maxSize];
      for (int i = 0; i < 2 * maxSize; i++)
         positionMeasures[i] = new Point3D();


      double maxDuration = 2 * windowDuration;
      this.windowDuration.addListener(v ->
                                      {
                                         if (this.windowDuration.getValue() > maxDuration)
                                            this.windowDuration.set(maxDuration, false);

                                         reset();
                                      });
   }

   private int computeMaxSize()
   {
      return (int) Math.ceil(windowDuration.getDoubleValue() / dt);
   }

   public void reset()
   {
      writeIndex = -1;
      measures.set(0);
      windowStart.setToNaN();
      setToNaN();
   }

   public void update(FrameTuple3DReadOnly positionMeasure)
   {
      update(positionMeasure.getReferenceFrame(), positionMeasure.getX(), positionMeasure.getY(), positionMeasure.getZ());
   }

   private final Point3D current = new Point3D();
   private final Vector3D sum = new Vector3D();
   private final Vector3D tempVelocity = new Vector3D();

   public void update(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      checkReferenceFrameMatch(referenceFrame);

      int maxSize = computeMaxSize();

      // Get the next index
      writeIndex = nextIndex(maxSize, writeIndex);
      // Add the new position measure to the history
      positionMeasures[writeIndex].set(x, y, z);
      // Update the size of the time history window
      measures.set(Math.min(maxSize, measures.getValue() + 1));

      current.set(x, y, z);

      if (measures.getValue() > 1)
      {
         double totalWeight = 0;
         int index = getStartOfWindowIndex(maxSize);
         windowStart.set(positionMeasures[index]);
         double windowDuration = (measures.getValue() - 1) * dt;

         sum.setToZero();
         for (int i = 0; i < measures.getValue() - 1; i++)
         {
            tempVelocity.sub(current, positionMeasures[index]);
            double weight = measures.getValue() - i;
            tempVelocity.scale( weight / windowDuration);
            sum.add(tempVelocity);

            index = nextIndex(maxSize, index);
            windowDuration -= dt;
            totalWeight += weight;
         }
         setAndScale(1.0 / totalWeight, sum);
      }
      else
         setToNaN();
   }

   private int nextIndex(int maxSize, int index)
   {
      if (index == maxSize - 1)
         return 0;
      return index + 1;
   }

   private int getStartOfWindowIndex(int maxSize)
   {
      if (measures.getValue() == maxSize)
      {
         // the buffer is full, so get the next write index, as it's the start
         return nextIndex(maxSize, writeIndex);
      }
      else
      {
         // the buffer isn't full, so get the first field
         return 0;
      }
   }
}
