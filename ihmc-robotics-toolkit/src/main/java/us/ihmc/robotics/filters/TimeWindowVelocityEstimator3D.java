package us.ihmc.robotics.filters;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.dataStructures.GrowableRingBuffer;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class TimeWindowVelocityEstimator3D extends YoFrameVector3D
{
   private final GrowableRingBuffer<Point3D> positionMeasures;
   private final DoubleProvider dt;

   private final YoFramePoint3D windowStart;
   private final YoDouble windowDuration;
   private final YoInteger measures;

   public TimeWindowVelocityEstimator3D(String namePrefix, ReferenceFrame referenceFrame, double windowDuration, double controlDt, YoRegistry registry)
   {
      this(namePrefix, "", referenceFrame, windowDuration, () -> controlDt, registry);
   }

   public TimeWindowVelocityEstimator3D(String namePrefix, ReferenceFrame referenceFrame, double windowDuration, DoubleProvider controlDt, YoRegistry registry)
   {
      this(namePrefix, "", referenceFrame, windowDuration, controlDt, registry);
   }

   public TimeWindowVelocityEstimator3D(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, double windowDuration, DoubleProvider controlDt, YoRegistry registry)
   {
      super(namePrefix, nameSuffix, referenceFrame, registry);

      this.windowStart  = new YoFramePoint3D(namePrefix + "_WindowStart", nameSuffix, referenceFrame, registry);
      this.windowDuration = new YoDouble(namePrefix + "_WindowDuration", registry);
      this.measures = new YoInteger(namePrefix + "_Measures", registry);

      this.windowDuration.set(windowDuration);
      this.dt = controlDt;

      int maxSize = computeWindowSize();
      positionMeasures = new GrowableRingBuffer<>(2 * maxSize, Point3D::new);
      positionMeasures.resize(maxSize);

      double maxDuration = 2 * windowDuration;
      this.windowDuration.addListener(v ->
                                      {
                                         if (this.windowDuration.getValue() > maxDuration)
                                            this.windowDuration.set(maxDuration, false);

                                         reset();
                                      });
   }

   private int computeWindowSize()
   {
      return (int) Math.ceil(windowDuration.getDoubleValue() / dt.getValue());
   }

   public void reset()
   {
      measures.set(0);
      windowStart.setToNaN();
      setToNaN();
   }

   public void update(FrameTuple3DReadOnly positionMeasure)
   {
      update(positionMeasure.getReferenceFrame(), positionMeasure);
   }

   private final Point3D current = new Point3D();
   private final Vector3D sum = new Vector3D();
   private final Vector3D tempVelocity = new Vector3D();

   public void update(ReferenceFrame referenceFrame, Tuple3DReadOnly positionMeasure)
   {
      checkReferenceFrameMatch(referenceFrame);

      positionMeasures.resize(computeWindowSize());

      // Add the new position measure to the history
      positionMeasures.add().set(positionMeasure);
      // Update the size of the time history window
      measures.set(positionMeasures.getCurrentSize());

      current.set(positionMeasure);

      if (measures.getValue() > 1)
      {
         double totalWeight = 0;
         windowStart.set(positionMeasures.get(0));
         double windowDuration = (measures.getValue() - 1) * dt.getValue();

         sum.setToZero();
         for (int i = 0; i < measures.getValue() - 1; i++)
         {
            tempVelocity.sub(current, positionMeasures.get(i));
            double weight = measures.getValue() - i;
            tempVelocity.scale( weight / windowDuration);
            sum.add(tempVelocity);

            windowDuration -= dt.getValue();
            totalWeight += weight;
         }
         setAndScale(1.0 / totalWeight, sum);
      }
      else
         setToNaN();
   }
}
