package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.math.filters.OnlineSplineFitter3D.WeightCalculator;
import us.ihmc.yoVariables.euclid.YoPoint3D;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Online filter for orientation measurements.
 * To summarize, it performs a spline fitting to a moving window of position measurements to estimate the position, linear velocity, and linear
 * acceleration.
 *
 * @see SavitzkyGolayOnlineOrientationFilter3D
 */
public class SplineBasedOnlinePositionFilter3D
{
   private final OnlineSplineFitter3D splineFitter;
   private final IntegerProvider polynomialDegree;
   private final IntegerProvider windowSizeMax;
   private final DoubleProvider windowTimeMax;

   private final YoPoint3D estimatedPosition;
   private final YoVector3D estimatedVelocity;
   private final YoVector3D estimatedAcceleration;

   public SplineBasedOnlinePositionFilter3D(String namePrefix,
                                            IntegerProvider windowSizeMax,
                                            DoubleProvider windowTimeMax,
                                            IntegerProvider polynomialDegree,
                                            YoRegistry registry)
   {
      splineFitter = new OnlineSplineFitter3D(polynomialDegree.getValue(), windowSizeMax.getValue(), windowTimeMax.getValue());
      this.polynomialDegree = polynomialDegree;
      this.windowSizeMax = windowSizeMax;
      this.windowTimeMax = windowTimeMax;

      estimatedPosition = new YoPoint3D(namePrefix + "EstimatedPosition", registry);
      estimatedVelocity = new YoVector3D(namePrefix + "EstimatedVelocity", registry);
      estimatedAcceleration = new YoVector3D(namePrefix + "EstimatedAcceleration", registry);
   }

   public void reset()
   {
      splineFitter.reset();
   }

   public void update(double time, Point3DReadOnly position)
   {
      splineFitter.setPolynomialOrder(polynomialDegree.getValue());
      splineFitter.setWindowSizeMax(windowSizeMax.getValue());
      splineFitter.setWindowTimeMax(windowTimeMax.getValue());
      splineFitter.recordNewPoint(time, position);

      estimatedPosition.set(splineFitter.evaluateValueAt(time));
      estimatedVelocity.set(splineFitter.evaluateRateAt(time));
      estimatedAcceleration.set(splineFitter.evaluateAccelerationAt(time));
   }

   public void compute(double time)
   {
      estimatedPosition.set(splineFitter.evaluateValueAt(time));
      estimatedVelocity.set(splineFitter.evaluateRateAt(time));
      estimatedAcceleration.set(splineFitter.evaluateAccelerationAt(time));
   }

   public void setRegularizationWeight(double regularizationWeight)
   {
      splineFitter.setRegularizationWeight(regularizationWeight);
   }

   public void setWeightCalculator(WeightCalculator weightCalculator)
   {
      splineFitter.setWeightCalculator(weightCalculator);
   }

   public double getNewestPointTime()
   {
      return splineFitter.getNewestPointTime();
   }

   public YoPoint3D getEstimatedPosition()
   {
      return estimatedPosition;
   }

   public YoVector3D getEstimatedVelocity()
   {
      return estimatedVelocity;
   }

   public YoVector3D getEstimatedAcceleration()
   {
      return estimatedAcceleration;
   }
}
