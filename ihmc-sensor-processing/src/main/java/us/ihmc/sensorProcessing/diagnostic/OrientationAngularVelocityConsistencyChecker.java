package us.ihmc.sensorProcessing.diagnostic;

import static us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector.createSimpleMovingAverageFilteredYoFrameVector;

import java.util.EnumMap;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.filters.FiniteDifferenceAngularVelocityYoFrameVector;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameQuaternion;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class OrientationAngularVelocityConsistencyChecker implements DiagnosticUpdatable
{
   private final YoRegistry registry;

   private final FiniteDifferenceAngularVelocityYoFrameVector localVelocityFromFD;
   private final YoFrameVector3D angularVelocityToCheck;

   private final SimpleMovingAverageFilteredYoFrameVector localVelocityFiltered;
   private final SimpleMovingAverageFilteredYoFrameVector filteredVelocityToCheck;

   private final EnumMap<Axis3D, DelayEstimatorBetweenTwoSignals> delayEstimators = new EnumMap<>(Axis3D.class);

   private final ReferenceFrame referenceFrameUsedForComparison;
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();

   public OrientationAngularVelocityConsistencyChecker(String namePrefix, YoFrameQuaternion orientation, YoFrameVector3D angularVelocityToCheck, double updateDT,
         YoRegistry parentRegistry)
   {
      this(namePrefix, orientation, angularVelocityToCheck, angularVelocityToCheck.getReferenceFrame(), updateDT, parentRegistry);
   }

   public OrientationAngularVelocityConsistencyChecker(String namePrefix, YoFrameQuaternion orientation, YoFrameVector3D angularVelocityToCheck,
         ReferenceFrame referenceFrameUsedForComparison, double updateDT, YoRegistry parentRegistry)
   {
      registry = new YoRegistry(namePrefix + "OrientationVelocityCheck");
      this.referenceFrameUsedForComparison = referenceFrameUsedForComparison;

      localVelocityFromFD = new FiniteDifferenceAngularVelocityYoFrameVector(namePrefix + "referenceFD", orientation, updateDT, registry);
      this.angularVelocityToCheck = angularVelocityToCheck;

      int windowSize = 10;
      localVelocityFiltered = createSimpleMovingAverageFilteredYoFrameVector(namePrefix, "_referenceFiltered", windowSize, referenceFrameUsedForComparison, registry);
      filteredVelocityToCheck = createSimpleMovingAverageFilteredYoFrameVector(namePrefix, "_filtered", windowSize, referenceFrameUsedForComparison, registry);

      DelayEstimatorBetweenTwoSignals xVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix + "WX", localVelocityFiltered.getYoX(), filteredVelocityToCheck.getYoX(), updateDT, registry);
      DelayEstimatorBetweenTwoSignals yVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix + "WY", localVelocityFiltered.getYoY(), filteredVelocityToCheck.getYoY(), updateDT, registry);
      DelayEstimatorBetweenTwoSignals zVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix + "WZ", localVelocityFiltered.getYoZ(), filteredVelocityToCheck.getYoZ(), updateDT, registry);

      delayEstimators.put(Axis3D.X, xVelocityDelayEstimator);
      delayEstimators.put(Axis3D.Y, yVelocityDelayEstimator);
      delayEstimators.put(Axis3D.Z, zVelocityDelayEstimator);

      parentRegistry.addChild(registry);
   }

   @Override
   public void enable()
   {
      for (Axis3D axis : Axis3D.values)
         delayEstimators.get(axis).enable();
   }

   @Override
   public void disable()
   {
      for (Axis3D axis : Axis3D.values)
         delayEstimators.get(axis).disable();
   }

   @Override
   public void update()
   {
      localVelocityFromFD.update();
      tempAngularVelocity.setIncludingFrame(localVelocityFromFD);
      tempAngularVelocity.changeFrame(referenceFrameUsedForComparison);
      localVelocityFiltered.update(tempAngularVelocity);

      tempAngularVelocity.setIncludingFrame(angularVelocityToCheck);
      tempAngularVelocity.changeFrame(referenceFrameUsedForComparison);
      filteredVelocityToCheck.update(tempAngularVelocity);

      if (!localVelocityFiltered.getHasBufferWindowFilled())
         return;

      for (Axis3D axis : Axis3D.values)
         delayEstimators.get(axis).update();
   }

   public boolean isEstimatingDelayAll()
   {
      for (Axis3D axis : Axis3D.values)
      {
         if (!delayEstimators.get(axis).isEstimatingDelay())
            return false;
      }
      return true;
   }

   public boolean isEstimatingDelay(Axis3D axis)
   {
      return delayEstimators.get(axis).isEstimatingDelay();
   }

   public double getCorrelation(Axis3D axis)
   {
      return delayEstimators.get(axis).getCorrelationCoefficient();
   }

   public double getEstimatedDelay(Axis3D axis)
   {
      return delayEstimators.get(axis).getEstimatedDelay();
   }
}
