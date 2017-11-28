package us.ihmc.sensorProcessing.diagnostic;

import static us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector.*;

import java.util.EnumMap;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.filters.FiniteDifferenceAngularVelocityYoFrameVector;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class OrientationAngularVelocityConsistencyChecker implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;

   private final FiniteDifferenceAngularVelocityYoFrameVector localVelocityFromFD;
   private final YoFrameVector angularVelocityToCheck;

   private final SimpleMovingAverageFilteredYoFrameVector localVelocityFiltered;
   private final SimpleMovingAverageFilteredYoFrameVector filteredVelocityToCheck;

   private final EnumMap<Axis, DelayEstimatorBetweenTwoSignals> delayEstimators = new EnumMap<>(Axis.class);

   private final ReferenceFrame referenceFrameUsedForComparison;
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();

   public OrientationAngularVelocityConsistencyChecker(String namePrefix, YoFrameQuaternion orientation, YoFrameVector angularVelocityToCheck, double updateDT,
         YoVariableRegistry parentRegistry)
   {
      this(namePrefix, orientation, angularVelocityToCheck, angularVelocityToCheck.getReferenceFrame(), updateDT, parentRegistry);
   }

   public OrientationAngularVelocityConsistencyChecker(String namePrefix, YoFrameQuaternion orientation, YoFrameVector angularVelocityToCheck,
         ReferenceFrame referenceFrameUsedForComparison, double updateDT, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + "OrientationVelocityCheck");
      this.referenceFrameUsedForComparison = referenceFrameUsedForComparison;

      localVelocityFromFD = new FiniteDifferenceAngularVelocityYoFrameVector(namePrefix + "referenceFD", orientation, updateDT, registry);
      this.angularVelocityToCheck = angularVelocityToCheck;

      int windowSize = 10;
      localVelocityFiltered = createSimpleMovingAverageFilteredYoFrameVector(namePrefix, "_referenceFiltered", windowSize, referenceFrameUsedForComparison, registry);
      filteredVelocityToCheck = createSimpleMovingAverageFilteredYoFrameVector(namePrefix, "_filtered", windowSize, referenceFrameUsedForComparison, registry);

      DelayEstimatorBetweenTwoSignals xVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix + "WX", localVelocityFiltered.getYoX(), filteredVelocityToCheck.getYoX(), updateDT, registry);
      DelayEstimatorBetweenTwoSignals yVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix + "WY", localVelocityFiltered.getYoY(), filteredVelocityToCheck.getYoY(), updateDT, registry);
      DelayEstimatorBetweenTwoSignals zVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix + "WZ", localVelocityFiltered.getYoZ(), filteredVelocityToCheck.getYoZ(), updateDT, registry);

      delayEstimators.put(Axis.X, xVelocityDelayEstimator);
      delayEstimators.put(Axis.Y, yVelocityDelayEstimator);
      delayEstimators.put(Axis.Z, zVelocityDelayEstimator);

      parentRegistry.addChild(registry);
   }

   @Override
   public void enable()
   {
      for (Axis axis : Axis.values)
         delayEstimators.get(axis).enable();
   }

   @Override
   public void disable()
   {
      for (Axis axis : Axis.values)
         delayEstimators.get(axis).disable();
   }

   @Override
   public void update()
   {
      localVelocityFromFD.update();
      localVelocityFromFD.getFrameTupleIncludingFrame(tempAngularVelocity);
      tempAngularVelocity.changeFrame(referenceFrameUsedForComparison);
      localVelocityFiltered.update(tempAngularVelocity);

      angularVelocityToCheck.getFrameTupleIncludingFrame(tempAngularVelocity);
      tempAngularVelocity.changeFrame(referenceFrameUsedForComparison);
      filteredVelocityToCheck.update(tempAngularVelocity);

      if (!localVelocityFiltered.getHasBufferWindowFilled())
         return;

      for (Axis axis : Axis.values)
         delayEstimators.get(axis).update();
   }

   public boolean isEstimatingDelayAll()
   {
      for (Axis axis : Axis.values)
      {
         if (!delayEstimators.get(axis).isEstimatingDelay())
            return false;
      }
      return true;
   }

   public boolean isEstimatingDelay(Axis axis)
   {
      return delayEstimators.get(axis).isEstimatingDelay();
   }

   public double getCorrelation(Axis axis)
   {
      return delayEstimators.get(axis).getCorrelationCoefficient();
   }

   public double getEstimatedDelay(Axis axis)
   {
      return delayEstimators.get(axis).getEstimatedDelay();
   }
}
