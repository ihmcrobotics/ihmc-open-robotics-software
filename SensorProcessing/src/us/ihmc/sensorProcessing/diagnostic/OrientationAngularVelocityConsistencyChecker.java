package us.ihmc.sensorProcessing.diagnostic;

import static us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector.*;

import java.util.EnumMap;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.FiniteDifferenceAngularVelocityYoFrameVector;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class OrientationAngularVelocityConsistencyChecker implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;

   private final FiniteDifferenceAngularVelocityYoFrameVector localVelocityFromFD;
   private final YoFrameVector angularVelocityToCheck;

   private final SimpleMovingAverageFilteredYoFrameVector localVelocityFiltered;
   private final SimpleMovingAverageFilteredYoFrameVector filteredVelocityToCheck;

   private final EnumMap<Direction, DelayEstimatorBetweenTwoSignals> delayEstimators = new EnumMap<>(Direction.class);

   private final ReferenceFrame referenceFrameUsedForComparison;
   private final FrameVector tempAngularVelocity = new FrameVector();

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

      delayEstimators.put(Direction.X, xVelocityDelayEstimator);
      delayEstimators.put(Direction.Y, yVelocityDelayEstimator);
      delayEstimators.put(Direction.Z, zVelocityDelayEstimator);

      parentRegistry.addChild(registry);
   }

   @Override
   public void enable()
   {
      for (Direction direction : Direction.values)
         delayEstimators.get(direction).enable();
   }

   @Override
   public void disable()
   {
      for (Direction direction : Direction.values)
         delayEstimators.get(direction).disable();
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

      for (Direction direction : Direction.values)
         delayEstimators.get(direction).update();
   }

   public boolean isEstimatingDelayAll()
   {
      for (Direction direction : Direction.values)
      {
         if (!delayEstimators.get(direction).isEstimatingDelay())
            return false;
      }
      return true;
   }

   public boolean isEstimatingDelay(Direction direction)
   {
      return delayEstimators.get(direction).isEstimatingDelay();
   }

   public double getCorrelation(Direction direction)
   {
      return delayEstimators.get(direction).getCorrelationCoefficient();
   }

   public double getEstimatedDelay(Direction direction)
   {
      return delayEstimators.get(direction).getEstimatedDelay();
   }
}
