package us.ihmc.sensorProcessing.diagnostic;

import static us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector.createSimpleMovingAverageFilteredYoFrameVector;

import java.util.EnumMap;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.FiniteDifferenceAngularVelocityYoFrameVector;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class OrientationAngularVelocityConsistencyChecker implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;

   private final FiniteDifferenceAngularVelocityYoFrameVector localVelocityFromFD;

   private final SimpleMovingAverageFilteredYoFrameVector localVelocityFiltered;
   private final SimpleMovingAverageFilteredYoFrameVector filteredVelocityToCheck;

   private final EnumMap<Axis, DelayEstimatorBetweenTwoSignals> delayEstimators = new EnumMap<>(Axis.class);

   private final FrameVector tempAngularVelocity = new FrameVector();

   public OrientationAngularVelocityConsistencyChecker(String namePrefix, YoFrameQuaternion orientation, YoFrameVector angularVelocityToCheck, double updateDT,
         YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + "OrientationVelocityCheck");
      localVelocityFromFD = new FiniteDifferenceAngularVelocityYoFrameVector(namePrefix + "referenceFD", orientation, updateDT, registry);
      int windowSize = 10;
      localVelocityFiltered = SimpleMovingAverageFilteredYoFrameVector.createSimpleMovingAverageFilteredYoFrameVector(namePrefix, "_referenceFiltered", windowSize, angularVelocityToCheck.getReferenceFrame(), registry);
      filteredVelocityToCheck = createSimpleMovingAverageFilteredYoFrameVector(namePrefix, "_filtered", windowSize, angularVelocityToCheck, registry);

      DelayEstimatorBetweenTwoSignals xVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix + "WX", localVelocityFiltered.getYoX(), filteredVelocityToCheck.getYoX(), updateDT, registry);
      DelayEstimatorBetweenTwoSignals yVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix + "WY", localVelocityFiltered.getYoY(), filteredVelocityToCheck.getYoY(), updateDT, registry);
      DelayEstimatorBetweenTwoSignals zVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix + "WZ", localVelocityFiltered.getYoZ(), filteredVelocityToCheck.getYoZ(), updateDT, registry);

      delayEstimators.put(Axis.X, xVelocityDelayEstimator);
      delayEstimators.put(Axis.Y, yVelocityDelayEstimator);
      delayEstimators.put(Axis.Z, zVelocityDelayEstimator);

      parentRegistry.addChild(registry);
   }

   public void enableAll()
   {
      for (Axis axis : Axis.values)
         delayEstimators.get(axis).enable();
   }

   public void disableAll()
   {
      for (Axis axis : Axis.values)
         delayEstimators.get(axis).disable();
   }

   @Override
   public void update()
   {
      localVelocityFromFD.update();
      localVelocityFromFD.getFrameTupleIncludingFrame(tempAngularVelocity);
      tempAngularVelocity.changeFrame(localVelocityFiltered.getReferenceFrame());
      localVelocityFiltered.update(tempAngularVelocity);
      filteredVelocityToCheck.update();

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
