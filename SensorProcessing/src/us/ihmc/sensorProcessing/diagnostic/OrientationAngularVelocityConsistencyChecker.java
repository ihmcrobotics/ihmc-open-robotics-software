package us.ihmc.sensorProcessing.diagnostic;

import static us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector.createSimpleMovingAverageFilteredYoFrameVector;

import java.util.EnumMap;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.filters.FiniteDifferenceAngularVelocityYoFrameVector;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class OrientationAngularVelocityConsistencyChecker
{
   private final YoVariableRegistry registry;

   private final FiniteDifferenceAngularVelocityYoFrameVector localVelocityFromFD;

   private final SimpleMovingAverageFilteredYoFrameVector localVelocityFiltered;
   private final SimpleMovingAverageFilteredYoFrameVector filteredVelocityToCheck;

   private final EnumMap<Axis, DelayEstimatorBetweenTwoSignals> delayEstimators = new EnumMap<>(Axis.class);

   public OrientationAngularVelocityConsistencyChecker(String namePrefix, YoFrameQuaternion orientation, YoFrameVector angularVelocityToCheck, double updateDT,
         YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + "OrientationVelocityCheck");
      localVelocityFromFD = new FiniteDifferenceAngularVelocityYoFrameVector(namePrefix + "referenceFD", orientation, updateDT, registry);
      int windowSize = 10;
      localVelocityFiltered = createSimpleMovingAverageFilteredYoFrameVector(namePrefix, "_referenceFiltered", windowSize, localVelocityFromFD, registry);
      filteredVelocityToCheck = createSimpleMovingAverageFilteredYoFrameVector(namePrefix, "_filtered", windowSize, angularVelocityToCheck, registry);

      DelayEstimatorBetweenTwoSignals xVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix, localVelocityFiltered.getYoX(),
            filteredVelocityToCheck.getYoX(), updateDT, registry);
      DelayEstimatorBetweenTwoSignals yVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix, localVelocityFiltered.getYoY(),
            filteredVelocityToCheck.getYoY(), updateDT, registry);
      DelayEstimatorBetweenTwoSignals zVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix, localVelocityFiltered.getYoZ(),
            filteredVelocityToCheck.getYoZ(), updateDT, registry);

      delayEstimators.put(Axis.X, xVelocityDelayEstimator);
      delayEstimators.put(Axis.Y, yVelocityDelayEstimator);
      delayEstimators.put(Axis.Z, zVelocityDelayEstimator);

      parentRegistry.addChild(registry);
   }

   public void update()
   {
      localVelocityFromFD.update();
      localVelocityFiltered.update();
      filteredVelocityToCheck.update();

      if (!localVelocityFiltered.getHasBufferWindowFilled())
         return;

      for (Axis axis : Axis.values)
         delayEstimators.get(axis).update();
   }
}
