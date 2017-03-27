package us.ihmc.sensorProcessing.diagnostic;

import static us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector.*;

import java.util.EnumMap;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;

public class PositionVelocity3DConsistencyChecker implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;

   private final FilteredVelocityYoFrameVector localVelocityFromFD;

   private final SimpleMovingAverageFilteredYoFrameVector localVelocityFiltered;
   private final SimpleMovingAverageFilteredYoFrameVector filteredVelocityToCheck;

   private final EnumMap<Direction, DelayEstimatorBetweenTwoSignals> delayEstimators = new EnumMap<>(Direction.class);

   private final DoubleYoVariable dummyAlpha;

   public PositionVelocity3DConsistencyChecker(String namePrefix, YoFramePoint position, YoFrameVector angularVelocityToCheck, double updateDT,
         YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + "PositionVelocity3DCheck");
      dummyAlpha = new DoubleYoVariable("dummyAlpha", registry);
      localVelocityFromFD = FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector(namePrefix, "referenceFD", dummyAlpha, updateDT, registry,
            position);
      int windowSize = 10;
      localVelocityFiltered = createSimpleMovingAverageFilteredYoFrameVector(namePrefix, "_referenceFiltered", windowSize, localVelocityFromFD, registry);
      filteredVelocityToCheck = createSimpleMovingAverageFilteredYoFrameVector(namePrefix, "_filtered", windowSize, angularVelocityToCheck, registry);

      DelayEstimatorBetweenTwoSignals xVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix + "X", localVelocityFiltered.getYoX(),
            filteredVelocityToCheck.getYoX(), updateDT, registry);
      DelayEstimatorBetweenTwoSignals yVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix + "Y", localVelocityFiltered.getYoY(),
            filteredVelocityToCheck.getYoY(), updateDT, registry);
      DelayEstimatorBetweenTwoSignals zVelocityDelayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix + "Z", localVelocityFiltered.getYoZ(),
            filteredVelocityToCheck.getYoZ(), updateDT, registry);

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
      localVelocityFiltered.update();
      filteredVelocityToCheck.update();

      if (!localVelocityFiltered.getHasBufferWindowFilled())
         return;

      for (Direction direction : Direction.values)
         delayEstimators.get(direction).update();
   }
}
