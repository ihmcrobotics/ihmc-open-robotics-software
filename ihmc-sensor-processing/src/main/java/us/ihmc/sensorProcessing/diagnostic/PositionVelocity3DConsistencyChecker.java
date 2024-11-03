package us.ihmc.sensorProcessing.diagnostic;

import static us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector.createSimpleMovingAverageFilteredYoFrameVector;

import java.util.EnumMap;

import us.ihmc.euclid.Axis3D;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoFrameVector;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class PositionVelocity3DConsistencyChecker implements DiagnosticUpdatable
{
   private final YoRegistry registry;

   private final FilteredVelocityYoFrameVector localVelocityFromFD;

   private final SimpleMovingAverageFilteredYoFrameVector localVelocityFiltered;
   private final SimpleMovingAverageFilteredYoFrameVector filteredVelocityToCheck;

   private final EnumMap<Axis3D, DelayEstimatorBetweenTwoSignals> delayEstimators = new EnumMap<>(Axis3D.class);

   private final YoDouble dummyAlpha;

   public PositionVelocity3DConsistencyChecker(String namePrefix, YoFramePoint3D position, YoFrameVector3D angularVelocityToCheck, double updateDT,
         YoRegistry parentRegistry)
   {
      registry = new YoRegistry(namePrefix + "PositionVelocity3DCheck");
      dummyAlpha = new YoDouble("dummyAlpha", registry);
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
      localVelocityFiltered.update();
      filteredVelocityToCheck.update();

      if (!localVelocityFiltered.getHasBufferWindowFilled())
         return;

      for (Axis3D axis : Axis3D.values)
         delayEstimators.get(axis).update();
   }
}
