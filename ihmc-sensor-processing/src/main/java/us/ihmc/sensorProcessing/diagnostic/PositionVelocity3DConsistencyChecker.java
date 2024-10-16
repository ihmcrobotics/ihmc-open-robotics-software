package us.ihmc.sensorProcessing.diagnostic;

import static us.ihmc.yoVariables.euclid.filters.SimpleMovingAverageFilteredYoFrameVector3D.createSimpleMovingAverageFilteredYoFrameVector;

import java.util.EnumMap;

import us.ihmc.euclid.Axis3D;
import us.ihmc.yoVariables.euclid.filters.FilteredFiniteDifferenceYoFrameVector3D;
import us.ihmc.yoVariables.euclid.filters.SimpleMovingAverageFilteredYoFrameVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class PositionVelocity3DConsistencyChecker implements DiagnosticUpdatable
{
   private final YoRegistry registry;

   private final FilteredFiniteDifferenceYoFrameVector3D localVelocityFromFD;

   private final SimpleMovingAverageFilteredYoFrameVector3D localVelocityFiltered;
   private final SimpleMovingAverageFilteredYoFrameVector3D filteredVelocityToCheck;

   private final EnumMap<Axis3D, DelayEstimatorBetweenTwoSignals> delayEstimators = new EnumMap<>(Axis3D.class);

   private final YoDouble dummyAlpha;

   public PositionVelocity3DConsistencyChecker(String namePrefix, YoFramePoint3D position, YoFrameVector3D angularVelocityToCheck, double updateDT,
         YoRegistry parentRegistry)
   {
      registry = new YoRegistry(namePrefix + "PositionVelocity3DCheck");
      dummyAlpha = new YoDouble("dummyAlpha", registry);
      localVelocityFromFD = new FilteredFiniteDifferenceYoFrameVector3D(namePrefix, "referenceFD", dummyAlpha, updateDT, registry,
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
