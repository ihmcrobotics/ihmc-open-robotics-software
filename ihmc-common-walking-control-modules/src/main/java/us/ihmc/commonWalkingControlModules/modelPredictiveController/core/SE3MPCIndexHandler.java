package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.providers.IntegerProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;
import java.util.function.IntUnaryOperator;

public class SE3MPCIndexHandler extends LinearMPCIndexHandler
{
   private static final double intermediateDt = 0.025;
   public static final int variablesPerOrientationTick = 6;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble orientationDt = new YoDouble("orientationDt", registry);

   private int totalNumberOfOrientationTicks = 0;
   private final TIntArrayList orientationStartIndices = new TIntArrayList();

   private final TIntArrayList ticksInSegment = new TIntArrayList();
   private final TDoubleArrayList tickDurations = new TDoubleArrayList();

   public SE3MPCIndexHandler(int numberOfBasisVectorsPerContactPoint)
   {
      super(numberOfBasisVectorsPerContactPoint);

      orientationDt.set(intermediateDt);
   }

   /**
    * Computes all the index values from the contact sequence directly.
    */
   public void initialize(List<ContactPlaneProvider> previewWindowContactSequence)
   {
      super.initialize(previewWindowContactSequence);

      orientationStartIndices.reset();
      tickDurations.reset();
      ticksInSegment.reset();

      totalNumberOfOrientationTicks = 0;
      for (int segmentNumber = 0; segmentNumber < previewWindowContactSequence.size(); segmentNumber++)
      {
         double segmentDuration = previewWindowContactSequence.get(segmentNumber).getTimeInterval().getDuration();
         int ticksInSegment = computeTicksInSegment(segmentDuration);
         double tickDuration = segmentDuration / ticksInSegment;

         this.ticksInSegment.add(ticksInSegment);
         this.tickDurations.add(tickDuration);

         totalNumberOfOrientationTicks += variablesPerOrientationTick;

         int orientationIndex = comStartIndices.get(segmentNumber);
         orientationStartIndices.add(orientationIndex);
         variablesInSegment.set(segmentNumber, variablesPerOrientationTick + variablesInSegment.get(segmentNumber));

         // shift the remaining segments
         for (int j = segmentNumber; j < previewWindowContactSequence.size(); j++)
         {
            comStartIndices.set(j, comStartIndices.get(j) + variablesPerOrientationTick);
            rhoStartIndices.set(j, rhoStartIndices.get(j) + variablesPerOrientationTick);
         }

         totalProblemSize += variablesPerOrientationTick;
      }
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public int getTotalNumberOfOrientationTicks()
   {
      return totalNumberOfOrientationTicks;
   }

   public int getOrientationStartIndex(int segment)
   {
      return orientationStartIndices.get(segment);
   }

   public int getTicksInSegment(int segment)
   {
      return ticksInSegment.get(segment);
   }

   public double getTickDuration(int segment)
   {
      return tickDurations.get(segment);
   }

   private int computeTicksInSegment(double segmentDuration)
   {
      if (segmentDuration > 0.0 && segmentDuration < orientationDt.getValue())
         return 1;
      else
         return (int) Math.floor(segmentDuration / orientationDt.getValue());
   }
}
