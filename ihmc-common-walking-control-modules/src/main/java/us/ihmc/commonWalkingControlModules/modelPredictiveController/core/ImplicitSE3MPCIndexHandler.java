package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;

import java.util.List;

public class ImplicitSE3MPCIndexHandler extends LinearMPCIndexHandler
{
   private static final double intermediateDt = 0.01;
   public static final int variablesPerOrientationTick = 6;

   private int totalNumberOfOrientationTicks = 0;
   private final TIntArrayList orientationStartIndices = new TIntArrayList();

   private final TIntArrayList ticksInSegment = new TIntArrayList();
   private final TDoubleArrayList tickDurations = new TDoubleArrayList();

   public ImplicitSE3MPCIndexHandler(int numberOfBasisVectorsPerContactPoint)
   {
      super(numberOfBasisVectorsPerContactPoint);
   }

   /**
    * Computes all the index values from the contact sequence directly.
    */
   public void initialize(List<ContactPlaneProvider> previewWindowContactSequence)
   {
      super.initialize(previewWindowContactSequence);

      orientationStartIndices.reset();
      orientationStartIndices.add(-1);

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

         if (segmentNumber > 0)
         {
            totalNumberOfOrientationTicks += variablesPerOrientationTick;

            int orientationIndex = rhoStartIndices.get(segmentNumber) + rhoCoefficientsInSegment.get(segmentNumber);
            orientationStartIndices.add(orientationIndex);

            // shift the remaining segments
            for (int j = segmentNumber + 1; j < previewWindowContactSequence.size(); j++)
            {
               comStartIndices.set(j, comStartIndices.get(j) + variablesPerOrientationTick);
               rhoStartIndices.set(j, rhoStartIndices.get(j) + variablesPerOrientationTick);
            }

            totalProblemSize += variablesPerOrientationTick;
         }
      }
   }

   public int getTotalNumberOfOrientationTicks()
   {
      return totalNumberOfOrientationTicks;
   }

   public int getOrientationStartIndices(int segment)
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

   private static int computeTicksInSegment(double segmentDuration)
   {
      if (segmentDuration > 0.0 && segmentDuration < intermediateDt)
         return 1;
      else
         return (int) Math.floor(segmentDuration / intermediateDt);
   }
}
