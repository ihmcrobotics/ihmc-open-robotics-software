package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;

import java.util.List;
import java.util.function.IntUnaryOperator;

public class SE3MPCIndexHandler extends LinearMPCIndexHandler
{
   public static final int variablesPerOrientationTick = 6;

   private static final double nominalOrientationDt = 0.01;

   private final TIntArrayList orientationStartIndices = new TIntArrayList();
   private final TDoubleArrayList orientationTickDuration = new TDoubleArrayList();
   private final TIntArrayList orientationTicksInSegment = new TIntArrayList();
   private final TIntArrayList orientationTickStartIndex = new TIntArrayList();

   protected final ListToSizeReturn listToSizeReturn = new ListToSizeReturn();

   public SE3MPCIndexHandler(int numberOfBasisVectorsPerContactPoint)
   {
      super(numberOfBasisVectorsPerContactPoint);
   }

   /**
    * Computes all the index values from the contact sequence directly.
    */
   public void initialize(List<ContactPlaneProvider> previewWindowContactSequence, double orientationWindowDuration)
   {
      super.initialize(previewWindowContactSequence);

      orientationStartIndices.clear();
      orientationTickDuration.clear();
      orientationTicksInSegment.clear();
      orientationTickStartIndex.clear();

      double remainingOrientationWindowDuration = orientationWindowDuration;
      for (int i = 0; i < previewWindowContactSequence.size(); i++)
      {
         double segmentDuration = Math.min(previewWindowContactSequence.get(i).getTimeInterval().getDuration(), remainingOrientationWindowDuration);
         int ticksInSegment = (int) Math.floor(previewWindowContactSequence.get(i).getTimeInterval().getDuration() / nominalOrientationDt);
         double tickDuration = segmentDuration / ticksInSegment;

         int orientationIndex = rhoStartIndices.get(i) + rhoCoefficientsInSegment.get(i);
         orientationStartIndices.add(orientationIndex);
         orientationTickDuration.add(tickDuration);
         orientationTicksInSegment.add(ticksInSegment);

         for (int tick = 0; tick < ticksInSegment; tick++)
         {
            orientationTickStartIndex.add(orientationIndex);
            orientationIndex += variablesPerOrientationTick;
         }

         int orientationCoefficientsInSegment = ticksInSegment * variablesPerOrientationTick;
         // shift the remaining segments
         for (int j = i + 1; j < previewWindowContactSequence.size(); j++)
         {
            comStartIndices.set(j, comStartIndices.get(j) + orientationCoefficientsInSegment);
            rhoStartIndices.set(j, rhoStartIndices.get(j) + orientationCoefficientsInSegment);
         }

         totalProblemSize += orientationCoefficientsInSegment;
         remainingOrientationWindowDuration -= segmentDuration;
      }
   }

   public int getOrientationTickStartIndex(int tick)
   {
      return orientationTickStartIndex.get(tick);
   }
}
