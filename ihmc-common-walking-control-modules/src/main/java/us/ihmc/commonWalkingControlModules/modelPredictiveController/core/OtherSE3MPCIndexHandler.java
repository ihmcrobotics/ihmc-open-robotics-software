package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;

import java.util.List;

public class OtherSE3MPCIndexHandler extends LinearMPCIndexHandler
{
   public static final int variablesPerOrientationTick = 6;

   private int totalNumberOfOrientationTicks = 0;
   private final TIntArrayList orientationStartIndices = new TIntArrayList();

   public OtherSE3MPCIndexHandler(int numberOfBasisVectorsPerContactPoint)
   {
      super(numberOfBasisVectorsPerContactPoint);
   }

   /**
    * Computes all the index values from the contact sequence directly.
    */
   public void initialize(List<ContactPlaneProvider> previewWindowContactSequence)
   {
      super.initialize(previewWindowContactSequence);

      orientationStartIndices.clear();
      orientationStartIndices.add(-1);

      totalNumberOfOrientationTicks = 0;
      for (int i = 1; i < previewWindowContactSequence.size(); i++)
      {
         totalNumberOfOrientationTicks += variablesPerOrientationTick;

         int orientationIndex = rhoStartIndices.get(i) + rhoCoefficientsInSegment.get(i);
         orientationStartIndices.add(orientationIndex);

         // shift the remaining segments
         for (int j = i + 1; j < previewWindowContactSequence.size(); j++)
         {
            comStartIndices.set(j, comStartIndices.get(j) + variablesPerOrientationTick);
            rhoStartIndices.set(j, rhoStartIndices.get(j) + variablesPerOrientationTick);
         }

         totalProblemSize += variablesPerOrientationTick;
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
}
