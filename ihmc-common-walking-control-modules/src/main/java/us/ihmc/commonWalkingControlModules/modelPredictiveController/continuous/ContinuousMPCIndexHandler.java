package us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.LinearMPCIndexHandler;

import java.util.function.IntUnaryOperator;

public class ContinuousMPCIndexHandler extends LinearMPCIndexHandler
{
   public static final boolean includeExponentialInOrientation = false;
   public static final int orientationCoefficientsPerSegment = includeExponentialInOrientation ? 6 : 4;

   private final TIntArrayList orientationStartIndices = new TIntArrayList();

   public ContinuousMPCIndexHandler(int numberOfBasisVectorsPerContactPoint)
   {
      super(numberOfBasisVectorsPerContactPoint);
   }

   @Override
   public void initialize(IntUnaryOperator pointsInContact, int numberOfContacts)
   {
      super.initialize(pointsInContact, numberOfContacts);

      orientationStartIndices.clear();
      for (int i = 0; i < numberOfContacts; i++)
      {
         orientationStartIndices.add(totalProblemSize);
         totalProblemSize += 3 * orientationCoefficientsPerSegment;
      }
   }

   public int getOrientationCoefficientsStartIndex(int segmentId)
   {
      return orientationStartIndices.get(segmentId);
   }

   public int getYawCoefficientsStartIndex(int segmentId)
   {
      return getOrientationCoefficientsStartIndex(segmentId);
   }

   public int getPitchCoefficientsStartIndex(int segmentId)
   {
      return orientationCoefficientsPerSegment + getYawCoefficientsStartIndex(segmentId);
   }

   public int getRollCoefficientsStartIndex(int segmentId)
   {
      return orientationCoefficientsPerSegment + getPitchCoefficientsStartIndex(segmentId);
   }
}
