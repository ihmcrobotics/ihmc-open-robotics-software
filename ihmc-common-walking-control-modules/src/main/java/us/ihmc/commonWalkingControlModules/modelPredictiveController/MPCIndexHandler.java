package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import gnu.trove.list.array.TIntArrayList;

import java.util.List;
import java.util.function.IntUnaryOperator;

public class MPCIndexHandler
{
   public static final int coefficientsPerRho = 4;
   public static final int comCoefficientsPerSegment = 6;
   public static final int orientationCoefficientsPerSegment = 6;

   private int totalProblemSize = 0;

   private final int numberOfBasisVectorsPerContactPoint;
   private final TIntArrayList comStartIndices = new TIntArrayList();
   private final TIntArrayList rhoStartIndices = new TIntArrayList();
   private final TIntArrayList orientationStartIndices = new TIntArrayList();
   private final TIntArrayList rhoCoefficientsInSegment = new TIntArrayList();

   private final ListToSizeReturn listToSizeReturn = new ListToSizeReturn();

   public MPCIndexHandler(int numberOfBasisVectorsPerContactPoint)
   {
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;
   }

   public void initialize(List<ContactPlaneProvider> contactSequence)
   {
      listToSizeReturn.setContacts(contactSequence);
      initialize(listToSizeReturn, contactSequence.size());
   }

   public void initialize(IntUnaryOperator pointsInContact, int numberOfContacts)
   {
      rhoStartIndices.clear();
      comStartIndices.clear();
      orientationStartIndices.clear();
      rhoCoefficientsInSegment.clear();

      totalProblemSize = 0;
      for (int i = 0; i < numberOfContacts; i++)
      {
         comStartIndices.add(totalProblemSize);
         totalProblemSize += comCoefficientsPerSegment;

         int rhoCoefficients = coefficientsPerRho * numberOfBasisVectorsPerContactPoint * pointsInContact.applyAsInt(i);
         rhoCoefficientsInSegment.add(rhoCoefficients);
         rhoStartIndices.add(totalProblemSize);
         totalProblemSize += rhoCoefficients;

         orientationStartIndices.add(totalProblemSize);
         totalProblemSize += 3 * orientationCoefficientsPerSegment;
      }
   }

   public int getComCoefficientStartIndex(int segmentId)
   {
      return comStartIndices.get(segmentId);
   }

   public int getComCoefficientStartIndex(int segmentId, int ordinal)
   {
      return getComCoefficientStartIndex(segmentId) + 2 * ordinal;
   }

   public int getNumberOfSegments()
   {
      return comStartIndices.size();
   }

   public int getRhoCoefficientsInSegment(int segmentId)
   {
      return rhoCoefficientsInSegment.get(segmentId);
   }

   public int getRhoCoefficientStartIndex(int segmentId)
   {
      return rhoStartIndices.get(segmentId);
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

   public int getTotalProblemSize()
   {
      return totalProblemSize;
   }

   private static class ListToSizeReturn implements IntUnaryOperator
   {
      private List<ContactPlaneProvider> contacts;

      public void setContacts(List<ContactPlaneProvider> contacts)
      {
         this.contacts = contacts;
      }

      @Override
      public int applyAsInt(int operand)
      {
         return contacts.get(operand).getTotalNumberOfPointsInContact();
      }
   }
}
