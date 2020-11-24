package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.IntUnaryOperator;

public class MPCIndexHandler
{
   public static final int coefficientsPerRho = 4;
   public static final int comCoefficientsPerSegment = 6;

   private int totalProblemSize = 0;

   private final int numberOfBasisVectorsPerContactPoint;
   private final TIntArrayList rhoStartIndices = new TIntArrayList();
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
      rhoCoefficientsInSegment.clear();

      totalProblemSize = comCoefficientsPerSegment * numberOfContacts;
      for (int i = 0; i < numberOfContacts; i++)
      {
         int rhoCoefficients = coefficientsPerRho * numberOfBasisVectorsPerContactPoint * pointsInContact.applyAsInt(i);
         rhoStartIndices.add(totalProblemSize);
         totalProblemSize += rhoCoefficients;
         rhoCoefficientsInSegment.add(rhoCoefficients);
      }
   }

   public int getComCoefficientStartIndex(int segmentId, int ordinal)
   {
      return segmentId * comCoefficientsPerSegment + 2 * ordinal;
   }

   public int getRhoCoefficientStartIndex(int segmentId)
   {
      return rhoStartIndices.get(segmentId);
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
