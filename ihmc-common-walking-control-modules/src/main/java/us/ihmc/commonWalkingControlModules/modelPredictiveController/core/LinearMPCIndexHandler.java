package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;

import java.util.List;
import java.util.function.IntUnaryOperator;

/**
 * This class is the index handler for the linear model predictive controller. It assumes that each segment has two coefficients that form the initial value
 * problem constraints (position and velocity) for each axis, and then four coefficients per generalized contact force (referred to as Rho). The combination of
 * these coefficients fully define the CoM motion function for each segment, and are grouped together.
 *
 * The purpose of this class is to provide assignments from the variable coefficients that go into the MPC to the corresponding motion functions.
 */
public class LinearMPCIndexHandler
{
   public static final int coefficientsPerRho = 4;
   public static final int comCoefficientsPerSegment = 6;

   protected int totalProblemSize = 0;

   protected final int numberOfBasisVectorsPerContactPoint;
   protected final TIntArrayList comStartIndices = new TIntArrayList();
   protected final TIntArrayList rhoStartIndices = new TIntArrayList();
   protected final TIntArrayList rhoCoefficientsInSegment = new TIntArrayList();

   protected final ListToSizeReturn listToSizeReturn = new ListToSizeReturn();

   public LinearMPCIndexHandler(int numberOfBasisVectorsPerContactPoint)
   {
      this.numberOfBasisVectorsPerContactPoint = numberOfBasisVectorsPerContactPoint;
   }

   /**
    * Computes all the index values from the contact sequence directly.
    */
   public void initialize(List<ContactPlaneProvider> contactSequence)
   {
      listToSizeReturn.setContacts(contactSequence);
      initialize(listToSizeReturn, contactSequence.size());
   }

   public void initialize(IntUnaryOperator pointsInContact, int numberOfContacts)
   {
      rhoStartIndices.clear();
      comStartIndices.clear();
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
      }
   }

   /**
    * Gets the start index for the linear coefficients for the segment specified by {@param segmentId}.
    *
    * @param segmentId segment number query
    */
   public int getComCoefficientStartIndex(int segmentId)
   {
      return comStartIndices.get(segmentId);
   }

   /**
    * Gets the start index for the linear coefficients for axis specified by {@param axisOrdinal} in the segment specified by {@param segmentId}.
    *
    * @param segmentId segment number query
    * @param axisOrdinal axis ordinal query
    */
   public int getComCoefficientStartIndex(int segmentId, int axisOrdinal)
   {
      return getComCoefficientStartIndex(segmentId) + 2 * axisOrdinal;
   }

   /**
    * Gets the total number of segments in the MPC problem
    *
    * @return total number of segments in the MPC problem
    */
   public int getNumberOfSegments()
   {
      return comStartIndices.size();
   }

   /**
    * Gets the total number coefficients used to represent the generalized contact forces for the indicated segment
    * @param segmentId segment to query.
    * @return total number of generalized contact force coefficients
    */
   public int getRhoCoefficientsInSegment(int segmentId)
   {
      return rhoCoefficientsInSegment.get(segmentId);
   }

   /**
    * Gets the index of the start variable for the generalized contact force coefficients for the indicated segment.
    * @param segmentId segment to query.
    * @return index of the start variable for the queried segment
    */
   public int getRhoCoefficientStartIndex(int segmentId)
   {
      return rhoStartIndices.get(segmentId);
   }

   /**
    * Gets the total number of variables in the optimization problem. This is the sum of all the variables in all the segments.
    * @return total number of variables
    */
   public int getTotalProblemSize()
   {
      return totalProblemSize;
   }

   protected static class ListToSizeReturn implements IntUnaryOperator
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
