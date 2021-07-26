package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class ActiveSetData
{
   private boolean hasData = false;

   private int segmentNumber = -1;
   private int numberOfVariablesInSegment = -1;
   private int numberOfInequalityConstraints = -1;

   private final TIntArrayList activeInequalityIndices = new TIntArrayList();

   private final DMatrixRMaj previousSolution = new DMatrixRMaj(1, 1);

   public void reset()
   {
      hasData = false;

      segmentNumber = -1;
      numberOfVariablesInSegment = -1;

      numberOfInequalityConstraints = 0;

      clearActiveSet();
   }

   public void resetConstraintCounter()
   {
      numberOfInequalityConstraints = 0;
   }

   public void clearActiveSet()
   {
      CommonOps_DDRM.fill(previousSolution, Double.NaN);

      activeInequalityIndices.reset();
   }

   public void setSegmentNumber(int segmentNumber)
   {
      this.segmentNumber = segmentNumber;
   }

   public void setNumberOfVariablesInSegment(int numberOfVariablesInSegment)
   {
      this.numberOfVariablesInSegment = numberOfVariablesInSegment;
      previousSolution.reshape(numberOfVariablesInSegment, 1);
      CommonOps_DDRM.fill(previousSolution, Double.NaN);
   }

   public DMatrixRMaj getPreviousSolution()
   {
      return previousSolution;
   }

   public void addInequalityConstraints(int numberOfConstraints)
   {
      numberOfInequalityConstraints += numberOfConstraints;
   }

   public void addActiveInequalityConstraint(int constraintIndex)
   {
      activeInequalityIndices.add(constraintIndex);
   }

   public int getNumberOfInequalityConstraints()
   {
      return numberOfInequalityConstraints;
   }

   public int getNumberOfActiveInequalityConstraints()
   {
      return activeInequalityIndices.size();
   }

   public int getActiveInequalityIndex(int constraint)
   {
      return activeInequalityIndices.get(constraint);
   }
}
