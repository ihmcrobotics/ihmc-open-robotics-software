package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import com.esotericsoftware.kryo.util.IntMap;
import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import us.ihmc.matrixlib.NativeMatrix;

public class RhoBoundConstraint
{
   private double accelerationViolationToAddToActiveSet = 1e-4;
   private double accelerationViolationToRemoveFromActiveSet = 1e-2;

   private final TIntArrayList rhoStartIndicesForConstraint = new TIntArrayList();

   private final TIntArrayList activeConstraints = new TIntArrayList();
   private final TIntArrayList constraintsToAdd = new TIntArrayList();
   private final TIntArrayList constraintsToRemove = new TIntArrayList();

   private final DMatrixRMaj ejmlJacobian = new DMatrixRMaj(4, 4);
   private final NativeMatrix leJacobian = new NativeMatrix(4, 4);
   private final NativeMatrix geJacobian = new NativeMatrix(4, 4);

   private final NativeMatrix constraintValues = new NativeMatrix(4, 1);
   private final NativeMatrix coefficientValue = new NativeMatrix(4, 1);

   private double upperBound = Double.POSITIVE_INFINITY;
   private double lowerBound = Double.NEGATIVE_INFINITY;

   public void reset()
   {
      rhoStartIndicesForConstraint.reset();
   }

   public void addValueStartIndices(int startIndex, int numberOfRhos)
   {
      int index = startIndex;
      for (int i = 0; i < numberOfRhos; i++, index += LinearMPCIndexHandler.coefficientsPerRho)
         rhoStartIndicesForConstraint.add(index);
   }

   public void setLowerBound(double lowerBound)
   {
      this.lowerBound = lowerBound;
   }

   public void setUpperBound(double upperBound)
   {
      this.upperBound = upperBound;
   }

   public void computeJacobian(double omega, double duration)
   {
      double omega2 = omega * omega;
      double exponential = Math.exp(omega * duration);
      double alpha = 1.0 * omega;

      ejmlJacobian.set(0, 0, omega2);
      ejmlJacobian.set(0, 1, omega2);
      ejmlJacobian.set(0, 3, 2.0);

      ejmlJacobian.set(1, 0, omega2 * (1.0 + omega / alpha));
      ejmlJacobian.set(1, 1, omega2 * (1.0 - omega / alpha));
      ejmlJacobian.set(1, 2, 6.0 / alpha);
      ejmlJacobian.set(1, 3, 2.0);

      ejmlJacobian.set(2, 0, omega2 * exponential * (1.0 - omega / alpha));
      ejmlJacobian.set(2, 1, omega2 / exponential * (1.0 + omega / alpha));
      ejmlJacobian.set(2, 2, 6.0 * (duration - 1.0 / alpha));
      ejmlJacobian.set(2, 3, 2.0);

      ejmlJacobian.set(3, 0, omega2 * exponential);
      ejmlJacobian.set(3, 1, omega2 / exponential);
      ejmlJacobian.set(3, 2, 6.0 * duration);
      ejmlJacobian.set(3, 3, 2.0);

      leJacobian.set(ejmlJacobian);
      geJacobian.set(leJacobian);
      geJacobian.scale(-1.0);
   }

   public int modifyActiveSet(NativeMatrix solution)
   {
      constraintValues.reshape(rhoStartIndicesForConstraint.size() * LinearMPCIndexHandler.coefficientsPerRho, 1);
      constraintValues.zero();

      boolean lowerBoundIsFinite = Double.isFinite(lowerBound);
      boolean upperBoundIsFinite = Double.isFinite(upperBound);

      if (!lowerBoundIsFinite && !upperBoundIsFinite)
         return 0;

      int constraintViolations = 0;
      int row = 0;
      int offset = 4 * constraintValues.getNumRows();

      for (int i = 0; i < rhoStartIndicesForConstraint.size(); i++)
      {
         int rhoStartIndex = rhoStartIndicesForConstraint.get(i);

         // FIXME this is a dumb way to do this.
         coefficientValue.zero();
         coefficientValue.addBlock(solution, 0, 0, rhoStartIndex, 0, LinearMPCIndexHandler.coefficientsPerRho, 0);
         constraintValues.multAddBlock(leJacobian, coefficientValue, rhoStartIndex, 0);

         for (int constraintRow = 0; constraintRow < 4; constraintRow++)
         {
            if (constraintValues.get(row, 0) < lowerBound - accelerationViolationToAddToActiveSet && !activeConstraints.contains(row))
               activeConstraints.add(row);
            if (constraintValues.get(row, 0) > upperBound + accelerationViolationToAddToActiveSet && !activeConstraints.contains(row + offset))
               activeConstraints.add(row + offset);
         }
      }

      return constraintViolations;
   }

   public double getLowerBound()
   {
      return lowerBound;
   }

   public double getUpperBound()
   {
      return upperBound;
   }

   public NativeMatrix getConstraintFullJacobian(int constraintNumber)
   {
      if (isLowerBoundConstraint(constraintNumber))
         return leJacobian;
      else
         return geJacobian;
   }

   public int getConstraintStartColumn(int constraintNumber)
   {
      int lowerBoundConstraints = LinearMPCIndexHandler.coefficientsPerRho * rhoStartIndicesForConstraint.size();
      if (constraintNumber >= lowerBoundConstraints)
         constraintNumber -= lowerBoundConstraints;

      return rhoStartIndicesForConstraint.get(Math.floorDiv(constraintNumber, 4));
   }

   public int getConstraintRow(int constraintNumber)
   {
      int lowerBoundConstraints = LinearMPCIndexHandler.coefficientsPerRho * rhoStartIndicesForConstraint.size();
      if (constraintNumber >= lowerBoundConstraints)
         constraintNumber -= lowerBoundConstraints;

      return constraintNumber % 4;
   }

   public int getActiveConstraints()
   {
      return activeConstraints.size();
   }

   private boolean isLowerBoundConstraint(int constraintNumber)
   {
      return constraintNumber < LinearMPCIndexHandler.coefficientsPerRho * rhoStartIndicesForConstraint.size();
   }
}
