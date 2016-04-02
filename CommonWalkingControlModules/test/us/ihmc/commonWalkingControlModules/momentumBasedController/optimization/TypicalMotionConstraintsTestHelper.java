package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import static org.junit.Assert.assertEquals;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;
import org.ejml.ops.SingularOps;

public class TypicalMotionConstraintsTestHelper
{
   private final DenseMatrix64F JPrimaryMotionConstraints, pPrimaryMotionConstraints, centroidalMomentumAMatrix, momentumDotEquationRightHandSide,
         momentumSubspace;

   private final DenseMatrix64F combinedConstraintsAndDesiredMomentum, combinedRightHandSide;

   private final DenseMatrix64F combinedUMatrix, combinedWMatrix, combinedVTransposeMatrix;

   private final double[] singularValuesOfCombinedConstraints;
   private final double minimumSingularValueOfCombinedConstraints;

   private final DenseMatrix64F combinedSolution;

   public DenseMatrix64F getCombinedSolution()
   {
      return combinedSolution;
   }

   public TypicalMotionConstraintsTestHelper(double[][] JPrimaryMotionConstraintsData, double[][] pPrimaryMotionConstraintsData,
         double[][] centroidalMomentumAMatrixData, double[][] momentumDotEquationRightHandSideData, double[][] momentumSubspaceData)
   {
      this.JPrimaryMotionConstraints = new DenseMatrix64F(JPrimaryMotionConstraintsData);
      this.pPrimaryMotionConstraints = new DenseMatrix64F(pPrimaryMotionConstraintsData);
      this.centroidalMomentumAMatrix = new DenseMatrix64F(centroidalMomentumAMatrixData);
      this.momentumDotEquationRightHandSide = new DenseMatrix64F(momentumDotEquationRightHandSideData);
      this.momentumSubspace = new DenseMatrix64F(momentumSubspaceData);

      DenseMatrix64F momentumSubspaceProjector = new DenseMatrix64F(momentumSubspace.getNumRows(), momentumSubspace.getNumRows());
      CommonOps.multOuter(momentumSubspace, momentumSubspaceProjector);

      DenseMatrix64F momentumSubspaceSelector = new DenseMatrix64F(momentumSubspace.getNumCols(), momentumSubspace.getNumRows());
      CommonOps.transpose(momentumSubspace, momentumSubspaceSelector);

      assertEquals(JPrimaryMotionConstraints.getNumRows(), pPrimaryMotionConstraints.getNumRows());

      //      DenseMatrix64F[] svd = computeSVD(JPrimaryMotionConstraints);
      //      DenseMatrix64F matrixU = svd[0];
      //      DenseMatrix64F matrixW = svd[1];
      //      DenseMatrix64F matrixVTranspose = svd[2];
      //
      //      System.out.println("matrixW for primary motion constraints = " + matrixW);
      //
      //      svd = computeSVD(centroidalMomentumAMatrix);
      //      matrixU = svd[0];
      //      matrixW = svd[1];
      //      matrixVTranspose = svd[2];
      //
      //      System.out.println("matrixW for centroidalMomentumAMatrix = " + matrixW);

      DenseMatrix64F centroidalMomentumAMatrixSelected = new DenseMatrix64F(momentumSubspaceSelector.getNumRows(), centroidalMomentumAMatrix.getNumCols());
      CommonOps.mult(momentumSubspaceSelector, centroidalMomentumAMatrix, centroidalMomentumAMatrixSelected);

      DenseMatrix64F momentumDotEquationRightHandSideSelected = new DenseMatrix64F(momentumSubspaceSelector.getNumRows(),
            momentumDotEquationRightHandSide.getNumCols());
      CommonOps.mult(momentumSubspaceSelector, momentumDotEquationRightHandSide, momentumDotEquationRightHandSideSelected);

      combinedConstraintsAndDesiredMomentum = new DenseMatrix64F(JPrimaryMotionConstraints.getNumRows() + centroidalMomentumAMatrixSelected.getNumRows(),
            JPrimaryMotionConstraints.getNumCols());

      CommonOps.extract(JPrimaryMotionConstraints, 0, JPrimaryMotionConstraints.getNumRows(), 0, JPrimaryMotionConstraints.getNumCols(),
            combinedConstraintsAndDesiredMomentum, 0, 0);
      CommonOps.extract(centroidalMomentumAMatrixSelected, 0, centroidalMomentumAMatrixSelected.getNumRows(), 0, centroidalMomentumAMatrixSelected.getNumCols(),
            combinedConstraintsAndDesiredMomentum, JPrimaryMotionConstraints.getNumRows(), 0);

      combinedRightHandSide = new DenseMatrix64F(pPrimaryMotionConstraints.getNumRows() + momentumDotEquationRightHandSideSelected.getNumRows(), 1);
      CommonOps.extract(pPrimaryMotionConstraints, 0, pPrimaryMotionConstraints.getNumRows(), 0, pPrimaryMotionConstraints.getNumCols(), combinedRightHandSide,
            0, 0);
      CommonOps.extract(momentumDotEquationRightHandSideSelected, 0, momentumDotEquationRightHandSideSelected.getNumRows(), 0,
            momentumDotEquationRightHandSideSelected.getNumCols(), combinedRightHandSide, pPrimaryMotionConstraints.getNumRows(), 0);

      ImmutablePair<DenseMatrix64F[], double[]> svd = computeSVD(combinedConstraintsAndDesiredMomentum);
      DenseMatrix64F[] matrices = svd.getLeft();
      singularValuesOfCombinedConstraints = svd.getRight();

      combinedUMatrix = matrices[0];
      combinedWMatrix = matrices[1];
      combinedVTransposeMatrix = matrices[2];

      //      System.out.println("matrixW for combinedConstraintsAndDesiredMomentum = " + combinedWMatrix);

      minimumSingularValueOfCombinedConstraints = findMinimum(singularValuesOfCombinedConstraints);

      if (combinedConstraintsAndDesiredMomentum.getNumRows() == combinedConstraintsAndDesiredMomentum.getNumCols())
      {
         combinedSolution = new DenseMatrix64F(combinedConstraintsAndDesiredMomentum.getNumRows(), 1);
         CommonOps.solve(combinedConstraintsAndDesiredMomentum, combinedRightHandSide, combinedSolution);
      }
      else
      {
         combinedSolution = null;
      }
   }

   public double[] getSingularValuesOfCombinedConstraints()
   {
      return singularValuesOfCombinedConstraints;
   }

   public double getMinimumSingularValueOfCombinedConstraints()
   {
      return minimumSingularValueOfCombinedConstraints;
   }

   private double findMinimum(double[] values)
   {
      double minimum = Double.POSITIVE_INFINITY;

      for (double value : values)
      {
         if (value < minimum)
            minimum = value;
      }

      return minimum;
   }

   private ImmutablePair<DenseMatrix64F[], double[]> computeSVD(DenseMatrix64F matrix)
   {
      SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(matrix.getNumRows(), matrix.getNumCols(), true, true, false);
      svd.decompose(matrix);

      DenseMatrix64F matrixU = new DenseMatrix64F(matrix.getNumRows(), matrix.getNumRows());
      DenseMatrix64F matrixW = new DenseMatrix64F(matrix.getNumRows(), matrix.getNumCols());
      DenseMatrix64F matrixVTranspose = new DenseMatrix64F(matrix.getNumCols(), matrix.getNumCols());
      svd.getU(matrixU, false);
      svd.getW(matrixW);
      svd.getV(matrixVTranspose, true);

      double[] singularValues = svd.getSingularValues();

      SingularOps.descendingOrder(matrixU, false, matrixW, matrixVTranspose, true);

      return new ImmutablePair<DenseMatrix64F[], double[]>(new DenseMatrix64F[] {matrixU, matrixW, matrixVTranspose}, singularValues);
   }
}
