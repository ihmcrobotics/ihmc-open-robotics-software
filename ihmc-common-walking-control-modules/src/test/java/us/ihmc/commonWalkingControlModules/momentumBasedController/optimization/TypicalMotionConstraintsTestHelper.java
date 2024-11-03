package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import static us.ihmc.robotics.Assert.assertEquals;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.SingularOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;

public class TypicalMotionConstraintsTestHelper
{
   private final DMatrixRMaj JPrimaryMotionConstraints, pPrimaryMotionConstraints, centroidalMomentumAMatrix, momentumDotEquationRightHandSide,
         momentumSubspace;

   private final DMatrixRMaj combinedConstraintsAndDesiredMomentum, combinedRightHandSide;

   private final DMatrixRMaj combinedUMatrix, combinedWMatrix, combinedVTransposeMatrix;

   private final double[] singularValuesOfCombinedConstraints;
   private final double minimumSingularValueOfCombinedConstraints;

   private final DMatrixRMaj combinedSolution;

   public DMatrixRMaj getCombinedSolution()
   {
      return combinedSolution;
   }

   public TypicalMotionConstraintsTestHelper(double[][] JPrimaryMotionConstraintsData, double[][] pPrimaryMotionConstraintsData,
         double[][] centroidalMomentumAMatrixData, double[][] momentumDotEquationRightHandSideData, double[][] momentumSubspaceData)
   {
      this.JPrimaryMotionConstraints = new DMatrixRMaj(JPrimaryMotionConstraintsData);
      this.pPrimaryMotionConstraints = new DMatrixRMaj(pPrimaryMotionConstraintsData);
      this.centroidalMomentumAMatrix = new DMatrixRMaj(centroidalMomentumAMatrixData);
      this.momentumDotEquationRightHandSide = new DMatrixRMaj(momentumDotEquationRightHandSideData);
      this.momentumSubspace = new DMatrixRMaj(momentumSubspaceData);

      DMatrixRMaj momentumSubspaceProjector = new DMatrixRMaj(momentumSubspace.getNumRows(), momentumSubspace.getNumRows());
      CommonOps_DDRM.multOuter(momentumSubspace, momentumSubspaceProjector);

      DMatrixRMaj momentumSubspaceSelector = new DMatrixRMaj(momentumSubspace.getNumCols(), momentumSubspace.getNumRows());
      CommonOps_DDRM.transpose(momentumSubspace, momentumSubspaceSelector);

      assertEquals(JPrimaryMotionConstraints.getNumRows(), pPrimaryMotionConstraints.getNumRows());

      //      DMatrixRMaj[] svd = computeSVD(JPrimaryMotionConstraints);
      //      DMatrixRMaj matrixU = svd[0];
      //      DMatrixRMaj matrixW = svd[1];
      //      DMatrixRMaj matrixVTranspose = svd[2];
      //
      //      System.out.println("matrixW for primary motion constraints = " + matrixW);
      //
      //      svd = computeSVD(centroidalMomentumAMatrix);
      //      matrixU = svd[0];
      //      matrixW = svd[1];
      //      matrixVTranspose = svd[2];
      //
      //      System.out.println("matrixW for centroidalMomentumAMatrix = " + matrixW);

      DMatrixRMaj centroidalMomentumAMatrixSelected = new DMatrixRMaj(momentumSubspaceSelector.getNumRows(), centroidalMomentumAMatrix.getNumCols());
      CommonOps_DDRM.mult(momentumSubspaceSelector, centroidalMomentumAMatrix, centroidalMomentumAMatrixSelected);

      DMatrixRMaj momentumDotEquationRightHandSideSelected = new DMatrixRMaj(momentumSubspaceSelector.getNumRows(),
            momentumDotEquationRightHandSide.getNumCols());
      CommonOps_DDRM.mult(momentumSubspaceSelector, momentumDotEquationRightHandSide, momentumDotEquationRightHandSideSelected);

      combinedConstraintsAndDesiredMomentum = new DMatrixRMaj(JPrimaryMotionConstraints.getNumRows() + centroidalMomentumAMatrixSelected.getNumRows(),
            JPrimaryMotionConstraints.getNumCols());

      CommonOps_DDRM.extract(JPrimaryMotionConstraints, 0, JPrimaryMotionConstraints.getNumRows(), 0, JPrimaryMotionConstraints.getNumCols(),
            combinedConstraintsAndDesiredMomentum, 0, 0);
      CommonOps_DDRM.extract(centroidalMomentumAMatrixSelected, 0, centroidalMomentumAMatrixSelected.getNumRows(), 0, centroidalMomentumAMatrixSelected.getNumCols(),
            combinedConstraintsAndDesiredMomentum, JPrimaryMotionConstraints.getNumRows(), 0);

      combinedRightHandSide = new DMatrixRMaj(pPrimaryMotionConstraints.getNumRows() + momentumDotEquationRightHandSideSelected.getNumRows(), 1);
      CommonOps_DDRM.extract(pPrimaryMotionConstraints, 0, pPrimaryMotionConstraints.getNumRows(), 0, pPrimaryMotionConstraints.getNumCols(), combinedRightHandSide,
            0, 0);
      CommonOps_DDRM.extract(momentumDotEquationRightHandSideSelected, 0, momentumDotEquationRightHandSideSelected.getNumRows(), 0,
            momentumDotEquationRightHandSideSelected.getNumCols(), combinedRightHandSide, pPrimaryMotionConstraints.getNumRows(), 0);

      ImmutablePair<DMatrixRMaj[], double[]> svd = computeSVD(combinedConstraintsAndDesiredMomentum);
      DMatrixRMaj[] matrices = svd.getLeft();
      singularValuesOfCombinedConstraints = svd.getRight();

      combinedUMatrix = matrices[0];
      combinedWMatrix = matrices[1];
      combinedVTransposeMatrix = matrices[2];

      //      System.out.println("matrixW for combinedConstraintsAndDesiredMomentum = " + combinedWMatrix);

      minimumSingularValueOfCombinedConstraints = findMinimum(singularValuesOfCombinedConstraints);

      if (combinedConstraintsAndDesiredMomentum.getNumRows() == combinedConstraintsAndDesiredMomentum.getNumCols())
      {
         combinedSolution = new DMatrixRMaj(combinedConstraintsAndDesiredMomentum.getNumRows(), 1);
         CommonOps_DDRM.solve(combinedConstraintsAndDesiredMomentum, combinedRightHandSide, combinedSolution);
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

   private ImmutablePair<DMatrixRMaj[], double[]> computeSVD(DMatrixRMaj matrix)
   {
      SingularValueDecomposition_F64<DMatrixRMaj> svd = DecompositionFactory_DDRM.svd(matrix.getNumRows(), matrix.getNumCols(), true, true, false);
      svd.decompose(matrix);

      DMatrixRMaj matrixU = new DMatrixRMaj(matrix.getNumRows(), matrix.getNumRows());
      DMatrixRMaj matrixW = new DMatrixRMaj(matrix.getNumRows(), matrix.getNumCols());
      DMatrixRMaj matrixVTranspose = new DMatrixRMaj(matrix.getNumCols(), matrix.getNumCols());
      svd.getU(matrixU, false);
      svd.getW(matrixW);
      svd.getV(matrixVTranspose, true);

      double[] singularValues = svd.getSingularValues();

      SingularOps_DDRM.descendingOrder(matrixU, false, matrixW, matrixVTranspose, true);

      return new ImmutablePair<DMatrixRMaj[], double[]>(new DMatrixRMaj[] {matrixU, matrixW, matrixVTranspose}, singularValues);
   }
}
