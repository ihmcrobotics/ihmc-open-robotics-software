package us.ihmc.convexOptimization.quadraticProgram;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.linearAlgebra.DiagonalMatrixTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

/**
 * Utilizes the same procedure as the Simple Efficient Active Set QP Solver, but assumes that the
 * quadratic cost matrix is diagonal.
 *
 */
public class SimpleDiagonalActiveSetQPSolver extends SimpleEfficientActiveSetQPSolver
{
   private static final double epsilon = 1e-10;

   private int maxNumberOfIterations = 10;

   private final DenseMatrix64F quadraticCostQMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F quadraticCostQVector = new DenseMatrix64F(0, 0);
   private double quadraticCostScalar;

   private final DenseMatrix64F linearEqualityConstraintsAMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F linearEqualityConstraintsBVector = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F linearInequalityConstraintsCMatrixO = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F linearInequalityConstraintsDVectorO = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F variableLowerBounds = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F variableUpperBounds = new DenseMatrix64F(0, 0);

   private final TIntArrayList activeInequalityIndices = new TIntArrayList();
   private final TIntArrayList activeUpperBoundIndices = new TIntArrayList();
   private final TIntArrayList activeLowerBoundIndices = new TIntArrayList();

   // Some temporary matrices:
   private final DenseMatrix64F negativeQuadraticCostQVector = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F linearInequalityConstraintsCheck = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F CBar = new DenseMatrix64F(0, 0); // Active inequality constraints
   private final DenseMatrix64F DBar = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CHat = new DenseMatrix64F(0, 0); // Active variable bounds constraints
   private final DenseMatrix64F DHat = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F ATranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CBarTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CHatTranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F QInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F AQInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F QInverseATranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CBarQInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CHatQInverse = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F AQInverseATranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F AQInverseCBarTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F AQInverseCHatTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CBarQInverseATranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CHatQInverseATranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F QInverseCBarTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F QInverseCHatTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CBarQInverseCBarTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CHatQInverseCHatTranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F CBarQInverseCHatTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F CHatQInverseCBarTranspose = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F ATransposeAndCTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F ATransposeMuAndCTransposeLambda = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F bigMatrixForLagrangeMultiplierSolution = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F bigVectorForLagrangeMultiplierSolution = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F tempVector = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F augmentedLagrangeMultipliers = new DenseMatrix64F(0, 0);

   private final TIntArrayList inequalityIndicesToAddToActiveSet = new TIntArrayList();
   private final TIntArrayList inequalityIndicesToRemoveFromActiveSet = new TIntArrayList();

   private final TIntArrayList upperBoundIndicesToAddToActiveSet = new TIntArrayList();
   private final TIntArrayList upperBoundIndicesToRemoveFromActiveSet = new TIntArrayList();

   private final TIntArrayList lowerBoundIndicesToAddToActiveSet = new TIntArrayList();
   private final TIntArrayList lowerBoundIndicesToRemoveFromActiveSet = new TIntArrayList();

   private final DenseMatrix64F computedObjectiveFunctionValue = new DenseMatrix64F(1, 1);

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   private boolean quadraticCostMatrixIsDiagonal = false;
   private boolean useWarmStart = false;

   private int previousNumberOfVariables = 0;
   private int previousNumberOfEqualityConstraints = 0;
   private int previousNumberOfInequalityConstraints = 0;
   private int previousNumberOfLowerBoundConstraints = 0;
   private int previousNumberOfUpperBoundConstraints = 0;

   @Override
   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      this.maxNumberOfIterations = maxNumberOfIterations;
   }

   @Override
   public void clear()
   {
      quadraticCostQMatrix.reshape(0, 0);
      quadraticCostQVector.reshape(0, 0);

      linearEqualityConstraintsAMatrix.reshape(0, 0);
      linearEqualityConstraintsBVector.reshape(0, 0);

      linearInequalityConstraintsCMatrixO.reshape(0, 0);
      linearInequalityConstraintsDVectorO.reshape(0, 0);

      variableLowerBounds.reshape(0, 0);
      variableUpperBounds.reshape(0, 0);
   }

   @Override
   public void setVariableBounds(DenseMatrix64F variableLowerBounds, DenseMatrix64F variableUpperBounds)
   {
      if (variableLowerBounds.getNumRows() != quadraticCostQMatrix.getNumRows())
         throw new RuntimeException("variableLowerBounds.getNumRows() != quadraticCostQMatrix.getNumRows()");
      if (variableUpperBounds.getNumRows() != quadraticCostQMatrix.getNumRows())
         throw new RuntimeException("variableUpperBounds.getNumRows() != quadraticCostQMatrix.getNumRows()");

      this.variableLowerBounds.set(variableLowerBounds);
      this.variableUpperBounds.set(variableUpperBounds);
   }

   @Override
   public void setVariableBounds(double[] variableLowerBounds, double[] variableUpperBounds)
   {
      setVariableBounds(MatrixTools.createVector(variableLowerBounds), MatrixTools.createVector(variableUpperBounds));
   }

   @Override
   public void setQuadraticCostFunction(double[][] quadraticCostFunctionQMatrix, double[] quadraticCostFunctionQVector, double quadraticCostScalar)
   {
      setQuadraticCostFunction(new DenseMatrix64F(quadraticCostFunctionQMatrix), MatrixTools.createVector(quadraticCostFunctionQVector), quadraticCostScalar);
   }

   @Override
   public void setQuadraticCostFunction(DenseMatrix64F costQuadraticMatrix, DenseMatrix64F costLinearVector, double quadraticCostScalar)
   {
      if (costLinearVector.getNumCols() != 1)
         throw new RuntimeException("costLinearVector.getNumCols() != 1");
      if (costQuadraticMatrix.getNumRows() != costLinearVector.getNumRows())
         throw new RuntimeException("costQuadraticMatrix.getNumRows() != costLinearVector.getNumRows()");
      if (costQuadraticMatrix.getNumRows() != costQuadraticMatrix.getNumCols())
         throw new RuntimeException("costQuadraticMatrix.getNumRows() != costQuadraticMatrix.getNumCols()");

      this.quadraticCostQMatrix.set(costQuadraticMatrix);
      this.quadraticCostQVector.set(costLinearVector);
      this.quadraticCostScalar = quadraticCostScalar;
   }

   @Override
   public double getObjectiveCost(DenseMatrix64F x)
   {
      multQuad(x, quadraticCostQMatrix, computedObjectiveFunctionValue);
      CommonOps.scale(0.5, computedObjectiveFunctionValue);
      CommonOps.multAddTransA(quadraticCostQVector, x, computedObjectiveFunctionValue);
      return computedObjectiveFunctionValue.get(0, 0) + quadraticCostScalar;
   }

   private final DenseMatrix64F temporaryMatrix = new DenseMatrix64F(0, 0);

   private void multQuad(DenseMatrix64F xVector, DenseMatrix64F QMatrix, DenseMatrix64F xTransposeQx)
   {
      temporaryMatrix.reshape(xVector.numCols, QMatrix.numCols);
      CommonOps.multTransA(xVector, QMatrix, temporaryMatrix);
      CommonOps.mult(temporaryMatrix, xVector, xTransposeQx);
   }

   @Override
   public void setLinearEqualityConstraints(double[][] linearEqualityConstraintsAMatrix, double[] linearEqualityConstraintsBVector)
   {
      setLinearEqualityConstraints(new DenseMatrix64F(linearEqualityConstraintsAMatrix), MatrixTools.createVector(linearEqualityConstraintsBVector));
   }

   @Override
   public void setLinearEqualityConstraints(DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector)
   {
      if (linearEqualityConstraintsBVector.getNumCols() != 1)
         throw new RuntimeException("linearEqualityConstraintsBVector.getNumCols() != 1");
      if (linearEqualityConstraintsAMatrix.getNumRows() != linearEqualityConstraintsBVector.getNumRows())
         throw new RuntimeException("linearEqualityConstraintsAMatrix.getNumRows() != linearEqualityConstraintsBVector.getNumRows()");
      if (linearEqualityConstraintsAMatrix.getNumCols() != quadraticCostQMatrix.getNumCols())
         throw new RuntimeException("linearEqualityConstraintsAMatrix.getNumCols() != quadraticCostQMatrix.getNumCols()");

      this.linearEqualityConstraintsBVector.set(linearEqualityConstraintsBVector);
      this.linearEqualityConstraintsAMatrix.set(linearEqualityConstraintsAMatrix);
   }

   @Override
   public void setLinearInequalityConstraints(double[][] linearInequalityConstraintsCMatrix, double[] linearInqualityConstraintsDVector)
   {
      setLinearInequalityConstraints(new DenseMatrix64F(linearInequalityConstraintsCMatrix), MatrixTools.createVector(linearInqualityConstraintsDVector));
   }

   @Override
   public void setLinearInequalityConstraints(DenseMatrix64F linearInequalityConstraintCMatrix, DenseMatrix64F linearInequalityConstraintDVector)
   {
      if (linearInequalityConstraintDVector.getNumCols() != 1)
         throw new RuntimeException("linearInequalityConstraintDVector.getNumCols() != 1");
      if (linearInequalityConstraintCMatrix.getNumRows() != linearInequalityConstraintDVector.getNumRows())
         throw new RuntimeException("linearInequalityConstraintCMatrix.getNumRows() != linearInequalityConstraintDVector.getNumRows()");
      if (linearInequalityConstraintCMatrix.getNumCols() != quadraticCostQMatrix.getNumCols())
         throw new RuntimeException("linearInequalityConstraintCMatrix.getNumCols() != quadraticCostQMatrix.getNumCols()");

      this.linearInequalityConstraintsDVectorO.set(linearInequalityConstraintDVector);
      this.linearInequalityConstraintsCMatrixO.set(linearInequalityConstraintCMatrix);
   }

   @Override
   public int solve(double[] solutionToPack)
   {
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrixO.getNumRows();

      double[] lagrangeEqualityConstraintMultipliersToPack = new double[numberOfEqualityConstraints];
      double[] lagrangeInequalityConstraintMultipliersToPack = new double[numberOfInequalityConstraints];

      return solve(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack);
   }

   @Override
   public int solve(double[] solutionToPack, double[] lagrangeEqualityConstraintMultipliersToPack, double[] lagrangeInequalityConstraintMultipliersToPack)
   {
      int numberOfLowerBoundConstraints = variableLowerBounds.getNumRows();
      int numberOfUpperBoundConstraints = variableUpperBounds.getNumRows();

      double[] lagrangeLowerBoundsConstraintMultipliersToPack = new double[numberOfLowerBoundConstraints];
      double[] lagrangeUpperBoundsConstraintMultipliersToPack = new double[numberOfUpperBoundConstraints];

      return solve(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack,
                   lagrangeLowerBoundsConstraintMultipliersToPack, lagrangeUpperBoundsConstraintMultipliersToPack);
   }

   @Override
   public int solve(double[] solutionToPack, double[] lagrangeEqualityConstraintMultipliersToPack, double[] lagrangeInequalityConstraintMultipliersToPack,
                    double[] lagrangeLowerBoundsConstraintMultipliersToPack, double[] lagrangeUpperBoundsConstraintMultipliersToPack)
   {
      int numberOfVariables = quadraticCostQMatrix.getNumCols();
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrixO.getNumRows();
      int numberOfLowerBoundConstraints = variableLowerBounds.getNumRows();
      int numberOfUpperBoundConstraints = variableUpperBounds.getNumRows();

      if (solutionToPack.length != numberOfVariables)
         throw new RuntimeException("solutionToPack.length != numberOfVariables");
      if (lagrangeEqualityConstraintMultipliersToPack.length != numberOfEqualityConstraints)
         throw new RuntimeException("lagrangeEqualityConstraintMultipliersToPack.length != numberOfEqualityConstraints");
      if (lagrangeInequalityConstraintMultipliersToPack.length != numberOfInequalityConstraints)
         throw new RuntimeException("lagrangeInequalityConstraintMultipliersToPack.length != numberOfInequalityConstraints");

      if (lagrangeLowerBoundsConstraintMultipliersToPack.length != numberOfLowerBoundConstraints)
         throw new RuntimeException("lagrangeLowerBoundsConstraintMultipliersToPack.length != numberOfLowerBoundConstraints. numberOfLowerBoundConstraints = "
               + numberOfLowerBoundConstraints);
      if (lagrangeUpperBoundsConstraintMultipliersToPack.length != numberOfUpperBoundConstraints)
         throw new RuntimeException("lagrangeUpperBoundsConstraintMultipliersToPack.length != numberOfUpperBoundConstraints");

      DenseMatrix64F solution = new DenseMatrix64F(numberOfVariables, 1);
      DenseMatrix64F lagrangeEqualityConstraintMultipliers = new DenseMatrix64F(numberOfEqualityConstraints, 1);
      DenseMatrix64F lagrangeInequalityConstraintMultipliers = new DenseMatrix64F(numberOfInequalityConstraints, 1);
      DenseMatrix64F lagrangeLowerBoundConstraintMultipliers = new DenseMatrix64F(numberOfLowerBoundConstraints, 1);
      DenseMatrix64F lagrangeUpperBoundConstraintMultipliers = new DenseMatrix64F(numberOfUpperBoundConstraints, 1);

      int numberOfIterations = solve(solution, lagrangeEqualityConstraintMultipliers, lagrangeInequalityConstraintMultipliers,
                                     lagrangeLowerBoundConstraintMultipliers, lagrangeUpperBoundConstraintMultipliers);

      double[] solutionData = solution.getData();

      for (int i = 0; i < numberOfVariables; i++)
      {
         solutionToPack[i] = solutionData[i];
      }

      double[] lagrangeEqualityConstraintMultipliersData = lagrangeEqualityConstraintMultipliers.getData();
      double[] lagrangeInequalityConstraintMultipliersData = lagrangeInequalityConstraintMultipliers.getData();
      double[] lagrangeLowerBoundMultipliersData = lagrangeLowerBoundConstraintMultipliers.getData();
      double[] lagrangeUpperBoundMultipliersData = lagrangeUpperBoundConstraintMultipliers.getData();

      for (int i = 0; i < numberOfEqualityConstraints; i++)
      {
         lagrangeEqualityConstraintMultipliersToPack[i] = lagrangeEqualityConstraintMultipliersData[i];
      }

      for (int i = 0; i < numberOfInequalityConstraints; i++)
      {
         lagrangeInequalityConstraintMultipliersToPack[i] = lagrangeInequalityConstraintMultipliersData[i];
      }

      for (int i = 0; i < numberOfLowerBoundConstraints; i++)
      {
         lagrangeLowerBoundsConstraintMultipliersToPack[i] = lagrangeLowerBoundMultipliersData[i];
      }

      for (int i = 0; i < numberOfUpperBoundConstraints; i++)
      {
         lagrangeUpperBoundsConstraintMultipliersToPack[i] = lagrangeUpperBoundMultipliersData[i];
      }

      return numberOfIterations;
   }

   @Override
   public void setUseWarmStart(boolean useWarmStart)
   {
      this.useWarmStart = useWarmStart;
   }

   @Override
   public void resetActiveConstraints()
   {
      CBar.reshape(0, 0);
      CHat.reshape(0, 0);
      DBar.reshape(0, 0);
      DHat.reshape(0, 0);

      activeInequalityIndices.reset();
      activeUpperBoundIndices.reset();
      activeLowerBoundIndices.reset();
   }

   private final DenseMatrix64F lagrangeEqualityConstraintMultipliersToThrowAway = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F lagrangeInequalityConstraintMultipliersToThrowAway = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F lagrangeLowerBoundMultipliersToThrowAway = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F lagrangeUpperBoundMultipliersToThrowAway = new DenseMatrix64F(0, 0);

   @Override
   public int solve(DenseMatrix64F solutionToPack)
   {
      return solve(solutionToPack, lagrangeEqualityConstraintMultipliersToThrowAway, lagrangeInequalityConstraintMultipliersToThrowAway);
   }

   @Override
   public int solve(DenseMatrix64F solutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack,
                    DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack)
   {
      return solve(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack,
                   lagrangeLowerBoundMultipliersToThrowAway, lagrangeUpperBoundMultipliersToThrowAway);
   }

   @Override
   public int solve(DenseMatrix64F solutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack,
                    DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack, DenseMatrix64F lagrangeLowerBoundConstraintMultipliersToPack,
                    DenseMatrix64F lagrangeUpperBoundConstraintMultipliersToPack)
   {
      if (!useWarmStart || problemSizeChanged())
         resetActiveConstraints();

      int numberOfIterations = 0;

      int numberOfVariables = quadraticCostQMatrix.getNumRows();
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrixO.getNumRows();
      int numberOfLowerBoundConstraints = variableLowerBounds.getNumRows();
      int numberOfUpperBoundConstraints = variableUpperBounds.getNumRows();

      solutionToPack.reshape(numberOfVariables, 1);
      lagrangeEqualityConstraintMultipliersToPack.reshape(numberOfEqualityConstraints, 1);
      lagrangeEqualityConstraintMultipliersToPack.zero();
      lagrangeInequalityConstraintMultipliersToPack.reshape(numberOfInequalityConstraints, 1);
      lagrangeInequalityConstraintMultipliersToPack.zero();
      lagrangeLowerBoundConstraintMultipliersToPack.reshape(numberOfLowerBoundConstraints, 1);
      lagrangeLowerBoundConstraintMultipliersToPack.zero();
      lagrangeUpperBoundConstraintMultipliersToPack.reshape(numberOfUpperBoundConstraints, 1);
      lagrangeUpperBoundConstraintMultipliersToPack.zero();

      computeQInverseAndAQInverse();

      solveEqualityConstrainedSubproblemEfficiently(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack,
                                                    lagrangeLowerBoundConstraintMultipliersToPack, lagrangeUpperBoundConstraintMultipliersToPack);

      //      System.out.println(numberOfInequalityConstraints + ", " + numberOfLowerBoundConstraints + ", " + numberOfUpperBoundConstraints);
      if ((numberOfInequalityConstraints == 0) && (numberOfLowerBoundConstraints == 0) && (numberOfUpperBoundConstraints == 0))
         return numberOfIterations;

      // Test the inequality constraints:

      for (int i = 0; i < maxNumberOfIterations; i++)
      {
         boolean activeSetWasModified = modifyActiveSetAndTryAgain(solutionToPack, lagrangeEqualityConstraintMultipliersToPack,
                                                                   lagrangeInequalityConstraintMultipliersToPack, lagrangeLowerBoundConstraintMultipliersToPack,
                                                                   lagrangeUpperBoundConstraintMultipliersToPack);
         numberOfIterations++;

         if (!activeSetWasModified)
            return numberOfIterations;
      }

      for (int i = 0; i < numberOfVariables; i++)
         solutionToPack.set(i, 0, Double.NaN);

      return numberOfIterations;
   }

   private boolean problemSizeChanged()
   {
      boolean sizeChanged = checkProblemSize();

      previousNumberOfVariables = quadraticCostQMatrix.getNumRows();
      previousNumberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      previousNumberOfInequalityConstraints = linearInequalityConstraintsCMatrixO.getNumRows();
      previousNumberOfLowerBoundConstraints = variableLowerBounds.getNumRows();
      previousNumberOfUpperBoundConstraints = variableUpperBounds.getNumRows();

      return sizeChanged;
   }

   private boolean checkProblemSize()
   {
      if (previousNumberOfVariables != quadraticCostQMatrix.getNumRows())
         return true;
      if (previousNumberOfEqualityConstraints != linearEqualityConstraintsAMatrix.getNumRows())
         return true;
      if (previousNumberOfInequalityConstraints != linearInequalityConstraintsCMatrixO.getNumRows())
         return true;
      if (previousNumberOfLowerBoundConstraints != variableLowerBounds.getNumRows())
         return true;
      if (previousNumberOfUpperBoundConstraints != variableUpperBounds.getNumRows())
         return true;

      return false;
   }

   private void computeQInverseAndAQInverse()
   {
      int numberOfVariables = quadraticCostQMatrix.getNumRows();
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();

      ATranspose.reshape(linearEqualityConstraintsAMatrix.getNumCols(), linearEqualityConstraintsAMatrix.getNumRows());
      CommonOps.transpose(linearEqualityConstraintsAMatrix, ATranspose);
      QInverse.reshape(numberOfVariables, numberOfVariables);

      AQInverse.reshape(numberOfEqualityConstraints, numberOfVariables);
      QInverseATranspose.reshape(numberOfVariables, numberOfEqualityConstraints);
      AQInverseATranspose.reshape(numberOfEqualityConstraints, numberOfEqualityConstraints);

      DiagonalMatrixTools.invertDiagonalMatrix(quadraticCostQMatrix, QInverse);

      if (numberOfEqualityConstraints > 0)
      {
         DiagonalMatrixTools.postMult(linearEqualityConstraintsAMatrix, QInverse, AQInverse);
         DiagonalMatrixTools.preMult(QInverse, ATranspose, QInverseATranspose);
         CommonOps.mult(AQInverse, ATranspose, AQInverseATranspose);
      }
   }

   private void computeCBarTempMatrices()
   {
      if (CBar.getNumRows() > 0)
      {
         CBarTranspose.reshape(CBar.getNumCols(), CBar.getNumRows());
         CommonOps.transpose(CBar, CBarTranspose);

         AQInverseCBarTranspose.reshape(AQInverse.getNumRows(), CBarTranspose.getNumCols());
         CommonOps.mult(AQInverse, CBarTranspose, AQInverseCBarTranspose);

         CBarQInverseATranspose.reshape(CBar.getNumRows(), QInverseATranspose.getNumCols());
         CommonOps.mult(CBar, QInverseATranspose, CBarQInverseATranspose);

         CBarQInverse.reshape(CBar.getNumRows(), QInverse.getNumCols());
         QInverseCBarTranspose.reshape(QInverse.getNumRows(), CBarTranspose.getNumCols());

         DiagonalMatrixTools.postMult(CBar, QInverse, CBarQInverse);
         DiagonalMatrixTools.preMult(QInverse, CBarTranspose, QInverseCBarTranspose);

         CBarQInverseCBarTranspose.reshape(CBar.getNumRows(), QInverseCBarTranspose.getNumCols());
         CommonOps.mult(CBar, QInverseCBarTranspose, CBarQInverseCBarTranspose);
      }
      else
      {
         CBarTranspose.reshape(0, 0);
         AQInverseCBarTranspose.reshape(0, 0);
         CBarQInverseATranspose.reshape(0, 0);
         CBarQInverse.reshape(0, 0);
         QInverseCBarTranspose.reshape(0, 0);
         CBarQInverseCBarTranspose.reshape(0, 0);
      }
   }

   private void computeCHatTempMatrices()
   {
      if (CHat.getNumRows() > 0)
      {
         CHatTranspose.reshape(CHat.getNumCols(), CHat.getNumRows());
         CommonOps.transpose(CHat, CHatTranspose);

         AQInverseCHatTranspose.reshape(AQInverse.getNumRows(), CHatTranspose.getNumCols());
         CommonOps.mult(AQInverse, CHatTranspose, AQInverseCHatTranspose);

         CHatQInverseATranspose.reshape(CHat.getNumRows(), QInverseATranspose.getNumCols());
         CommonOps.mult(CHat, QInverseATranspose, CHatQInverseATranspose);

         CHatQInverse.reshape(CHat.getNumRows(), QInverse.getNumCols());
         QInverseCHatTranspose.reshape(QInverse.getNumRows(), CHatTranspose.getNumCols());

         DiagonalMatrixTools.postMult(CHat, QInverse, CHatQInverse);
         DiagonalMatrixTools.preMult(QInverse, CHatTranspose, QInverseCHatTranspose);

         CHatQInverseCHatTranspose.reshape(CHat.getNumRows(), QInverseCHatTranspose.getNumCols());
         CommonOps.mult(CHat, QInverseCHatTranspose, CHatQInverseCHatTranspose);

         CBarQInverseCHatTranspose.reshape(CBar.getNumRows(), CHat.getNumRows());
         CHatQInverseCBarTranspose.reshape(CHat.getNumRows(), CBar.getNumRows());

         if (CBar.getNumRows() > 0)
         {
            CommonOps.mult(CBar, QInverseCHatTranspose, CBarQInverseCHatTranspose);
            CommonOps.mult(CHat, QInverseCBarTranspose, CHatQInverseCBarTranspose);
         }
      }
      else
      {
         CHatTranspose.reshape(0, 0);
         AQInverseCHatTranspose.reshape(0, 0);
         CHatQInverseATranspose.reshape(0, 0);
         CHatQInverse.reshape(0, 0);
         QInverseCHatTranspose.reshape(0, 0);
         CHatQInverseCHatTranspose.reshape(0, 0);
         CBarQInverseCHatTranspose.reshape(0, 0);
         CHatQInverseCBarTranspose.reshape(0, 0);
      }
   }

   private boolean modifyActiveSetAndTryAgain(DenseMatrix64F solutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack,
                                              DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack,
                                              DenseMatrix64F lagrangeLowerBoundConstraintMultipliersToPack,
                                              DenseMatrix64F lagrangeUpperBoundConstraintMultipliersToPack)
   {
      if (containsNaN(solutionToPack))
         return false;

      boolean activeSetWasModified = false;

      int numberOfVariables = quadraticCostQMatrix.getNumRows();
      //      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrixO.getNumRows();
      int numberOfLowerBoundConstraints = variableLowerBounds.getNumRows();
      int numberOfUpperBoundConstraints = variableUpperBounds.getNumRows();

      inequalityIndicesToAddToActiveSet.reset();
      inequalityIndicesToRemoveFromActiveSet.reset();
      if (numberOfInequalityConstraints != 0)
      {

         linearInequalityConstraintsCheck.reshape(numberOfInequalityConstraints, 1);
         CommonOps.mult(linearInequalityConstraintsCMatrixO, solutionToPack, linearInequalityConstraintsCheck);
         CommonOps.subtractEquals(linearInequalityConstraintsCheck, linearInequalityConstraintsDVectorO);

         for (int i = 0; i < numberOfInequalityConstraints; i++)
         {
            if (activeInequalityIndices.contains(i))
               continue; // Only check violation on those that are not active. Otherwise check should just return 0.0, but roundoff could cause problems.
            if (linearInequalityConstraintsCheck.get(i, 0) > epsilon)
            {
               activeSetWasModified = true;
               inequalityIndicesToAddToActiveSet.add(i);
            }
         }

         for (int i = 0; i < activeInequalityIndices.size(); i++)
         {
            int indexToCheck = activeInequalityIndices.get(i);

            double lagrangeMultiplier = lagrangeInequalityConstraintMultipliersToPack.get(indexToCheck);
            if (lagrangeMultiplier < 0.0)
            {
               activeSetWasModified = true;
               inequalityIndicesToRemoveFromActiveSet.add(indexToCheck);
            }
         }
      }

      // Check the Bounds
      lowerBoundIndicesToAddToActiveSet.reset();
      for (int i = 0; i < numberOfLowerBoundConstraints; i++)
      {
         if (activeLowerBoundIndices.contains(i))
            continue; // Only check violation on those that are not active. Otherwise check should just return 0.0, but roundoff could cause problems.

         double solutionVariable = solutionToPack.get(i, 0);
         double lowerBound = this.variableLowerBounds.get(i, 0);
         if (solutionVariable < lowerBound - epsilon)
         {
            activeSetWasModified = true;
            lowerBoundIndicesToAddToActiveSet.add(i);
         }
      }

      upperBoundIndicesToAddToActiveSet.reset();
      for (int i = 0; i < numberOfUpperBoundConstraints; i++)
      {
         if (activeUpperBoundIndices.contains(i))
            continue; // Only check violation on those that are not active. Otherwise check should just return 0.0, but roundoff could cause problems.

         double solutionVariable = solutionToPack.get(i, 0);
         double upperBound = this.variableUpperBounds.get(i, 0);
         if (solutionVariable > upperBound + epsilon)
         {
            activeSetWasModified = true;
            upperBoundIndicesToAddToActiveSet.add(i);
         }
      }

      lowerBoundIndicesToRemoveFromActiveSet.reset();
      for (int i = 0; i < activeLowerBoundIndices.size(); i++)
      {
         int indexToCheck = activeLowerBoundIndices.get(i);

         double lagrangeMultiplier = lagrangeLowerBoundConstraintMultipliersToPack.get(indexToCheck);
         if (lagrangeMultiplier < 0.0)
         {
            activeSetWasModified = true;
            lowerBoundIndicesToRemoveFromActiveSet.add(indexToCheck);
         }
      }

      upperBoundIndicesToRemoveFromActiveSet.reset();
      for (int i = 0; i < activeUpperBoundIndices.size(); i++)
      {
         int indexToCheck = activeUpperBoundIndices.get(i);

         double lagrangeMultiplier = lagrangeUpperBoundConstraintMultipliersToPack.get(indexToCheck);
         //         if ((lagrangeMultiplier < 0) || (Double.isInfinite(lagrangeMultiplier)))
         if (lagrangeMultiplier < 0.0)
         {
            activeSetWasModified = true;
            upperBoundIndicesToRemoveFromActiveSet.add(indexToCheck);
         }
      }

      if (!activeSetWasModified)
         return false;

      for (int i = 0; i < inequalityIndicesToAddToActiveSet.size(); i++)
      {
         activeInequalityIndices.add(inequalityIndicesToAddToActiveSet.get(i));
      }
      for (int i = 0; i < inequalityIndicesToRemoveFromActiveSet.size(); i++)
      {
         activeInequalityIndices.remove(inequalityIndicesToRemoveFromActiveSet.get(i));
      }

      for (int i = 0; i < lowerBoundIndicesToAddToActiveSet.size(); i++)
      {
         activeLowerBoundIndices.add(lowerBoundIndicesToAddToActiveSet.get(i));
      }
      for (int i = 0; i < lowerBoundIndicesToRemoveFromActiveSet.size(); i++)
      {
         activeLowerBoundIndices.remove(lowerBoundIndicesToRemoveFromActiveSet.get(i));
      }

      for (int i = 0; i < upperBoundIndicesToAddToActiveSet.size(); i++)
      {
         activeUpperBoundIndices.add(upperBoundIndicesToAddToActiveSet.get(i));
      }
      for (int i = 0; i < upperBoundIndicesToRemoveFromActiveSet.size(); i++)
      {
         activeUpperBoundIndices.remove(upperBoundIndicesToRemoveFromActiveSet.get(i));
      }

      // Add active set constraints as equality constraints:
      int sizeOfActiveSet = activeInequalityIndices.size();

      CBar.reshape(sizeOfActiveSet, numberOfVariables);
      DBar.reshape(sizeOfActiveSet, 1);

      for (int i = 0; i < sizeOfActiveSet; i++)
      {
         int inequalityConstraintIndex = activeInequalityIndices.get(i);
         CommonOps.extract(linearInequalityConstraintsCMatrixO, inequalityConstraintIndex, inequalityConstraintIndex + 1, 0, numberOfVariables, CBar, i, 0);
         CommonOps.extract(linearInequalityConstraintsDVectorO, inequalityConstraintIndex, inequalityConstraintIndex + 1, 0, 1, DBar, i, 0);
      }

      // Add active bounds constraints as equality constraints:
      int sizeOfLowerBoundsActiveSet = activeLowerBoundIndices.size();
      int sizeOfUpperBoundsActiveSet = activeUpperBoundIndices.size();

      int sizeOfBoundsActiveSet = sizeOfLowerBoundsActiveSet + sizeOfUpperBoundsActiveSet;

      CHat.reshape(sizeOfBoundsActiveSet, numberOfVariables);
      DHat.reshape(sizeOfBoundsActiveSet, 1);

      CHat.zero();
      DHat.zero();

      int row = 0;

      for (int i = 0; i < sizeOfLowerBoundsActiveSet; i++)
      {
         int lowerBoundsConstraintIndex = activeLowerBoundIndices.get(i);

         CHat.set(row, lowerBoundsConstraintIndex, -1.0);
         DHat.set(row, 0, -variableLowerBounds.get(lowerBoundsConstraintIndex));
         row++;
      }

      for (int i = 0; i < sizeOfUpperBoundsActiveSet; i++)
      {
         int upperBoundsConstraintIndex = activeUpperBoundIndices.get(i);

         CHat.set(row, upperBoundsConstraintIndex, 1.0);
         DHat.set(row, 0, variableUpperBounds.get(upperBoundsConstraintIndex));
         row++;
      }

      solveEqualityConstrainedSubproblemEfficiently(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack,
                                                    lagrangeLowerBoundConstraintMultipliersToPack, lagrangeUpperBoundConstraintMultipliersToPack);

      return true;
   }

   private boolean containsNaN(DenseMatrix64F solution)
   {
      for (int i = 0; i < solution.getNumRows(); i++)
      {
         if (Double.isNaN(solution.get(i, 0)))
            return true;
      }

      return false;
   }

   private void solveEqualityConstrainedSubproblemEfficiently(DenseMatrix64F xSolutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack,
                                                              DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack,
                                                              DenseMatrix64F lagrangeLowerBoundConstraintMultipliersToPack,
                                                              DenseMatrix64F lagrangeUpperBoundConstraintMultipliersToPack)
   {
      int numberOfVariables = quadraticCostQMatrix.getNumRows();
      int numberOfOriginalEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();

      int numberOfActiveInequalityConstraints = activeInequalityIndices.size();
      int numberOfActiveLowerBoundConstraints = activeLowerBoundIndices.size();
      int numberOfActiveUpperBoundConstraints = activeUpperBoundIndices.size();

      int numberOfAugmentedEqualityConstraints = numberOfOriginalEqualityConstraints + numberOfActiveInequalityConstraints + numberOfActiveLowerBoundConstraints
            + numberOfActiveUpperBoundConstraints;

      negativeQuadraticCostQVector.set(quadraticCostQVector);
      CommonOps.scale(-1.0, negativeQuadraticCostQVector);

      if (numberOfAugmentedEqualityConstraints == 0)
      {
         DiagonalMatrixTools.preMult(QInverse, negativeQuadraticCostQVector, xSolutionToPack);
         return;
      }

      computeCBarTempMatrices();
      computeCHatTempMatrices();

      bigMatrixForLagrangeMultiplierSolution.reshape(numberOfAugmentedEqualityConstraints, numberOfAugmentedEqualityConstraints);
      bigVectorForLagrangeMultiplierSolution.reshape(numberOfAugmentedEqualityConstraints, 1);

      CommonOps.insert(AQInverseATranspose, bigMatrixForLagrangeMultiplierSolution, 0, 0);
      CommonOps.insert(AQInverseCBarTranspose, bigMatrixForLagrangeMultiplierSolution, 0, numberOfOriginalEqualityConstraints);
      CommonOps.insert(AQInverseCHatTranspose, bigMatrixForLagrangeMultiplierSolution, 0,
                       numberOfOriginalEqualityConstraints + numberOfActiveInequalityConstraints);

      CommonOps.insert(CBarQInverseATranspose, bigMatrixForLagrangeMultiplierSolution, numberOfOriginalEqualityConstraints, 0);
      CommonOps.insert(CBarQInverseCBarTranspose, bigMatrixForLagrangeMultiplierSolution, numberOfOriginalEqualityConstraints,
                       numberOfOriginalEqualityConstraints);
      CommonOps.insert(CBarQInverseCHatTranspose, bigMatrixForLagrangeMultiplierSolution, numberOfOriginalEqualityConstraints,
                       numberOfOriginalEqualityConstraints + numberOfActiveInequalityConstraints);

      CommonOps.insert(CHatQInverseATranspose, bigMatrixForLagrangeMultiplierSolution,
                       numberOfOriginalEqualityConstraints + numberOfActiveInequalityConstraints, 0);
      CommonOps.insert(CHatQInverseCBarTranspose, bigMatrixForLagrangeMultiplierSolution,
                       numberOfOriginalEqualityConstraints + numberOfActiveInequalityConstraints, numberOfOriginalEqualityConstraints);
      CommonOps.insert(CHatQInverseCHatTranspose, bigMatrixForLagrangeMultiplierSolution,
                       numberOfOriginalEqualityConstraints + numberOfActiveInequalityConstraints,
                       numberOfOriginalEqualityConstraints + numberOfActiveInequalityConstraints);

      if (numberOfOriginalEqualityConstraints > 0)
      {
         tempVector.reshape(numberOfOriginalEqualityConstraints, 1);
         CommonOps.mult(AQInverse, quadraticCostQVector, tempVector);
         CommonOps.addEquals(tempVector, linearEqualityConstraintsBVector);
         CommonOps.scale(-1.0, tempVector);

         CommonOps.insert(tempVector, bigVectorForLagrangeMultiplierSolution, 0, 0);
      }

      if (numberOfActiveInequalityConstraints > 0)
      {
         tempVector.reshape(numberOfActiveInequalityConstraints, 1);
         CommonOps.mult(CBarQInverse, quadraticCostQVector, tempVector);
         CommonOps.addEquals(tempVector, DBar);
         CommonOps.scale(-1.0, tempVector);

         CommonOps.insert(tempVector, bigVectorForLagrangeMultiplierSolution, numberOfOriginalEqualityConstraints, 0);
      }

      if (numberOfActiveLowerBoundConstraints + numberOfActiveUpperBoundConstraints > 0)
      {
         tempVector.reshape(numberOfActiveLowerBoundConstraints + numberOfActiveUpperBoundConstraints, 1);
         CommonOps.mult(CHatQInverse, quadraticCostQVector, tempVector);
         CommonOps.addEquals(tempVector, DHat);
         CommonOps.scale(-1.0, tempVector);

         CommonOps.insert(tempVector, bigVectorForLagrangeMultiplierSolution, numberOfOriginalEqualityConstraints + numberOfActiveInequalityConstraints, 0);
      }

      augmentedLagrangeMultipliers.reshape(numberOfAugmentedEqualityConstraints, 1);
      solver.setA(bigMatrixForLagrangeMultiplierSolution);
      solver.solve(bigVectorForLagrangeMultiplierSolution, augmentedLagrangeMultipliers);

      ATransposeAndCTranspose.reshape(numberOfVariables, numberOfAugmentedEqualityConstraints);
      CommonOps.insert(ATranspose, ATransposeAndCTranspose, 0, 0);
      CommonOps.insert(CBarTranspose, ATransposeAndCTranspose, 0, numberOfOriginalEqualityConstraints);
      CommonOps.insert(CHatTranspose, ATransposeAndCTranspose, 0, numberOfOriginalEqualityConstraints + numberOfActiveInequalityConstraints);

      ATransposeMuAndCTransposeLambda.reshape(numberOfVariables, 1);
      CommonOps.mult(ATransposeAndCTranspose, augmentedLagrangeMultipliers, ATransposeMuAndCTransposeLambda);

      tempVector.set(quadraticCostQVector);
      CommonOps.scale(-1.0, tempVector);
      CommonOps.subtractEquals(tempVector, ATransposeMuAndCTransposeLambda);

      DiagonalMatrixTools.preMult(QInverse, tempVector, xSolutionToPack);

      int startRow = 0;
      int numberOfRows = numberOfOriginalEqualityConstraints;
      CommonOps.extract(augmentedLagrangeMultipliers, startRow, startRow + numberOfRows, 0, 1, lagrangeEqualityConstraintMultipliersToPack, 0, 0);

      startRow += numberOfRows;
      lagrangeInequalityConstraintMultipliersToPack.zero();
      for (int i = 0; i < numberOfActiveInequalityConstraints; i++)
      {
         int inequalityConstraintIndex = activeInequalityIndices.get(i);
         CommonOps.extract(augmentedLagrangeMultipliers, startRow + i, startRow + i + 1, 0, 1, lagrangeInequalityConstraintMultipliersToPack,
                           inequalityConstraintIndex, 0);
      }

      startRow += numberOfActiveInequalityConstraints;
      lagrangeLowerBoundConstraintMultipliersToPack.zero();
      for (int i = 0; i < numberOfActiveLowerBoundConstraints; i++)
      {
         int lowerBoundConstraintIndex = activeLowerBoundIndices.get(i);
         CommonOps.extract(augmentedLagrangeMultipliers, startRow + i, startRow + i + 1, 0, 1, lagrangeLowerBoundConstraintMultipliersToPack,
                           lowerBoundConstraintIndex, 0);
      }

      startRow += numberOfActiveLowerBoundConstraints;
      lagrangeUpperBoundConstraintMultipliersToPack.zero();
      for (int i = 0; i < numberOfActiveUpperBoundConstraints; i++)
      {
         int upperBoundConstraintIndex = activeUpperBoundIndices.get(i);
         CommonOps.extract(augmentedLagrangeMultipliers, startRow + i, startRow + i + 1, 0, 1, lagrangeUpperBoundConstraintMultipliersToPack,
                           upperBoundConstraintIndex, 0);
      }
   }
}
