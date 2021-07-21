package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import gnu.trove.list.TIntList;
import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.convexOptimization.IntermediateSolutionListener;
import us.ihmc.convexOptimization.quadraticProgram.InverseMatrixCalculator;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeMatrix;

import java.lang.annotation.Native;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Solves a Quadratic Program using a simple active set method. Does not work for problems where
 * having multiple inequality constraints in the active set make the problem infeasible. Seems to
 * work well for problems with benign inequality constraints, such as variable bounds. Algorithm is
 * very fast when it can find a solution. Uses the algorithm and naming convention found in MIT
 * Paper "An efficiently solvable quadratic program for stabilizing dynamic locomotion" by Scott
 * Kuindersma, Frank Permenter, and Russ Tedrake.
 *
 * @author JerryPratt
 */
public class MPCQPSolver
{
   private static  final boolean debug = true;

   private static final double violationFractionToAdd = 0.95;
   private static final double violationFractionToRemove = 0.8;
   //private static final double violationFractionToAdd = 1.0;
   //private static final double violationFractionToRemove = 1.0;
   private double convergenceThreshold = 1e-4;
   //private double convergenceThresholdForLagrangeMultipliers = 0.0;
   private double convergenceThresholdForLagrangeMultipliers = 1e-3;
   private int maxNumberOfIterations = 10;
   private boolean reportFailedConvergenceAsNaN = true;
   private boolean resetActiveSetOnSizeChange = true;

//   protected double quadraticCostScalar;

   private final RowMajorNativeMatrixGrower nativeMatrixGrower = new RowMajorNativeMatrixGrower();

   private final DMatrixRMaj activeVariables = new DMatrixRMaj(0, 0);

   private final TIntArrayList activeInequalityIndices = new TIntArrayList();

   // Some temporary matrices:
   protected final NativeMatrix nativexSolutionMatrix = new NativeMatrix(0, 0);
   public final NativeMatrix costQuadraticMatrix = new NativeMatrix(0, 0);

   private final NativeMatrix linearInequalityConstraintsCheck = new NativeMatrix(0, 0);

   public final NativeMatrix quadraticCostQVector = new NativeMatrix(0, 0);
   public final NativeMatrix quadraticCostQMatrix = new NativeMatrix(0, 0);
   public final NativeMatrix linearEqualityConstraintsAMatrix = new NativeMatrix(0, 0);
   public final NativeMatrix linearEqualityConstraintsBVector = new NativeMatrix(0, 0);

   public final NativeMatrix linearInequalityConstraintsCMatrixO = new NativeMatrix(0, 0);
   public final NativeMatrix linearInequalityConstraintsDVectorO = new NativeMatrix(0, 0);
   public final DMatrixRMaj linearInequalityConstraintsSlackVariableCost = new DMatrixRMaj(0, 0);

   /** Active inequality constraints */
   private final NativeMatrix CBar = new NativeMatrix(0, 0);
   private final NativeMatrix DBar = new NativeMatrix(0, 0);
   private final DMatrixRMaj slackBar = new DMatrixRMaj(0, 0);

   private final NativeMatrix inverseSlackHessian = new NativeMatrix(0, 0);
   private final NativeMatrix QInverse = new NativeMatrix(0, 0);
   private final NativeMatrix AQInverse = new NativeMatrix(0, 0);
   private final NativeMatrix QInverseATranspose = new NativeMatrix(0, 0);
   private final NativeMatrix CBarQInverse = new NativeMatrix(0, 0);
   private final NativeMatrix AQInverseATranspose = new NativeMatrix(0, 0);
   private final NativeMatrix AQInverseCBarTranspose = new NativeMatrix(0, 0);
   private final NativeMatrix CBarQInverseATranspose = new NativeMatrix(0, 0);
   private final NativeMatrix QInverseCBarTranspose = new NativeMatrix(0, 0);
   private final NativeMatrix CBarQInverseCBarTranspose = new NativeMatrix(0, 0);

   private final NativeMatrix AAndC = new NativeMatrix(0, 0);
   private final NativeMatrix ATransposeMuAndCTransposeLambda = new NativeMatrix(0, 0);

   private final NativeMatrix bigMatrixForLagrangeMultiplierSolution = new NativeMatrix(0, 0);
   private final NativeMatrix bigVectorForLagrangeMultiplierSolution = new NativeMatrix(0, 0);

   private final NativeMatrix tempVector = new NativeMatrix(0, 0);
   private final NativeMatrix augmentedLagrangeMultipliers = new NativeMatrix(0, 0);

   private final TIntArrayList inequalityIndicesToAddToActiveSet = new TIntArrayList();
   private final TIntArrayList inequalityIndicesToRemoveFromActiveSet = new TIntArrayList();

   protected final NativeMatrix computedObjectiveFunctionValue = new NativeMatrix(1, 1);

   private InverseMatrixCalculator<NativeMatrix> inverseSolver = new DefaultInverseMatrixCalculator();

   private boolean useWarmStart = false;

   private int previousNumberOfVariables = 0;
   private int previousNumberOfEqualityConstraints = 0;
   private int previousNumberOfInequalityConstraints = 0;

   private final List<IntermediateSolutionListener> solutionListeners = new ArrayList<>();

   public void setConvergenceThreshold(double convergenceThreshold)
   {
      this.convergenceThreshold = convergenceThreshold;
   }

   public void setConvergenceThresholdForLagrangeMultipliers(double convergenceThresholdForLagrangeMultipliers)
   {
      this.convergenceThresholdForLagrangeMultipliers = convergenceThresholdForLagrangeMultipliers;
   }

   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      this.maxNumberOfIterations = maxNumberOfIterations;
   }

   public void addIntermediateSolutionListener(IntermediateSolutionListener solutionListener)
   {
      this.solutionListeners.add(solutionListener);
   }

   public void setReportFailedConvergenceAsNaN(boolean reportFailedConvergenceAsNaN)
   {
      this.reportFailedConvergenceAsNaN = reportFailedConvergenceAsNaN;
   }

   public void setResetActiveSetOnSizeChange(boolean resetActiveSetOnSizeChange)
   {
      this.resetActiveSetOnSizeChange = resetActiveSetOnSizeChange;
   }

   public void setActiveInequalityIndices(TIntList activeInequalityIndices)
   {
      this.activeInequalityIndices.reset();
      for (int i = 0; i < activeInequalityIndices.size(); i++)
         this.activeInequalityIndices.add(activeInequalityIndices.get(i));
   }

   public TIntList getActiveInequalityIndices()
   {
      return activeInequalityIndices;
   }

   public void initialize(int problemSize)
   {
      costQuadraticMatrix.reshape(problemSize, problemSize);
      quadraticCostQVector.reshape(problemSize, 1);

      linearEqualityConstraintsAMatrix.zero();
      linearEqualityConstraintsBVector.zero();

      linearInequalityConstraintsCMatrixO.zero();
      linearInequalityConstraintsDVectorO.zero();
      linearInequalityConstraintsSlackVariableCost.zero();

      linearEqualityConstraintsAMatrix.reshape(0, problemSize);
      linearEqualityConstraintsBVector.reshape(0, 1);

      linearInequalityConstraintsCMatrixO.reshape(0, problemSize);
      linearInequalityConstraintsDVectorO.reshape(0, 1);
      linearInequalityConstraintsSlackVariableCost.reshape(0, 1);

      costQuadraticMatrix.zero();
      quadraticCostQVector.zero();
   }

   public void addRegularization(int startIndex, int numberOfVariables, double value)
   {
      for (int i = 0; i < numberOfVariables; i++)
         costQuadraticMatrix.add(startIndex + i, startIndex + i, value);
   }

   public void addRateRegularization(int startIndex, int numberOfVariables, double value, DMatrix previousSolution)
   {
      for (int i = 0; i < numberOfVariables; i++)
      {
         double previousValue = previousSolution.get(startIndex + i, 0);
         if (Double.isNaN(previousValue))
            continue;
         costQuadraticMatrix.add(startIndex + i, startIndex + i, value);
         quadraticCostQVector.add(startIndex + i, 0, -previousValue * value);
      }
   }


   public void addObjective(NativeMatrix taskJacobian, NativeMatrix taskObjective, double taskWeight, int offset)
   {
      addObjective(taskJacobian, taskObjective, taskWeight, taskJacobian.getNumCols(), offset);
   }

   public void addObjective(NativeMatrix taskJacobian, NativeMatrix taskObjective, double taskWeight, int problemSize, int offset)
   {
      if (taskJacobian.getNumCols() != problemSize)
      {
         throw new RuntimeException("Motion task needs to have size matching the DoFs of the robot.");
      }
      int variables = taskJacobian.getNumCols();
      if (variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      // Compute: H += J^T W J
      // TODO figure out an efficient inner product in eigen
      costQuadraticMatrix.multAddBlockTransA(taskWeight, taskJacobian, taskJacobian, offset, offset);
      if (debug && costQuadraticMatrix.containsNaN())
         throw new RuntimeException("error");

      // Compute: f += - J^T W Objective
      quadraticCostQVector.multAddBlockTransA(-taskWeight, taskJacobian, taskObjective, offset, 0);
      if (debug && quadraticCostQVector.containsNaN())
         throw new RuntimeException("error");
   }

   private final NativeMatrix tempJtW = new NativeMatrix(0, 0);

   public void addObjective(NativeMatrix taskJacobian, NativeMatrix taskObjective, NativeMatrix taskWeight, int offset)
   {
      addObjective(taskJacobian, taskObjective, taskWeight, taskJacobian.getNumCols(), offset);
   }

   public void addObjective(NativeMatrix taskJacobian, NativeMatrix taskObjective, NativeMatrix taskWeight, int problemSize, int offset)
   {
      int taskSize = taskJacobian.getNumRows();
      if (taskJacobian.getNumCols() != problemSize)
      {
         throw new RuntimeException("Motion task needs to have size matching the DoFs of the robot.");
      }
      int variables = taskJacobian.getNumCols();
      if (variables > problemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      tempJtW.reshape(variables, taskSize);

      // J^T W
      tempJtW.multTransA(taskJacobian, taskWeight);

      // Compute: H += J^T W J
      costQuadraticMatrix.multAddBlock(tempJtW, taskJacobian, offset, offset);
      if (debug && costQuadraticMatrix.containsNaN())
         throw new RuntimeException("error");

      // Compute: f += - J^T W Objective
      quadraticCostQVector.multAddBlock(-1.0, tempJtW, taskObjective, offset, 0);
      if (debug && quadraticCostQVector.containsNaN())
         throw new RuntimeException("error");
   }

   public void addDirectObjective(NativeMatrix directCostHessian, NativeMatrix directCostGradient, double weightScalar, int offset)
   {
      int size = directCostHessian.getNumCols();
      costQuadraticMatrix.addBlock(directCostHessian, offset, offset, 0, 0, size, size, weightScalar);
      if (debug && costQuadraticMatrix.containsNaN())
         throw new RuntimeException("error");
      quadraticCostQVector.addBlock(directCostGradient, offset, 0, 0, 0, size, 1, weightScalar);
      if (debug && quadraticCostQVector.containsNaN())
         throw new RuntimeException("Error");
   }

   public void addEqualityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective, int problemSize, int taskColOffset)
   {
      addEqualityConstraint(taskJacobian, taskObjective, taskJacobian.getNumCols(), problemSize, taskColOffset);
   }

   public void addEqualityConstraint(NativeMatrix taskJacobian,
                                     NativeMatrix taskObjective,
                                     int problemSize,
                                     int totalProblemSize,
                                     int colOffset)
   {
      if (taskJacobian.getNumCols() != problemSize)
      {
         throw new RuntimeException("Motion task needs to have size matching the DoFs of the robot.");
      }

      int variables = taskJacobian.getNumCols();
      if (variables + colOffset > totalProblemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      nativeMatrixGrower.appendRows(linearEqualityConstraintsAMatrix, colOffset, taskJacobian);
      nativeMatrixGrower.appendRows(linearEqualityConstraintsBVector, taskObjective);

      if (debug && linearEqualityConstraintsAMatrix.containsNaN())
         throw new RuntimeException("error");
      if (debug && linearEqualityConstraintsBVector.containsNaN())
         throw new RuntimeException("error");
   }

   public void addMotionLesserOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective, int problemSize, int colOffset)
   {
      addMotionLesserOrEqualInequalityConstraint(taskJacobian, taskObjective, taskJacobian.getNumCols(), problemSize, colOffset);
   }

   public void addMotionLesserOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective, double slackVariableWeight, int problemSize, int colOffset)
   {
      addMotionLesserOrEqualInequalityConstraint(taskJacobian, taskObjective, slackVariableWeight, taskJacobian.getNumCols(), problemSize, colOffset);
   }

   public void addMotionLesserOrEqualInequalityConstraint(NativeMatrix taskJacobian,
                                                          NativeMatrix taskObjective,
                                                          int problemSize,
                                                          int totalProblemSize,
                                                          int colOffset)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, 1.0, problemSize, totalProblemSize, colOffset);
   }

   public void addMotionLesserOrEqualInequalityConstraint(NativeMatrix taskJacobian,
                                                          NativeMatrix taskObjective,
                                                          double slackVariableWeight,
                                                          int problemSize,
                                                          int totalProblemSize,
                                                          int colOffset)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, 1.0, slackVariableWeight, problemSize, totalProblemSize, colOffset);
   }

   public void addMotionGreaterOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective, int problemSize, int colOffset)
   {
      addMotionGreaterOrEqualInequalityConstraint(taskJacobian, taskObjective, taskJacobian.getNumCols(), problemSize, colOffset);
   }

   public void addMotionGreaterOrEqualInequalityConstraint(NativeMatrix taskJacobian, NativeMatrix taskObjective, double slackVariableWeight, int problemSize, int colOffset)
   {
      addMotionGreaterOrEqualInequalityConstraint(taskJacobian, taskObjective, slackVariableWeight, taskJacobian.getNumCols(), problemSize, colOffset);
   }

   public void addMotionGreaterOrEqualInequalityConstraint(NativeMatrix taskJacobian,
                                                           NativeMatrix taskObjective,
                                                           int problemSize,
                                                           int totalProblemSize,
                                                           int colOffset)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, -1.0, problemSize, totalProblemSize, colOffset);
   }
   
   public void addMotionGreaterOrEqualInequalityConstraint(NativeMatrix taskJacobian,
                                                           NativeMatrix taskObjective,
                                                           double slackVariableWeight,
                                                           int problemSize,
                                                           int totalProblemSize,
                                                           int colOffset)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, -1.0, slackVariableWeight, problemSize, totalProblemSize, colOffset);
   }

   private void addInequalityConstraintInternal(NativeMatrix taskJacobian,
                                                NativeMatrix taskObjective,
                                                double sign,
                                                int problemSize,
                                                int totalProblemSize,
                                                int colOffset)
   {
      addInequalityConstraintInternal(taskJacobian, taskObjective, sign, Double.NaN, problemSize, totalProblemSize, colOffset);
   }

   private void addInequalityConstraintInternal(NativeMatrix taskJacobian,
                                                NativeMatrix taskObjective,
                                                double sign,
                                                double slackVariableWeight,
                                                int problemSize,
                                                int totalProblemSize,
                                                int colOffset)
   {
      int variables = taskJacobian.getNumCols();
      if (taskJacobian.getNumCols() != problemSize)
      {
         throw new RuntimeException("Motion task needs to have size matching the DoFs of the robot.");
      }
      if (variables > totalProblemSize)
      {
         throw new RuntimeException("This task does not fit.");
      }

      nativeMatrixGrower.appendRows(linearInequalityConstraintsCMatrixO, colOffset, sign, taskJacobian);
      nativeMatrixGrower.appendRows(linearInequalityConstraintsDVectorO, sign, taskObjective);

      int previousSize = linearInequalityConstraintsSlackVariableCost.getNumRows();
      int taskSize = taskObjective.getNumRows();
      linearInequalityConstraintsSlackVariableCost.reshape(previousSize + taskSize, 1, true);
      Arrays.fill(linearInequalityConstraintsSlackVariableCost.data, previousSize, previousSize + taskSize, slackVariableWeight);
   }

   public double getObjectiveCost(DMatrixRMaj x)
   {
      nativexSolutionMatrix.set(x);

      computedObjectiveFunctionValue.multQuad(nativexSolutionMatrix, quadraticCostQMatrix);
      computedObjectiveFunctionValue.scale(0.5);

      computedObjectiveFunctionValue.multAddTransA(quadraticCostQVector, nativexSolutionMatrix);
      return computedObjectiveFunctionValue.get(0, 0);// + quadraticCostScalar;
   }

   public int getNumberOfEqualityConstraints()
   {
      return linearEqualityConstraintsAMatrix.getNumRows();
   }

   public int getNumberOfInequalityConstraints()
   {
      return linearInequalityConstraintsCMatrixO.getNumRows();
   }

   private void assertSizesCorrect()
   {
      if (quadraticCostQVector.getNumCols() != 1)
         throw new RuntimeException("costLinearVector.getNumCols() != 1");
      if (costQuadraticMatrix.getNumRows() != quadraticCostQVector.getNumRows())
         throw new RuntimeException("costQuadraticMatrix.getNumRows() != costLinearVector.getNumRows()");
      if (costQuadraticMatrix.getNumRows() != costQuadraticMatrix.getNumCols())
         throw new RuntimeException("costQuadraticMatrix.getNumRows() != costQuadraticMatrix.getNumCols()");

      if (linearEqualityConstraintsBVector.getNumCols() != 1)
         throw new RuntimeException("linearEqualityConstraintsBVector.getNumCols() != 1");
      if (linearEqualityConstraintsAMatrix.getNumRows() != linearEqualityConstraintsBVector.getNumRows())
         throw new RuntimeException("linearEqualityConstraintsAMatrix.getNumRows() != linearEqualityConstraintsBVector.getNumRows()");
      if (linearEqualityConstraintsAMatrix.getNumCols() != costQuadraticMatrix.getNumCols())
         throw new RuntimeException("linearEqualityConstraintsAMatrix.getNumCols() != quadraticCostQMatrix.getNumCols()");

      if (linearInequalityConstraintsDVectorO.getNumCols() != 1)
         throw new RuntimeException("linearInequalityConstraintDVector.getNumCols() != 1");
      if (linearInequalityConstraintsCMatrixO.getNumRows() != linearInequalityConstraintsDVectorO.getNumRows())
         throw new RuntimeException("linearInequalityConstraintCMatrix.getNumRows() != linearInequalityConstraintDVector.getNumRows()");
      if (linearInequalityConstraintsCMatrixO.getNumCols() != costQuadraticMatrix.getNumCols())
         throw new RuntimeException("linearInequalityConstraintCMatrix.getNumCols() != quadraticCostQMatrix.getNumCols()");
   }

   private void computeSymmetricHessian()
   {
      quadraticCostQMatrix.transpose(this.costQuadraticMatrix);
      quadraticCostQMatrix.addEquals(this.costQuadraticMatrix);
      quadraticCostQMatrix.scale(0.5);
   }

   public void setUseWarmStart(boolean useWarmStart)
   {
      this.useWarmStart = useWarmStart;
   }

   public void resetActiveSet()
   {
      CBar.reshape(0, 0);
      DBar.reshape(0, 0);

      activeInequalityIndices.reset();
   }

   private final NativeMatrix lagrangeEqualityConstraintMultipliers = new NativeMatrix(0, 0);
   private final NativeMatrix lagrangeInequalityConstraintMultipliers = new NativeMatrix(0, 0);

   public int solve(DMatrix solutionToPack)
   {
//      printActiveSetInfo("At beginning " );

      // TODO CHECK SIZE
      assertSizesCorrect();
      computeSymmetricHessian();

      if (!useWarmStart || (resetActiveSetOnSizeChange && problemSizeChanged()))
         resetActiveSet();
      else
         addActiveSetConstraintsAsEqualityConstraints();

      int numberOfIterations = 0;

      int numberOfVariables = quadraticCostQMatrix.getNumRows();
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrixO.getNumRows();

      nativexSolutionMatrix.reshape(numberOfVariables, 1);
      lagrangeEqualityConstraintMultipliers.reshape(numberOfEqualityConstraints, 1);
      lagrangeEqualityConstraintMultipliers.zero();
      lagrangeInequalityConstraintMultipliers.reshape(numberOfInequalityConstraints, 1);
      lagrangeInequalityConstraintMultipliers.zero();

      computeQInverseAndAQInverse();

      solveEqualityConstrainedSubproblemEfficiently(nativexSolutionMatrix,
                                                    lagrangeEqualityConstraintMultipliers,
                                                    lagrangeInequalityConstraintMultipliers);

      //      System.out.println(numberOfInequalityConstraints + ", " + numberOfLowerBoundConstraints + ", " + numberOfUpperBoundConstraints);
      if (numberOfInequalityConstraints == 0)
      {
         solutionToPack.set(nativexSolutionMatrix);
         return numberOfIterations;
      }

      for (int i = 0; i < maxNumberOfIterations; i++)
      {
         boolean activeSetWasModified = modifyActiveSetAndTryAgain(nativexSolutionMatrix,
                                                                   lagrangeEqualityConstraintMultipliers,
                                                                   lagrangeInequalityConstraintMultipliers);
         numberOfIterations++;

         if (!activeSetWasModified)
         {
            solutionToPack.set(nativexSolutionMatrix);
//            printActiveSetInfo("At end " );

            return numberOfIterations;
         }
      }

      // No solution found. Pack NaN in all variables
      if (reportFailedConvergenceAsNaN)
      {
         if (solutionToPack.getNumRows() != numberOfVariables)
            throw new IllegalArgumentException("Bad number of rows.");
         for (int i = 0; i < numberOfVariables; i++)
            solutionToPack.set(i, 0, Double.NaN);
      }
      else
      {
         solutionToPack.set(nativexSolutionMatrix);
      }

      printActiveSetInfo("At end " );
      return numberOfIterations;
   }

   private void printActiveSetInfo(String prefix)
   {
      LogTools.info(prefix + " Active set size is " + activeInequalityIndices.size() + " out of " + getNumberOfInequalityConstraints() + " constraints.");
   }

   private boolean problemSizeChanged()
   {
      boolean sizeChanged = checkProblemSize();

      previousNumberOfVariables = (int) CommonOps_DDRM.elementSum(activeVariables);
      previousNumberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();
      previousNumberOfInequalityConstraints = linearInequalityConstraintsCMatrixO.getNumRows();

      return sizeChanged;
   }

   private boolean checkProblemSize()
   {
      if (previousNumberOfVariables != CommonOps_DDRM.elementSum(activeVariables))
         return true;
      if (previousNumberOfEqualityConstraints != linearEqualityConstraintsAMatrix.getNumRows())
         return true;
      if (previousNumberOfInequalityConstraints != linearInequalityConstraintsCMatrixO.getNumRows())
         return true;

      return false;
   }

   private void computeQInverseAndAQInverse()
   {
      int numberOfVariables = quadraticCostQMatrix.getNumRows();
      int numberOfEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();

      inverseSolver.computeInverse(quadraticCostQMatrix, QInverse);

      if (numberOfEqualityConstraints > 0)
      {
         AQInverse.mult(linearEqualityConstraintsAMatrix, QInverse);
         QInverseATranspose.multTransB(QInverse, linearEqualityConstraintsAMatrix);
         AQInverseATranspose.multTransB(AQInverse, linearEqualityConstraintsAMatrix);
      }
      else
      {
         AQInverse.reshape(numberOfEqualityConstraints, numberOfVariables);
         QInverseATranspose.reshape(numberOfVariables, numberOfEqualityConstraints);
         AQInverseATranspose.reshape(numberOfEqualityConstraints, numberOfEqualityConstraints);
      }
   }

   private void computeCBarTempMatrices()
   {
      int size = CBar.getNumRows();
      if (size > 0)
      {
         CBarQInverseATranspose.mult(CBar, QInverseATranspose);
         AQInverseCBarTranspose.transpose(CBarQInverseATranspose);

         CBarQInverse.mult(CBar, QInverse);
         QInverseCBarTranspose.transpose(CBarQInverse);

         CBarQInverseCBarTranspose.mult(CBar, QInverseCBarTranspose);

         inverseSlackHessian.reshape(size, size);
         inverseSlackHessian.zero();

         for (int i = 0; i < size; i++)
         {
            double slackCost = slackBar.get(i, 0);
            if (!MathTools.epsilonEquals(slackCost, 0.0, 1e-5) && Double.isFinite(slackCost))
               inverseSlackHessian.set(i, i, 1.0 / slackCost);
         }
//
         CBarQInverseCBarTranspose.addEquals(inverseSlackHessian);
      }
      else
      {
         AQInverseCBarTranspose.reshape(0, 0);
         CBarQInverseATranspose.reshape(0, 0);
         CBarQInverse.reshape(0, 0);
         QInverseCBarTranspose.reshape(0, 0);
         CBarQInverseCBarTranspose.reshape(0, 0);
      }
   }

   private boolean modifyActiveSetAndTryAgain(NativeMatrix solutionToPack, NativeMatrix lagrangeEqualityConstraintMultipliersToPack,
                                              NativeMatrix lagrangeInequalityConstraintMultipliersToPack)
   {
      if (solutionToPack.containsNaN())
         return false;

      boolean activeSetWasModified = false;

      // find the constraints to add
      int numberOfInequalityConstraints = linearInequalityConstraintsCMatrixO.getNumRows();

      double maxInequalityViolation = Double.NEGATIVE_INFINITY;
      if (numberOfInequalityConstraints != 0)
      {
         linearInequalityConstraintsCheck.scale(-1.0, linearInequalityConstraintsDVectorO);
         linearInequalityConstraintsCheck.multAdd(linearInequalityConstraintsCMatrixO, solutionToPack);

         for (int i = 0; i < linearInequalityConstraintsCheck.getNumRows(); i++)
         {
            if (activeInequalityIndices.contains(i))
               continue;

            if (linearInequalityConstraintsCheck.get(i, 0) >= maxInequalityViolation)
               maxInequalityViolation = linearInequalityConstraintsCheck.get(i, 0);
         }
      }

      double minViolationToAdd = (1.0 - violationFractionToAdd) * maxInequalityViolation + convergenceThreshold;

      // check inequality constraints
      inequalityIndicesToAddToActiveSet.reset();
      if (maxInequalityViolation > minViolationToAdd)
      {
         for (int i = 0; i < numberOfInequalityConstraints; i++)
         {
            if (activeInequalityIndices.contains(i))
               continue; // Only check violation on those that are not active. Otherwise check should just return 0.0, but roundoff could cause problems.

            if (linearInequalityConstraintsCheck.get(i, 0) > minViolationToAdd)
            {
               activeSetWasModified = true;
               inequalityIndicesToAddToActiveSet.add(i);
            }
         }
      }

      // find the constraints to remove
      int numberOfActiveInequalityConstraints = activeInequalityIndices.size();

      double minLagrangeInequalityMultiplier = Double.POSITIVE_INFINITY;

      if (numberOfActiveInequalityConstraints != 0)
         minLagrangeInequalityMultiplier = lagrangeInequalityConstraintMultipliersToPack.min();

      double maxLagrangeMultiplierToRemove = -(1.0 - violationFractionToRemove) * minLagrangeInequalityMultiplier - convergenceThresholdForLagrangeMultipliers;

      inequalityIndicesToRemoveFromActiveSet.reset();
      if (minLagrangeInequalityMultiplier < maxLagrangeMultiplierToRemove)
      {
         for (int i = 0; i < activeInequalityIndices.size(); i++)
         {
            int indexToCheck = activeInequalityIndices.get(i);

            double lagrangeMultiplier = lagrangeInequalityConstraintMultipliersToPack.get(indexToCheck, 0);
            if (lagrangeMultiplier < maxLagrangeMultiplierToRemove)
            {
               activeSetWasModified = true;
               inequalityIndicesToRemoveFromActiveSet.add(indexToCheck);
            }
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

      // Add active set constraints as equality constraints:
      addActiveSetConstraintsAsEqualityConstraints();

      solveEqualityConstrainedSubproblemEfficiently(solutionToPack,
                                                    lagrangeEqualityConstraintMultipliersToPack,
                                                    lagrangeInequalityConstraintMultipliersToPack);

      return true;
   }

   private void addActiveSetConstraintsAsEqualityConstraints()
   {
      int numberOfVariables = quadraticCostQMatrix.getNumRows();

      int sizeOfActiveSet = activeInequalityIndices.size();

      CBar.reshape(sizeOfActiveSet, numberOfVariables);
      DBar.reshape(sizeOfActiveSet, 1);
      slackBar.reshape(sizeOfActiveSet, 1);

      for (int i = 0; i < sizeOfActiveSet; i++)
      {
         int inequalityConstraintIndex = activeInequalityIndices.get(i);
         CBar.insert(linearInequalityConstraintsCMatrixO, inequalityConstraintIndex, inequalityConstraintIndex + 1, 0, numberOfVariables, i, 0);
         DBar.insert(linearInequalityConstraintsDVectorO, inequalityConstraintIndex, inequalityConstraintIndex + 1, 0, 1, i, 0);
         slackBar.set(i, 0, linearInequalityConstraintsSlackVariableCost.get(inequalityConstraintIndex, 0));
      }

      //printSetChanges();
   }

   private void printSetChanges()
   {
      if (!inequalityIndicesToAddToActiveSet.isEmpty())
      {
         LogTools.info("Inequality constraint indices added : ");
         for (int i = 0; i < inequalityIndicesToAddToActiveSet.size(); i++)
            LogTools.info("" + inequalityIndicesToAddToActiveSet.get(i));
      }
      if (!inequalityIndicesToRemoveFromActiveSet.isEmpty())
      {
         LogTools.info("Inequality constraint indices removed : ");
         for (int i = 0; i < inequalityIndicesToRemoveFromActiveSet.size(); i++)
            LogTools.info("" + inequalityIndicesToRemoveFromActiveSet.get(i));
      }
   }

   private void solveEqualityConstrainedSubproblemEfficiently(NativeMatrix xSolutionToPack, NativeMatrix lagrangeEqualityConstraintMultipliersToPack,
                                                              NativeMatrix lagrangeInequalityConstraintMultipliersToPack)
   {
      int numberOfVariables = quadraticCostQMatrix.getNumRows();
      int numberOfOriginalEqualityConstraints = linearEqualityConstraintsAMatrix.getNumRows();

      int numberOfActiveInequalityConstraints = activeInequalityIndices.size();

      int numberOfAugmentedEqualityConstraints = numberOfOriginalEqualityConstraints + numberOfActiveInequalityConstraints;

      if (numberOfAugmentedEqualityConstraints == 0)
      {
         xSolutionToPack.mult(-1.0, QInverse, quadraticCostQVector);
         reportSolution(xSolutionToPack);
         return;
      }

      computeCBarTempMatrices();

      bigMatrixForLagrangeMultiplierSolution.reshape(numberOfAugmentedEqualityConstraints, numberOfAugmentedEqualityConstraints);
      bigVectorForLagrangeMultiplierSolution.reshape(numberOfAugmentedEqualityConstraints, 1);

      bigMatrixForLagrangeMultiplierSolution.insert(AQInverseATranspose, 0, 0);
      bigMatrixForLagrangeMultiplierSolution.insert(AQInverseCBarTranspose, 0, numberOfOriginalEqualityConstraints);

      bigMatrixForLagrangeMultiplierSolution.insert(CBarQInverseATranspose, numberOfOriginalEqualityConstraints, 0);
      bigMatrixForLagrangeMultiplierSolution.insert(CBarQInverseCBarTranspose, numberOfOriginalEqualityConstraints, numberOfOriginalEqualityConstraints);

      if (numberOfOriginalEqualityConstraints > 0)
      {
         bigVectorForLagrangeMultiplierSolution.insert(linearEqualityConstraintsBVector, 0, 0);
         bigVectorForLagrangeMultiplierSolution.multAddBlock(AQInverse, quadraticCostQVector, 0, 0);
      }

      if (numberOfActiveInequalityConstraints > 0)
      {
         bigVectorForLagrangeMultiplierSolution.insert(DBar, numberOfOriginalEqualityConstraints, 0);
         bigVectorForLagrangeMultiplierSolution.multAddBlock(CBarQInverse, quadraticCostQVector, numberOfOriginalEqualityConstraints, 0);
      }

      bigVectorForLagrangeMultiplierSolution.scale(-1.0, bigVectorForLagrangeMultiplierSolution);

      boolean wasInvertible = augmentedLagrangeMultipliers.solveCheck(bigMatrixForLagrangeMultiplierSolution, bigVectorForLagrangeMultiplierSolution);

      if (debug)
      {
         NativeMatrix enforcementCheck = new NativeMatrix(bigVectorForLagrangeMultiplierSolution);
         enforcementCheck.mult(bigMatrixForLagrangeMultiplierSolution, augmentedLagrangeMultipliers);
         for (int i = 0; i < bigMatrixForLagrangeMultiplierSolution.getNumRows(); i++)
         {
            if (!MathTools.epsilonEquals(enforcementCheck.get(i, 0), bigVectorForLagrangeMultiplierSolution.get(i, 0), 1e-5))
               LogTools.info("Crap." + wasInvertible);
         }
      }

      AAndC.reshape(numberOfAugmentedEqualityConstraints, numberOfVariables);
      AAndC.insert(linearEqualityConstraintsAMatrix, 0, 0);
      AAndC.insert(CBar, numberOfOriginalEqualityConstraints, 0);

      ATransposeMuAndCTransposeLambda.multTransA(AAndC, augmentedLagrangeMultipliers);

      tempVector.add(quadraticCostQVector, ATransposeMuAndCTransposeLambda);

      xSolutionToPack.mult(-1.0, QInverse, tempVector);
      reportSolution(xSolutionToPack);

      if (debug)
      {
         NativeMatrix enforcementCheck = new NativeMatrix(bigVectorForLagrangeMultiplierSolution);
         enforcementCheck.mult(bigMatrixForLagrangeMultiplierSolution, augmentedLagrangeMultipliers);
         boolean subspaceError = false;
         for (int i = 0; i < bigMatrixForLagrangeMultiplierSolution.getNumRows(); i++)
         {
            if (!MathTools.epsilonEquals(enforcementCheck.get(i, 0), bigVectorForLagrangeMultiplierSolution.get(i, 0), 1e-5))
               subspaceError = true;
         }

         boolean constraintError = false;
         enforcementCheck.mult(linearEqualityConstraintsAMatrix, xSolutionToPack);
         for (int i = 0; i < linearEqualityConstraintsBVector.getNumRows(); i++)
         {
            if (!MathTools.epsilonEquals(enforcementCheck.get(i, 0), linearEqualityConstraintsBVector.get(i, 0), 1e-3))
               constraintError = true;
         }
         if (subspaceError)
            LogTools.error("Crap. The subspace solution messed up." + wasInvertible);
         if (constraintError)
            LogTools.error("Crap. The equality constraints don't hold." + wasInvertible);

         for (int row = 0; row < linearInequalityConstraintsCMatrixO.getNumRows(); row++)
         {
            if (valuesInRow(linearInequalityConstraintsCMatrixO, row) > 4)
               LogTools.error("Wrong number of values in the row.");
         }

         for (int row = 0; row < linearInequalityConstraintsCMatrixO.getNumRows(); row++)
         {
            if (valuesInCol(linearInequalityConstraintsCMatrixO, row) > 4)
               LogTools.error("Wrong number of values in the column.");
         }
      }
      int startRow = 0;
      int numberOfRows = numberOfOriginalEqualityConstraints;
      lagrangeEqualityConstraintMultipliersToPack.insert(augmentedLagrangeMultipliers, startRow, startRow + numberOfRows, 0, 1, 0, 0);

      startRow += numberOfRows;
      lagrangeInequalityConstraintMultipliersToPack.zero();
      for (int i = 0; i < numberOfActiveInequalityConstraints; i++)
      {
         int inequalityConstraintIndex = activeInequalityIndices.get(i);
         lagrangeInequalityConstraintMultipliersToPack.insert(augmentedLagrangeMultipliers, startRow + i, startRow + i + 1, 0, 1, inequalityConstraintIndex, 0);
      }
   }

   private static int valuesInRow(NativeMatrix matrix, int row)
   {
      int counter = 0;
      for (int col = 0; col < matrix.getNumCols(); col++)
      {
         if (!MathTools.epsilonEquals(matrix.get(row, col), 0.0, 1e-4))
            counter++;
      }

      return counter;
   }

   private static int valuesInCol(NativeMatrix matrix, int col)
   {
      int counter = 0;
      for (int row = 0; row < matrix.getNumRows(); row++)
      {
         if (!MathTools.epsilonEquals(matrix.get(row, col), 0.0, 1e-4))
            counter++;
      }

      return counter;
   }


   private void reportSolution(NativeMatrix solution)
   {
      for (int i = 0; i < solutionListeners.size(); i++)
         solutionListeners.get(i).reportSolution(solution, activeInequalityIndices, null, null);
   }

   public void getLagrangeEqualityConstraintMultipliers(DMatrixRMaj multipliersMatrixToPack)
   {
      lagrangeEqualityConstraintMultipliers.get(multipliersMatrixToPack);
   }

   public void getLagrangeInequalityConstraintMultipliers(DMatrixRMaj multipliersMatrixToPack)
   {
      lagrangeInequalityConstraintMultipliers.get(multipliersMatrixToPack);
   }

   public void setInverseHessianCalculator(InverseMatrixCalculator<NativeMatrix> inverseSolver)
   {
      this.inverseSolver = inverseSolver;
   }

   private static class DefaultInverseMatrixCalculator implements InverseMatrixCalculator<NativeMatrix>
   {
      @Override
      public void computeInverse(NativeMatrix matrixToInvert, NativeMatrix inverseMatrixToPack)
      {
         inverseMatrixToPack.invert(matrixToInvert);
      }
   }
}
