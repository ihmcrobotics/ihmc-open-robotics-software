package us.ihmc.convexOptimization.quadraticProgram;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;

import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.decomposition.CholeskyDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

/**
 * Solves a Quadratic Program using an active set method known as the Goldfarb-Idani method.
 * This allows it to work where simple active set solvers may not work, finding a solution
 * even when some inequality constraints may violate one another.
 *
 * This algorithm is the same algorithm implemented in the QuadProg, QuadProg++, and uQuadProg++ algorithms.
 *
 * Uses the algorithm found in the paper
 * "A numerically stable dual method for solving strictly convex quadratic programs"
 * by D. Goldfarb and A. Idnani.
 *
 * @author Robert Griffin
 */
public class JavaQuadProgSolver implements SimpleActiveSetQPSolverInterface
{
   private enum QuadProgStep {step1, step2, step2a}

   private static final int TRUE = 1;
   private static final int FALSE = 0;

   private static final int defaultSize = 100;
   private static final double epsilon = 0.00001;

   private final DenseMatrix64F R = new DenseMatrix64F(defaultSize, defaultSize);

   private final DenseMatrix64F inequalityConstraintViolations = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F stepDirectionInPrimalSpace = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F stepDirectionInDualSpace = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F d = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F np = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F lagrangeMultipliers = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F previousLagrangeMultipliers = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F previousSolution = new DenseMatrix64F(defaultSize);

   private final TIntArrayList activeSetIndices = new TIntArrayList(defaultSize);
   private final TIntArrayList previousActiveSetIndices = new TIntArrayList(defaultSize);
   private final TIntArrayList inactiveSetIndices = new TIntArrayList(defaultSize);
   private final TIntArrayList excludeConstraintFromActiveSet = new TIntArrayList(defaultSize); // booleans

   private final DenseMatrix64F J = new DenseMatrix64F(defaultSize, defaultSize);

   private final CholeskyDecomposition<DenseMatrix64F> decomposer = DecompositionFactory.chol(defaultSize, false);
   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(defaultSize);

   private final DenseMatrix64F quadraticCostQMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F quadraticCostQVector = new DenseMatrix64F(defaultSize);
   private double quadraticCostScalar;

   private final DenseMatrix64F linearEqualityConstraintsAMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F linearEqualityConstraintsBVector = new DenseMatrix64F(defaultSize);

   private final DenseMatrix64F linearInequalityConstraintsCMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F linearInequalityConstraintsDVector = new DenseMatrix64F(defaultSize);

   private int problemSize;
   private int numberOfInequalityConstraints;
   private int numberOfEqualityConstraints;

   private int numberOfActiveConstraints;
   private double R_norm;
   private int constraintIndexForMinimumStepLength;

   private int maxNumberOfIterations = 10;

   /**
    * The internal problem is of the form:
    * min 0.5 * x G x + g0 x
    * s.t.
    *     CE^T x + ce0 = 0
    *     CI^T x + ci0 >= 0
    *
    *
    * To interface with it, assume the standard form:
    * min 0.5 * x G x + g0 x
    * s.t.
    *     CE^T x = ce0
    *     CI^T x >= ci0
    */



   //// // TODO: 5/15/17 get rid of this 

   private void reshape()
   {
      int numberOfConstraints = numberOfEqualityConstraints + numberOfInequalityConstraints;

      R.reshape(problemSize, problemSize);
      inequalityConstraintViolations.reshape(numberOfConstraints, 1);
      stepDirectionInPrimalSpace.reshape(problemSize, 1);
      stepDirectionInDualSpace.reshape(numberOfConstraints, 1);
      d.reshape(problemSize, 1);
      np.reshape(problemSize, 1);
      lagrangeMultipliers.reshape(numberOfConstraints, 1);
      previousSolution.reshape(problemSize, 1);
      previousLagrangeMultipliers.reshape(numberOfConstraints, 1);

      activeSetIndices.fill(0, numberOfConstraints, 0);
      previousActiveSetIndices.fill(0, numberOfConstraints, 0);
      inactiveSetIndices.fill(0, numberOfConstraints, 0);
      excludeConstraintFromActiveSet.fill(0, numberOfConstraints, FALSE);
   }

   // // TODO: 5/15/17  get rid of this 
   private void zero()
   {
      R.zero();
      inequalityConstraintViolations.zero();
      stepDirectionInPrimalSpace.zero();
      stepDirectionInDualSpace.zero();
      d.zero();
      np.zero();
      lagrangeMultipliers.zero();
      previousSolution.zero();
      previousLagrangeMultipliers.zero();
   }

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

      linearInequalityConstraintsCMatrix.reshape(0, 0);
      linearInequalityConstraintsDVector.reshape(0, 0);
   }

   @Override
   public void setVariableBounds(DenseMatrix64F variableLowerBounds, DenseMatrix64F variableUpperBounds)
   {
      throw new RuntimeException("This has yet to be implemented!");
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

      problemSize = quadraticCostQMatrix.getNumCols();
   }

   private final DenseMatrix64F computedObjectiveFunctionValue = new DenseMatrix64F(1, 1);
   @Override
   public double getObjectiveCost(DenseMatrix64F x)
   {
      multQuad(x, quadraticCostQMatrix, computedObjectiveFunctionValue);
      CommonOps.scale(0.5, computedObjectiveFunctionValue);
      CommonOps.multAddTransA(quadraticCostQVector, x, computedObjectiveFunctionValue);
      return computedObjectiveFunctionValue.get(0, 0) + quadraticCostScalar;
   }

   private final DenseMatrix64F temporaryMatrix = new DenseMatrix64F(0, 0);

   private void multQuad(DenseMatrix64F xVector, DenseMatrix64F QUpperMatrix, DenseMatrix64F xTransposeQx)
   {
      temporaryMatrix.reshape(xVector.numCols, QUpperMatrix.numCols);
      tempMatrix.reshape(xVector.numCols, QUpperMatrix.numCols);
      CommonOps.multTransA(xVector, QUpperMatrix, tempMatrix);
      CommonOps.multTransB(tempMatrix, QUpperMatrix, temporaryMatrix);
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

      this.linearEqualityConstraintsAMatrix.reshape(linearEqualityConstraintsAMatrix.getNumCols(), linearEqualityConstraintsAMatrix.getNumRows());
      CommonOps.transpose(linearEqualityConstraintsAMatrix, this.linearEqualityConstraintsAMatrix);
      CommonOps.scale(-1.0, this.linearEqualityConstraintsAMatrix);

      this.linearEqualityConstraintsBVector.set(linearEqualityConstraintsBVector);
      numberOfEqualityConstraints = linearEqualityConstraintsBVector.getNumRows();
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

      linearInequalityConstraintsCMatrix.reshape(linearInequalityConstraintCMatrix.getNumCols(), linearInequalityConstraintCMatrix.getNumRows());
      CommonOps.transpose(linearInequalityConstraintCMatrix, linearInequalityConstraintsCMatrix);
      CommonOps.scale(-1.0, linearInequalityConstraintsCMatrix);

      this.linearInequalityConstraintsDVector.set(linearInequalityConstraintDVector);
      numberOfInequalityConstraints = linearInequalityConstraintDVector.getNumRows();
   }

   @Override
   public int solve(double[] solutionToPack)
   {
      int numberOfEqualityConstraints = linearEqualityConstraintsBVector.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsDVector.getNumRows();

      double[] lagrangeEqualityConstraintMultipliersToPack = new double[numberOfEqualityConstraints];
      double[] lagrangeInequalityConstraintMultipliersToPack = new double[numberOfInequalityConstraints];

      return solve(solutionToPack, lagrangeEqualityConstraintMultipliersToPack, lagrangeInequalityConstraintMultipliersToPack);
   }

   @Override
   public int solve(double[] solutionToPack, double[] lagrangeEqualityConstraintMultipliersToPack, double[] lagrangeInequalityConstraintMultipliersToPack)
   {
      //// TODO: 5/15/17  bounds
      //int numberOfLowerBoundConstraints = variableLowerBounds.getNumRows();
      //int numberOfUpperBoundConstraints = variableUpperBounds.getNumRows();
      int numberOfLowerBoundConstraints = 0;
      int numberOfUpperBoundConstraints = 0;

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
      int numberOfEqualityConstraints = linearEqualityConstraintsBVector.getNumRows();
      int numberOfInequalityConstraints = linearInequalityConstraintsDVector.getNumRows();
      //// TODO: 5/15/17  bounds
      //int numberOfLowerBoundConstraints = variableLowerBounds.getNumRows();
      //int numberOfUpperBoundConstraints = variableUpperBounds.getNumRows();
      int numberOfLowerBoundConstraints = 0;
      int numberOfUpperBoundConstraints = 0;

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
      throw new RuntimeException("Not yet implemented");
   }

   @Override
   public void resetActiveConstraints()
   {
      throw new RuntimeException("Not yet implemented");
   }

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(defaultSize, defaultSize);



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

   // The solving function, implementing the Goldfarb-Idani method
   @Override
   public int solve(DenseMatrix64F solutionToPack, DenseMatrix64F lagrangeEqualityConstraintMultipliersToPack,
         DenseMatrix64F lagrangeInequalityConstraintMultipliersToPack, DenseMatrix64F lagrangeLowerBoundConstraintMultipliersToPack,
         DenseMatrix64F lagrangeUpperBoundConstraintMultipliersToPack)
   {
      solutionToPack.reshape(problemSize, 1);
      solutionToPack.zero();

      reshape();
      zero();

      QuadProgStep currentStep = QuadProgStep.step1;

      double c1, c2;
      int solutionPairConstraintIndex = 0; // this is the index of the constraint to be added to the active set
      int numberOfIterations = 0;

      J.reshape(problemSize, problemSize);
      tempMatrix.reshape(problemSize, 1);

      /** Preprocessing phase */

      // compute the trace of the original matrix quadraticCostQMatrix
      c1 = CommonOps.trace(quadraticCostQMatrix);

      // decompose the matrix quadraticCostQMatrix in the form L^T L
      decomposer.decompose(quadraticCostQMatrix);

      R_norm = 1.0; // this variable will hold the norm of the matrix R

      // compute the inverse of the factorized matrix G^-1, this is the initial value for H //// TODO: 5/14/17 combine this with the decomposition 
      solver.setA(quadraticCostQMatrix);
      solver.invert(J);
      c2 = CommonOps.trace(J);

      // c1 * c2 is an estimate for cond(G)

      // Find the unconstrained minimizer of the quadratic form 0.5 * x G x + g0 x
      // this is the feasible point in the dual space.
      // x = -G^-1 * g0 = -J * J^T * g0
      CommonOps.multTransA(J, quadraticCostQVector, tempMatrix);
      CommonOps.mult(-1.0, J, tempMatrix, solutionToPack);

      // There aren't any constraints, so this is a valid solution
      if (numberOfInequalityConstraints + numberOfEqualityConstraints == 0)
      {
         return numberOfIterations;
      }

      // Add equality constraints to the working set A
      numberOfActiveConstraints = 0;
      double minimumStepInPrimalSpace;
      for (int i = 0; i < numberOfEqualityConstraints; i++)
      {
         MatrixTools.setMatrixBlock(np, 0, 0, linearEqualityConstraintsAMatrix, 0, i, problemSize, 1, 1.0); // np is the ith constraint value

         compute_d();
         updateStepDirectionInPrimalSpace();
         updateStepDirectionInDualSpace();

         // compute full step length minimumStepInPrimalSpace: i.e., the minimum step in primal space s.t. the constraint becomes feasible
         minimumStepInPrimalSpace = 0.0;
         if (Math.abs(CommonOps.dot(stepDirectionInPrimalSpace, stepDirectionInPrimalSpace)) > epsilon) // i.e. z != 0
         {
            minimumStepInPrimalSpace = (-CommonOps.dot(np, solutionToPack) - linearEqualityConstraintsBVector.get(i)) / CommonOps.dot(stepDirectionInPrimalSpace, np);
         }

         // set x = x + minimumStepInPrimalSpace * stepDirectionInPrimalSpace
         CommonOps.addEquals(solutionToPack, minimumStepInPrimalSpace, stepDirectionInPrimalSpace);

         // set u = u+
         lagrangeMultipliers.set(numberOfActiveConstraints, minimumStepInPrimalSpace);
         MatrixTools.addMatrixBlock(lagrangeMultipliers, 0, 0, stepDirectionInDualSpace, 0, 0, numberOfActiveConstraints, 1, minimumStepInPrimalSpace);

         activeSetIndices.set(i, -i - 1);

         if (!addConstraint())
         {
            throw new RuntimeException("Constraints are linearly dependent.");
         }
      }
      
      // // TODO: 5/15/17 exit if the current active set is valid 

      // set iai = K \ A, i.e., the whole set is inactive
      for (int i = 0; i < numberOfInequalityConstraints; i++)
         inactiveSetIndices.set(i, i);

      double solutionPairConstraintViolation = 0.0;
      constraintIndexForMinimumStepLength = 0;

      while (numberOfIterations < maxNumberOfIterations)
      {
         switch(currentStep)
         {
         case step1:
            numberOfIterations++;

            // step 1: choose a violated constraint
            for (int i = numberOfEqualityConstraints; i < numberOfActiveConstraints; i++)
            {
               solutionPairConstraintIndex = activeSetIndices.get(i);
               inactiveSetIndices.set(solutionPairConstraintIndex, -1);
            }

            // compute s(x) = ci^T * x + ci0 for all elements of K \ A
            solutionPairConstraintViolation = 0.0;
            double psi = 0.0; // this value will contain the sum of all infeasibilities
            solutionPairConstraintIndex = 0;
            for (int i = 0; i < numberOfInequalityConstraints; i++)
            {
               excludeConstraintFromActiveSet.set(i, TRUE);
               double sum = 0.0;
               for (int j = 0; j < problemSize; j++)
                  sum += linearInequalityConstraintsCMatrix.get(j, i) * solutionToPack.get(j);
               sum += linearInequalityConstraintsDVector.get(i);
               inequalityConstraintViolations.set(i, sum);
               psi += Math.min(0.0, sum);
            }

            if (Math.abs(psi) < numberOfInequalityConstraints * epsilon * c1 * c2 * 100.0)
            {
               // numerically there are not infeasibilities anymore
               return numberOfIterations;
            }

            // save old values for u, x, and A
            MatrixTools.setMatrixBlock(previousLagrangeMultipliers, 0, 0, lagrangeMultipliers, 0, 0, numberOfActiveConstraints, 1, 1.0);
            previousSolution.set(solutionToPack);
            for (int i = 0; i < numberOfActiveConstraints; i++)
               previousActiveSetIndices.set(i, activeSetIndices.get(i));

         case step2:
            // Step 2: check for feasibility and determine a new S-pair
            for (int i = 0; i < numberOfInequalityConstraints; i++)
            {
               // select the constraint from the inactive set that is most violated
               if (inequalityConstraintViolations.get(i) < solutionPairConstraintViolation && inactiveSetIndices.get(i) != -1
                     && excludeConstraintFromActiveSet.get(i) == TRUE)
               {
                  solutionPairConstraintViolation = inequalityConstraintViolations.get(i);
                  solutionPairConstraintIndex = i;
               }
            }
            if (solutionPairConstraintViolation >= 0.0)
            { // Solution is valid
               return numberOfIterations;
            }

            // set np = n(solutionPairConstraintIndex)
            MatrixTools.setMatrixBlock(np, 0, 0, linearEqualityConstraintsAMatrix, 0, solutionPairConstraintIndex, problemSize, 1, 1.0); // np is the solutionPairConstraint
            // set u = [u 0]^T
            lagrangeMultipliers.set(numberOfActiveConstraints, 0.0);
            // add the violated constraint to the active set A
            activeSetIndices.set(numberOfActiveConstraints, solutionPairConstraintIndex);

         case step2a:
            // Step 2a: determine step direction
            compute_d();
            // compute z = H np: the step direction in the primal space (through J, see the paper)
            updateStepDirectionInPrimalSpace();
            // compute N* np (if activeSetSize > 0): the negative of the step direction in the dual space
            updateStepDirectionInDualSpace();

            break;
         default:
            throw new RuntimeException("This is an empty state.");
         }


         // Step 2b: compute step length
         constraintIndexForMinimumStepLength = 0;
         // Compute partial step length (maximum step in dual space without violating dual feasibility
         double maximumStepInDualSpace = Double.POSITIVE_INFINITY;
         // find the constraintIndexForMinimumStepLength s.t. it reaches the minimum of u+(x) / r
         for (int k = numberOfEqualityConstraints; k < numberOfActiveConstraints; k++)
         {
            double minimumStepLength = -lagrangeMultipliers.get(k) / stepDirectionInDualSpace.get(k);
            if (stepDirectionInDualSpace.get(k) < 0.0 && minimumStepLength < maximumStepInDualSpace)
            {
               maximumStepInDualSpace = minimumStepLength;
               constraintIndexForMinimumStepLength = activeSetIndices.get(k);
            }
         }

         // Compute full step length (minimum step in primal space such that the violated constraint becomes feasible
         if (Math.abs(CommonOps.dot(stepDirectionInPrimalSpace, stepDirectionInPrimalSpace)) > epsilon)
         {
            minimumStepInPrimalSpace = -inequalityConstraintViolations.get(solutionPairConstraintIndex) / CommonOps.dot(stepDirectionInPrimalSpace, np);
            if (minimumStepInPrimalSpace < 0.0) // patch suggested by Takano Akio for handling numerical inconsistencies
               minimumStepInPrimalSpace = Double.POSITIVE_INFINITY;
         }
         else
         {
            minimumStepInPrimalSpace = Double.POSITIVE_INFINITY;
         }

         // the step is chosen as the minimum of maximumStepInDualSpace and minimumStepInPrimalSpace
         double minimumStep = Math.min(maximumStepInDualSpace, minimumStepInPrimalSpace); // step length, minimum of partial step (maximumStepInDualSpace) and full step (minimumStepInPrimalSpace);

         // Step 2c: determine new S-pair and take step:

         if (!Double.isFinite(minimumStep))
         { // case (i): no step in primal or dual space, QPP is infeasible
            return numberOfIterations;
         }
         else if (!Double.isFinite(minimumStepInPrimalSpace))
         { // case (ii): step in dual space
            // set u = u + t * [r 1] and drop constraintIndexForMinimumStepLength from the active set
            MatrixTools.addMatrixBlock(lagrangeMultipliers, 0, 0, stepDirectionInDualSpace, 0, 0, numberOfActiveConstraints, 1, minimumStep);
            lagrangeMultipliers.set(numberOfActiveConstraints, lagrangeMultipliers.get(numberOfActiveConstraints) + minimumStep);
            inactiveSetIndices.set(constraintIndexForMinimumStepLength, constraintIndexForMinimumStepLength);
            deleteConstraint(J, activeSetIndices, lagrangeMultipliers);

            currentStep = QuadProgStep.step2a;
         }
         else
         { // case (iii): step in primal and dual space.
            MatrixTools.addMatrixBlock(solutionToPack, 0, 0, stepDirectionInPrimalSpace, 0, 0, problemSize, 1, minimumStep);

            // u = u + t * [r 1]
            MatrixTools.addMatrixBlock(lagrangeMultipliers, 0, 0, stepDirectionInDualSpace, 0, 0, numberOfActiveConstraints, 1, minimumStep);
            lagrangeMultipliers.set(numberOfActiveConstraints, lagrangeMultipliers.get(numberOfActiveConstraints) + minimumStep);


            if (MathTools.epsilonEquals(minimumStep, minimumStepInPrimalSpace, epsilon))
            { // full step has been taken, using the minimumStepInPrimalSpace
               // add the violated constraint to the active set
               if (!addConstraint())
               { // fail out because adding the constraint causes it to degenerate
                  excludeConstraintFromActiveSet.set(solutionPairConstraintIndex, FALSE);
                  deleteConstraint(J, activeSetIndices, lagrangeMultipliers);

                  for (int i = 0; i < numberOfInequalityConstraints; i++)
                     inactiveSetIndices.set(i, i);

                  for (int i = 0; i < numberOfActiveConstraints; i++)
                  {
                     activeSetIndices.set(i, previousActiveSetIndices.get(i));
                     inactiveSetIndices.set(activeSetIndices.get(i), -1);
                  }
                  MatrixTools.setMatrixBlock(lagrangeMultipliers, 0, 0, previousLagrangeMultipliers, 0, 0, numberOfActiveConstraints, 1, 1.0);

                  solutionToPack.set(previousSolution);

                  currentStep = QuadProgStep.step2;
               }
               else
               {
                  inactiveSetIndices.set(solutionPairConstraintIndex, -1);
                  // we need to go all the way back to the beginning, as this constraint has been saturated.
                  currentStep = QuadProgStep.step1;
               }
            }
            else
            { // a partial step has been taken
               // drop constraintIndexForMinimumStepLength
               inactiveSetIndices.set(constraintIndexForMinimumStepLength, constraintIndexForMinimumStepLength);
               deleteConstraint(J, activeSetIndices, lagrangeMultipliers);

               // update s[ip] = CI * x + ci0
               double sum = 0.0;
               for (int k = 0; k < problemSize; k++)
                  sum += linearInequalityConstraintsCMatrix.get(k, solutionPairConstraintIndex) * solutionToPack.get(k);
               inequalityConstraintViolations.set(solutionPairConstraintIndex, sum + linearInequalityConstraintsDVector.get(solutionPairConstraintIndex));

               currentStep = QuadProgStep.step2a;
            }
         }
      }

      return numberOfIterations;
   }

   private void compute_d()
   {
      // compute d = H^T * np
      CommonOps.multTransA(J, np, d);
   }

   // compute z = H np: the step direction in the primal space (through J, see the paper)
   private void updateStepDirectionInPrimalSpace()
   {
      // // TODO: 5/15/17 make block operations
      // setting of z = J * d
      for (int i = 0; i < problemSize; i++)
      {
         double sum = 0.0;
         for (int j = numberOfActiveConstraints; j < problemSize; j++)
            sum += J.get(i, j) * d.get(j);

         stepDirectionInPrimalSpace.set(i, sum);
      }
   }

   // compute -N* np (if activeSetSize > 0): the step direction in the dual space
   private void updateStepDirectionInDualSpace()
   {
      // // TODO: 5/15/17  make block operations
      // setting of r = -R^-1 * d
      for (int i = numberOfActiveConstraints - 1; i >= 0; i--)
      {
         double sum = 0.0;
         for (int j = i + 1; j < numberOfActiveConstraints; j++)
            sum += R.get(i, j) * stepDirectionInDualSpace.get(j);

         stepDirectionInDualSpace.set(i, (sum - d.get(i)) / R.get(i, i));
      }
   }

   private boolean addConstraint()
   {
      // we have to find the Givens rotation which will reduce the element d(j) to zero.
      // if it is already zero, we don't have to do anything, except decrease j
      for (int j = problemSize - 1; j >= numberOfActiveConstraints + 1; j--)
      {
         /* The Givens rotation is done with the matrix (cc cs, cs -cc). If cc is one, then element (j) of d is zero compared with
          element (j - 1). Hence we don't have to do anything.
          If cc is zero, then we just have to switch column (j) and column (j - 1) of J. Since we only switch columns in J, we have
          to be careful how we update d depending on the sign of gs.
          Otherwise we have to apply the Givens rotation to these columns.
          The i - 1 element of d has to be updated to h. */
         double cc = d.get(j - 1);
         double ss = d.get(j);
         double h = distance(cc, ss);
         if (Math.abs(h) < epsilon) // h == 0
            continue;
         d.set(j, 0.0);
         ss = ss / h;
         cc = cc / h;
         if (cc < 0.0)
         {
            cc = -cc;
            ss = -ss;
            d.set(j - 1, -h);
         }
         else
         {
            d.set(j - 1, h);
         }

         double xny = ss / (1.0 + cc);
         for (int k = 0; k < problemSize; k++)
         {
            double t1 = J.get(k, j - 1);
            double t2 = J.get(k, j);
            J.set(k, j - 1, t1 * cc + t2 * ss);
            J.set(k, j, xny * (t1 + J.get(k, j - 1)) - t2);
         }
      }

      // update the number of constraints added
      numberOfActiveConstraints++;

      // To update R we have to put the numberOfActiveConstraints components of the d vector into column numberOfActiveConstraints - 1 of R
      for (int i = 0; i < numberOfActiveConstraints; i++)
         R.set(i, numberOfActiveConstraints - 1, d.get(i));

      if (Math.abs(d.get(numberOfActiveConstraints - 1)) < epsilon * R_norm)
      {
         // problem degenerate
         return false;
      }

      R_norm = Math.max(R_norm, Math.abs(d.get(numberOfActiveConstraints - 1)));
      return true;
   }

   private void deleteConstraint(DenseMatrix64F J, TIntArrayList AToPack, DenseMatrix64F lagrangeMultipliersToPack)
   {
      double cc, ss, h, xny, t1, t2;
      int qq = -1;

      // Find the index qq for active constraintIndexForMinimumStepLength to be removed
      for (int i = numberOfEqualityConstraints; i < numberOfActiveConstraints; i++)
      {
         if (AToPack.get(i) == constraintIndexForMinimumStepLength)
         {
            qq = i;
            break;
         }
      }

      // remove the constraint from the active set and the duals
      for (int i = qq; i < numberOfActiveConstraints - 1; i++)
      {
         AToPack.set(i, AToPack.get(i + 1));
         lagrangeMultipliersToPack.set(i, lagrangeMultipliersToPack.get(i + 1));

         for (int j = 0; j < problemSize; j++)
            R.set(j, i, R.get(j, i + 1));
      }

      AToPack.set(numberOfActiveConstraints - 1, AToPack.get(numberOfActiveConstraints));
      lagrangeMultipliersToPack.set(numberOfActiveConstraints - 1, lagrangeMultipliersToPack.get(numberOfActiveConstraints));
      AToPack.set(numberOfActiveConstraints, 0);
      lagrangeMultipliersToPack.set(numberOfActiveConstraints, 0.0);
      for (int j = 0; j < numberOfActiveConstraints; j++)
         R.set(j, numberOfActiveConstraints - 1, 0.0);

      // constraint has been fully removed
      numberOfActiveConstraints--;

      if (numberOfActiveConstraints == 0)
         return;

      for (int j = qq; j < numberOfActiveConstraints; j++)
      {
         cc = R.get(j, j);
         ss = R.get(j + 1, j);
         h = distance(cc, ss);

         if (Math.abs(h) < epsilon) // h == 0
            continue;

         cc = cc / h;
         ss = ss / h;
         R.set(j + 1, j, 0.0);

         if (cc < 0.0)
         {
            R.set(j, j, -h);
            cc = -cc;
            ss = -ss;
         }
         else
         {
            R.set(j, j, h);
         }

         xny = ss / (1.0 + cc);
         for (int k = j + 1; k < numberOfActiveConstraints; k++)
         {
            t1 = R.get(j, k);
            t2 = R.get(j + 1, k);
            R.set(j, k, t1 * cc + t2 * ss);
            R.set(j + 1, k, xny * (t1 + R.get(j, k)) - t2);
         }

         for (int k = 0; k < problemSize; k++)
         {
            t1 = J.get(k, j);
            t2 = J.get(k, j + 1);
            J.set(k, j, t1 * cc + t2 * ss);
            J.set(k, j + 1, xny * (J.get(k, j) + t1) - t2);
         }
      }
   }


   /**
    * Computes the Euclidean distance between two numbers
    * @param a first number
    * @param b second number
    * @return Euclidean distance
    */
   private static double distance(double a, double b)
   {
      double a1 = Math.abs(a);
      double b1 = Math.abs(b);
      if (a1 > b1)
      {
         double t = b1 / a1;
         return a1 * Math.sqrt(1.0 + t * t);
      }
      else if (b1 > a1)
      {
         double t = a1 / b1;
         return b1 * Math.sqrt(1.0 + t * t);
      }

      return a1 * Math.sqrt(2.0);
   }
}
