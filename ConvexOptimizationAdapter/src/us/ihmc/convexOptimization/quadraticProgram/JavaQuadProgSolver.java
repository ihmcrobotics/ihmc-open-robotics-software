package us.ihmc.convexOptimization.quadraticProgram;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;

import org.ejml.factory.DecompositionFactory;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.decomposition.CholeskyDecomposition;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.PrintTools;

public class JavaQuadProgSolver
{
   private enum QuadProgStep {step1, step2, step2a, step2b, step2c, step3};

   private final DenseMatrix64F negAin = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F negAeq = new DenseMatrix64F(defaultSize);

   private QuadProgStep currentStep;

   private static final boolean traceSolver = false;
   private static final int TRUE = 1;
   private static final int FALSE = 0;

   private static final int defaultSize = 100;
   private static final double epsilon = 0.00001;

   private final DenseMatrix64F R = new DenseMatrix64F(defaultSize, defaultSize);

   private final DenseMatrix64F inequalityConstraintViolations = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F z = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F r = new DenseMatrix64F(defaultSize);
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

   private int numberOfIterations;

   /**
    * The problem is of the form:
    * min 0.5 * x G x + g0 x
    * s.t.
    *     CE^T x + ce0 = 0
    *     CI^T x + ci0 >= 0
    */



   private void compute_d(DenseMatrix64F dToPack, DenseMatrix64F J, DenseMatrix64F np)
   {
      // compute d = H^T * np
      CommonOps.multTransA(J, np, dToPack);
   }

   private void update_z(DenseMatrix64F zToPack, DenseMatrix64F J, DenseMatrix64F d)
   {
      // setting of z = J * d
      for (int i = 0; i < problemSize; i++)
      {
         double sum = 0.0;
         for (int j = numberOfActiveConstraints; j < problemSize; j++)
            sum += J.get(i, j) * d.get(j);

         zToPack.set(i, sum);
      }
   }

   private void update_r(DenseMatrix64F rToPack, DenseMatrix64F R, DenseMatrix64F d)
   {
      // setting of r = R^-1 * d
      for (int i = numberOfActiveConstraints - 1; i >= 0; i--)
      {
         double sum = 0.0;
         for (int j = i + 1; j < numberOfActiveConstraints; j++)
            sum += R.get(i, j) * rToPack.get(j);

         rToPack.set(i, (d.get(i) - sum) / R.get(i, i));
      }
   }

   private boolean addConstraint(DenseMatrix64F J)
   {
      PrintTools.debug(traceSolver, "Add constraint " + numberOfActiveConstraints);

      double cc, ss, h, t1, t2, xny;

      // we have to find the Givens rotation which will reduce the element d(j) to zero.
      // if it is already zero, we don't have to do anything, except of decreasing j
      for (int j = problemSize - 1; j >= numberOfActiveConstraints + 1; j--)
      {
         /* The Givens rotation is done with the matrix (cc cs, cs -cc). If cc is one, then element (j) of d is zero compared with
          element (j - 1). Hence we don't have to do anything.
          If cc is zero, then we just have to switch column (j) and column (j - 1) of J. Since we only switch columns in J, we have
          to be careful how we update d depending on the sign of gs.
          Otherwise we have to apply the Givens rotation to these columns.
          The i - 1 element of d has to be updated to h. */
         cc = d.get(j - 1);
         ss = d.get(j);
         h = distance(cc, ss);
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

         xny = ss / (1.0 + cc);
         for (int k = 0; k < problemSize; k++)
         {
            t1 = J.get(k, j - 1);
            t2 = J.get(k, j);
            J.set(k, j - 1, t1 * cc + t2 * ss);
            J.set(k, j, xny * (t1 + J.get(k, j - 1)) - t2);
         }
      }

      // update the number of constraints added
      numberOfActiveConstraints++;

      // To update R we have to put the numberOfActiveConstraints components of the d vector into column numberOfActiveConstraints - 1 of R
      for (int i = 0; i < numberOfActiveConstraints; i++)
         R.set(i, numberOfActiveConstraints - 1, d.get(i));

      PrintTools.debug(traceSolver, "" + numberOfActiveConstraints);
      PrintTools.debug(traceSolver, "R : " + R);
      PrintTools.debug(traceSolver, "J : " + J);
      PrintTools.debug(traceSolver, "d : " + d);

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
      PrintTools.debug(traceSolver, "Delete constraint " + constraintIndexForMinimumStepLength + " " + numberOfActiveConstraints);

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
      PrintTools.debug(traceSolver, "/ " + numberOfActiveConstraints);

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


   public void reshape()
   {
      int numberOfConstraints = numberOfEqualityConstraints + numberOfInequalityConstraints;

      R.reshape(problemSize, problemSize);
      inequalityConstraintViolations.reshape(numberOfConstraints, 1);
      z.reshape(problemSize, 1);
      r.reshape(numberOfConstraints, 1);
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

   private void zero()
   {
      R.zero();
      inequalityConstraintViolations.zero();
      z.zero();
      r.zero();
      d.zero();
      np.zero();
      lagrangeMultipliers.zero();
      previousSolution.zero();
      previousLagrangeMultipliers.zero();
   }

   private void allocateTempraryMatrixOnDemand(int nvar, int neq, int nin)
   {
      negAin.reshape(nvar, nin);
      negAeq.reshape(nvar, neq);
   }

   public double solve(DenseMatrix64F Q, DenseMatrix64F f, DenseMatrix64F Aeq, DenseMatrix64F beq, DenseMatrix64F Ain,
         DenseMatrix64F bin, DenseMatrix64F x, boolean initialize)
   {
      allocateTempraryMatrixOnDemand(Q.numCols, Aeq.numRows, Ain.numRows);

      CommonOps.transpose(Aeq, this.negAeq);
      CommonOps.scale(-1, this.negAeq);
      CommonOps.transpose(Ain, this.negAin);
      CommonOps.scale(-1, this.negAin);

      return solveQuadprog(Q, f, negAeq, beq, negAin, bin, x, initialize);
   }


   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   // The solving function, implementing the Goldfarb-Idani method
   public double solveQuadprog(DenseMatrix64F G, DenseMatrix64F g0,
         DenseMatrix64F CE, DenseMatrix64F ce0, DenseMatrix64F CI, DenseMatrix64F ci0, DenseMatrix64F solutionToPack, boolean initialize)
   {
      //// TODO: 5/13/17  initialize
      problemSize = G.getNumCols();
      numberOfEqualityConstraints = CE.getNumCols();
      numberOfInequalityConstraints = CI.getNumCols();

      if (g0.getNumCols() != 1)
         throw new RuntimeException("g0.getNumCols() != 1");
      if (G.getNumRows() != g0.getNumRows())
         throw new RuntimeException("G.getNumRows() != g0.getNumRows()");
      if (G.getNumRows() != problemSize)
         throw new RuntimeException("G.getNumRows() != G.getNumCols()");
      if (CE.getNumRows() != problemSize)
         throw new RuntimeException("Equality constraint matrix is incompatible, wrong number of variables.");
      if (ce0.getNumRows() != numberOfEqualityConstraints)
         throw new RuntimeException("Equality constraint objective is incompatible, wrong number of constraints.");
      if (CI.getNumRows() != problemSize)
         throw new RuntimeException("Inequality constraint matrix is incompatible, wrong number of variables.");
      if (ci0.getNumRows() != numberOfInequalityConstraints)
         throw new RuntimeException("IneEquality constraint objective is incompatible, wrong number of constraints.");

      quadraticCostQMatrix.reshape(problemSize, problemSize);
      quadraticCostQVector.reshape(problemSize, 1);
      quadraticCostQMatrix.set(G);
      quadraticCostQVector.set(g0);

      linearEqualityConstraintsAMatrix.reshape(problemSize, numberOfEqualityConstraints);
      linearEqualityConstraintsBVector.reshape(numberOfEqualityConstraints, 1);
      linearEqualityConstraintsAMatrix.set(CE);
      linearEqualityConstraintsBVector.set(ce0);

      linearInequalityConstraintsCMatrix.reshape(problemSize, numberOfInequalityConstraints);
      linearInequalityConstraintsDVector.reshape(numberOfInequalityConstraints, 1);
      linearInequalityConstraintsCMatrix.set(CI);
      linearInequalityConstraintsDVector.set(ci0);


      solutionToPack.reshape(problemSize, 1);
      solutionToPack.zero();

      reshape();
      zero();

      currentStep = QuadProgStep.step1;

      double solutionValue, c1, c2;
      double t = 0.0, t1 = 0.0, t2 = 0.0; // t is the step length, which is the minimum of the partial step t1 and the full step length t2
      int solutionPairConstraintIndex = 0; // this is the index of the constraint to be added to the active set
      numberOfIterations = 0;

      PrintTools.debug(traceSolver, "G : " + quadraticCostQMatrix);
      PrintTools.debug(traceSolver, "g0 : " + quadraticCostQVector);
      PrintTools.debug(traceSolver, "CE : " + linearEqualityConstraintsAMatrix);
      PrintTools.debug(traceSolver, "ce0 : " + linearEqualityConstraintsBVector);
      PrintTools.debug(traceSolver, "CI : " + linearInequalityConstraintsCMatrix);
      PrintTools.debug(traceSolver, "ci0 : " + linearInequalityConstraintsDVector);

      J.reshape(problemSize, problemSize);
      tempMatrix.reshape(problemSize, 1);

      /** Preprocessing phase */

      // compute the trace of the original matrix quadraticCostQMatrix
      c1 = CommonOps.trace(quadraticCostQMatrix);


      // decompose the matrix quadraticCostQMatrix in the form L^T L
      decomposer.decompose(quadraticCostQMatrix);
      PrintTools.debug(traceSolver, "G : " + quadraticCostQMatrix);

      R_norm = 1.0; // this variable will hold the norm of the matrix R

      // compute the inverse of the factorized matrix G^-1, this is the initial value for H //// TODO: 5/14/17 combine this with the decomposition 
      solver.setA(quadraticCostQMatrix);
      solver.invert(J);
      c2 = CommonOps.trace(J);
      PrintTools.debug(traceSolver, "J : " + J);

      // c1 * c2 is an estimate for cond(G)

      // Find the unconstrained minimizer of the quadratic form 0.5 * x G x + g0 x
      // this is the feasible point in the dual space.
      // x = -G^-1 * g0 = -J * J^T * g0
      CommonOps.multTransA(J, quadraticCostQVector, tempMatrix);
      CommonOps.mult(-1.0, J, tempMatrix, solutionToPack);

      // compute the current x value
      solutionValue = 0.5 * CommonOps.dot(quadraticCostQVector, solutionToPack);

      PrintTools.debug(traceSolver, "Unconstrained x : " + solutionValue);
      PrintTools.debug(traceSolver, "x : " + solutionToPack);

      // Add equality constraints to the working set A
      numberOfActiveConstraints = 0;
      for (int i = 0; i < numberOfEqualityConstraints; i++)
      {
         for (int j = 0; j < problemSize; j++)
            np.set(j, linearEqualityConstraintsAMatrix.get(j, i));

         compute_d(d, J, np);
         update_z(z, J, d);
         update_r(r, R, d);

         PrintTools.debug(traceSolver, "R : " +  R);
         PrintTools.debug(traceSolver, "z : " +  z);
         PrintTools.debug(traceSolver, "r : " +  r);
         PrintTools.debug(traceSolver, "d : " +  d);

         // compute full step length t2: i.e., the minimum step in primal space s.t. the constraint becomes feasible
         t2 = 0.0;
         if (Math.abs(CommonOps.dot(z, z)) > epsilon) // i.e. z != 0
         {
            t2 = (-CommonOps.dot(np, solutionToPack) - linearEqualityConstraintsBVector.get(i)) / CommonOps.dot(z, np);
         }

         // set x = x + t2 * z
         CommonOps.addEquals(solutionToPack, t2, z);

         // set u = u+
         lagrangeMultipliers.set(numberOfActiveConstraints, t2);
         for (int k = 0; k < numberOfActiveConstraints; k++)
            lagrangeMultipliers.set(k, lagrangeMultipliers.get(k) - t2 * r.get(k));

         // compute the new solution value
         solutionValue += 0.5 * Math.pow(t2, 2.0) * CommonOps.dot(z, np);
         activeSetIndices.set(i, -i - 1);

         if (!addConstraint(J))
         {
            PrintTools.info("Constraints are linearly dependent.");
            return solutionValue;
         }
      }

      // set iai = K \ A
      for (int i = 0; i < numberOfInequalityConstraints; i++)
         inactiveSetIndices.set(i, i);

      double solutionPairConstraintViolation = 0.0;
      constraintIndexForMinimumStepLength = 0;

      while (true)
      {
         switch(currentStep)
         {
         case step1: // ideally, step 1
            numberOfIterations++;

            PrintTools.debug(traceSolver, "x : " + solutionToPack);

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

            PrintTools.debug(traceSolver, "s : " + inequalityConstraintViolations);

            if (Math.abs(psi) < numberOfInequalityConstraints * epsilon * c1 * c2 * 100.0)
            {
               // numerically there are not infeasibilities anymore
               return solutionValue;
            }

            // save old values for u, x, and A
            for (int i = 0; i < numberOfActiveConstraints; i++)
            {
               previousLagrangeMultipliers.set(i, lagrangeMultipliers.get(i));
               previousActiveSetIndices.set(i, activeSetIndices.get(i));
            }
            // and for x
            for (int i = 0; i < problemSize; i++)
               previousSolution.set(i, solutionToPack.get(i));

            currentStep = QuadProgStep.step2;
            continue;

         case step2:
            // Step 2: check for feasibility and determine a new S-pair
            for (int i = 0; i < numberOfInequalityConstraints; i++)
            {
               // select the constraint from the inactive set that is most violated
               if (inequalityConstraintViolations.get(i) < solutionPairConstraintViolation && inactiveSetIndices.get(i) != -1 && excludeConstraintFromActiveSet.get(i) == TRUE)
               {
                  solutionPairConstraintViolation = inequalityConstraintViolations.get(i);
                  solutionPairConstraintIndex = i;
               }
            }
            if (solutionPairConstraintViolation >= 0.0)
            {
               return solutionValue;
            }

            // set np = n(solutionPairConstraintIndex)
            for (int i = 0; i < problemSize; i++)
               np.set(i, linearInequalityConstraintsCMatrix.get(i, solutionPairConstraintIndex));
            // set u = [u 0]^T
            lagrangeMultipliers.set(numberOfActiveConstraints, 0.0);
            // add the violated constraint to the active set A
            activeSetIndices.set(numberOfActiveConstraints, solutionPairConstraintIndex);

            PrintTools.debug(traceSolver, "Trying with constraint " + solutionPairConstraintIndex);
            PrintTools.debug(traceSolver, "np : " + np);

            currentStep = QuadProgStep.step2a;
            continue;

         case step2a:
            // Step 2a: determine step direction
            // compute z = H np: the step direction in the primal space (through J, see the paper)
            compute_d(d, J, np);
            update_z(z, J, d);
            // compute N* np (if activeSetSize > 0): the negative of the step direction in the dual space
            update_r(r, R, d);

            PrintTools.debug(traceSolver, "Step direction z.");
            PrintTools.debug(traceSolver, "z :" + z);
            PrintTools.debug(traceSolver, "r :" + r);
            PrintTools.debug(traceSolver, "u :" + lagrangeMultipliers);
            PrintTools.debug(traceSolver, "d :" + d);
            PrintTools.debug(traceSolver, "A :" + activeSetIndices);

            currentStep = QuadProgStep.step2b;
            continue;

         case step2b:
            // Step 2b: compute step length
            constraintIndexForMinimumStepLength = 0;
            // Compute t1: partial step length (maximum step in dual space without violating dual feasibility
            t1 = Double.POSITIVE_INFINITY;
            // find the constraintIndexForMinimumStepLength s.t. it reaches the minimum of u+(x) / r
            for (int k = numberOfEqualityConstraints; k < numberOfActiveConstraints; k++)
            {
               double minimumStepLength = lagrangeMultipliers.get(k) / r.get(k);
               if (r.get(k) > 0.0 && minimumStepLength < t1)
               {
                  t1 = minimumStepLength;
                  constraintIndexForMinimumStepLength = activeSetIndices.get(k);
               }
            }

            // Compute t2: full step length (minimum step in primal space such that the violated constraint becomes feasible
            if (Math.abs(CommonOps.dot(z, z)) > epsilon)
            {
               t2 = -inequalityConstraintViolations.get(solutionPairConstraintIndex) / CommonOps.dot(z, np);
               if (t2 < 0.0) // patch suggested by Takano Akio for handling numerical inconsistencies
                  t2 = Double.POSITIVE_INFINITY;
            }
            else
            {
               t2 = Double.POSITIVE_INFINITY;
            }

            // the step is chosen as the minimum of t1 and t2
            t = Math.min(t1, t2);

            PrintTools.debug(traceSolver, "Step Sizes: " + t + " (t1 = " + t1 + ", t2 = " + t2 + ") ");
            currentStep = QuadProgStep.step2c;
            continue;

         case step2c:
            // Step 2c: determine new S-pair and take step:

            // case (i): no step in primal or dual space
            if (t >= Double.POSITIVE_INFINITY)
            {
               // QPP is infeasible
               // FIXME: unbounded to raise
               return Double.POSITIVE_INFINITY;
            }

            // case (ii): step in dual space
            if (t2 >= Double.POSITIVE_INFINITY)
            {
               // set u = u + t * [-r 1] and drop constraintIndexForMinimumStepLength from the active set
               for (int k = 0; k < numberOfActiveConstraints; k++)
                  lagrangeMultipliers.set(k, lagrangeMultipliers.get(k) - t * r.get(k));
               lagrangeMultipliers.set(numberOfActiveConstraints, lagrangeMultipliers.get(numberOfActiveConstraints) + t);
               inactiveSetIndices.set(constraintIndexForMinimumStepLength, constraintIndexForMinimumStepLength);
               deleteConstraint(J, activeSetIndices, lagrangeMultipliers);

               PrintTools.debug(traceSolver, "in dual space : " + solutionValue);
               PrintTools.debug(traceSolver, "x : " + solutionToPack);
               PrintTools.debug(traceSolver, "z : " + z);
               PrintTools.debug(traceSolver, "A : " + activeSetIndices);

               currentStep = QuadProgStep.step2a;
               continue;
            }

            // case (iii): step in primal and dual space
            for (int k = 0; k < problemSize; k++)
               solutionToPack.set(k, solutionToPack.get(k) + t * z.get(k));
            // update the solution value
            solutionValue += t * CommonOps.dot(z, np) * (0.5 * t + lagrangeMultipliers.get(numberOfActiveConstraints));

            // u = u + t * [-r 1]
            for (int k = 0; k < numberOfActiveConstraints; k++)
               lagrangeMultipliers.set(k, lagrangeMultipliers.get(k) - t * r.get(k));
            lagrangeMultipliers.set(numberOfActiveConstraints, lagrangeMultipliers.get(numberOfActiveConstraints) + t);

            PrintTools.debug(traceSolver, "in dual space: " + solutionValue);
            PrintTools.debug(traceSolver, "x : " + solutionToPack);
            PrintTools.debug(traceSolver, "u : " + lagrangeMultipliers);
            PrintTools.debug(traceSolver, "r : " + r);
            PrintTools.debug(traceSolver, "A : " + activeSetIndices);

            currentStep = QuadProgStep.step3;
            continue;

         case step3:

            if (Math.abs(t - t2) < epsilon)
            {
               PrintTools.debug(traceSolver, "Full step has taken " + t);
               PrintTools.debug(traceSolver, "x : " + solutionToPack);

               // full step has been taken
               // add the violated constraint to the active set
               if (!addConstraint(J))
               {
                  excludeConstraintFromActiveSet.set(solutionPairConstraintIndex, FALSE);
                  deleteConstraint(J, activeSetIndices, lagrangeMultipliers);

                  PrintTools.debug(traceSolver, "R : " + R);
                  PrintTools.debug(traceSolver, "A : " + activeSetIndices);
                  PrintTools.debug(traceSolver, "iai : " + inactiveSetIndices);

                  for (int i = 0; i < numberOfInequalityConstraints; i++)
                     inactiveSetIndices.set(i, i);

                  for (int i = 0; i < numberOfActiveConstraints; i++)
                  {
                     activeSetIndices.set(i, previousActiveSetIndices.get(i));
                     inactiveSetIndices.set(activeSetIndices.get(i), -1);
                     lagrangeMultipliers.set(i, previousLagrangeMultipliers.get(i));
                  }
                  for (int i = 0; i < problemSize; i++)
                     solutionToPack.set(i, previousSolution.get(i));

                  currentStep = QuadProgStep.step2;
                  continue;
               }
               else
               {
                  inactiveSetIndices.set(solutionPairConstraintIndex, -1);
               }

               PrintTools.debug(traceSolver, "R : " + R);
               PrintTools.debug(traceSolver, "A : " + activeSetIndices);
               PrintTools.debug(traceSolver, "iai : " + inactiveSetIndices);

               currentStep = QuadProgStep.step1;
               continue;
            }

            // a partial step has taken
            PrintTools.debug(traceSolver, "Partial step has taken " + t);
            PrintTools.debug(traceSolver, "x : " + solutionToPack);

            // drop constraint constraintIndexForMinimumStepLength
            inactiveSetIndices.set(constraintIndexForMinimumStepLength, constraintIndexForMinimumStepLength);
            deleteConstraint(J, activeSetIndices, lagrangeMultipliers);

            PrintTools.debug(traceSolver, "R : " + R);
            PrintTools.debug(traceSolver, "A : " + activeSetIndices);

            // update s[ip] = CI * x + ci0
            double sum = 0.0;
            for (int k = 0; k < problemSize; k++)
               sum += linearInequalityConstraintsCMatrix.get(k, solutionPairConstraintIndex) * solutionToPack.get(k);
            inequalityConstraintViolations.set(solutionPairConstraintIndex, sum + linearInequalityConstraintsDVector.get(solutionPairConstraintIndex));

            PrintTools.debug(traceSolver, "s : " + inequalityConstraintViolations);

            currentStep = QuadProgStep.step2a;
            continue;

         default:
            throw new RuntimeException("This is an empty state.");
         }
      }
   }

   public int getNumberOfIterations()
   {
      return numberOfIterations;
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

   private final DenseMatrix64F y = new DenseMatrix64F(defaultSize);
   private void choleskySolve(DenseMatrix64F L, DenseMatrix64F xToPack, DenseMatrix64F b)
   {
      y.reshape(problemSize, 1);

      // Solve L * y = b
      forwardElimination(L, y, b);
      // Solve L^T * x = y
      backwardElimination(L, xToPack, y);
   }

   private static void forwardElimination(DenseMatrix64F L, DenseMatrix64F yToPack, DenseMatrix64F b)
   {
      int n = yToPack.getNumRows();
      yToPack.set(0, b.get(0) / L.get(0, 0));
      for (int i = 1; i < n; i++)
      {
         yToPack.set(i, b.get(i));

         for (int j = 0; j < i; j++)
         {
            yToPack.set(i, yToPack.get(i) - L.get(i,j) * yToPack.get(j));
         }
         yToPack.set(i, yToPack.get(i) / L.get(i,i));
      }
   }

   private static void backwardElimination(DenseMatrix64F U, DenseMatrix64F xToPack, DenseMatrix64F y)
   {
      int n = xToPack.getNumRows();

      xToPack.set(n - 1, y.get(n - 1) / U.get(n - 1, n - 1));

      for (int i = n - 2; i >= 0; i--)
      {
         xToPack.set(i, y.get(i));

         for (int j = i + 1; j < n; j++)
            xToPack.set(i, xToPack.get(i) - U.get(i, j) * xToPack.get(j));

         xToPack.set(i, xToPack.get(i) / U.get(i, i));
      }
   }

   private static void choleskyDecomposition(DenseMatrix64F matrixToPack)
   {
      double sum;
      int n = matrixToPack.getNumRows();

      for (int i = 0; i < n; i++)
      {
         for (int j = 1; j < n; j++)
         {
            sum = matrixToPack.get(i, j);

            for (int k = i - 1; k >= 0; k--)
               sum -= matrixToPack.get(i, k) * matrixToPack.get(j, k);

            if (i == j)
            {
               if (sum < 0.0)
               {
                  // raise error
                  throw new RuntimeException("The matrix passed to the Cholesky A = L L^T decomposition is not positive definite");
               }
               matrixToPack.set(i, i, Math.sqrt(sum));
            }
            else
            {
               matrixToPack.set(j, i, sum / matrixToPack.get(i, i));
            }
         }
         for (int k = i + 1; k < n; k++)
         {
            matrixToPack.set(i, k, matrixToPack.get(k, i));
         }
      }
   }
}
