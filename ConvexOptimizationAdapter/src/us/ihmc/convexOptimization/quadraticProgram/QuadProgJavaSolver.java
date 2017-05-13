package us.ihmc.convexOptimization.quadraticProgram;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class QuadProgJavaSolver
{
   private static final boolean traceSolver = false;

   private static final int defaultSize = 100;
   private static final double epsilon = 0.00001;

   private final DenseMatrix64F R = new DenseMatrix64F(defaultSize, defaultSize);

   private final DenseMatrix64F s = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F z = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F r = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F d = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F np = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F u = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F x_old = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F u_old = new DenseMatrix64F(defaultSize);

   private final TIntArrayList A = new TIntArrayList(defaultSize);
   private final TIntArrayList A_old = new TIntArrayList(defaultSize);
   private final TIntArrayList iai = new TIntArrayList(defaultSize);
   private final DenseMatrix64F iaexcl = new DenseMatrix64F(defaultSize);

   private final DenseMatrix64F costQuadraticLowerInverse = new DenseMatrix64F(defaultSize, defaultSize);

   private final DenseMatrix64F scalarMatrix = new DenseMatrix64F(1, 1);

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.lu(0);

   private int problemSize;
   private int numberOfEqualityConstraints;
   private int numberOfInequalityConstraints;

   /**
    * The problem is of the form:
    * min 0.5 * x G x + g0 x
    * s.t.
    *     CE^T x + ce0 = 0
    *     CI^T x + ci0 >= 0
    */
   /** Utility functions for updating some data needed by the solution method */
   private void compute_d(DenseMatrix64F dToPack, DenseMatrix64F J, DenseMatrix64F np)
   {
      CommonOps.multTransA(J, np, dToPack);

   }

   private void update_z(DenseMatrix64F zToPack, DenseMatrix64F J, DenseMatrix64F d, int iq)
   {
      CommonOps.mult(J, d, zToPack);
   }

   private void update_r(DenseMatrix64F rToPack, DenseMatrix64F R, DenseMatrix64F d, int iq)
   {
      solver.setA(R);
      solver.solve(d, rToPack);
   }

   private boolean addConstraint(DenseMatrix64F R, DenseMatrix64F J, DenseMatrix64F d, int iq, double R_norm)
   {
      int n = d.getNumRows();

      PrintTools.debug(traceSolver, "Add constraint " + iq);

      double cc, ss, h, t1, t2, xny;

      // we have to find the Givens rotation which will reduce the element d(j) to zero.
      // if it is already zero, we don't have to do anything, except of decreasing j
      for (int j = n - 1; j >= iq; j--)
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
         if (MathTools.epsilonEquals(h, 0.0, epsilon))
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
         for (int k = 0; k < n; k++)
         {
            t1 = J.get(k, j - 1);
            t2 = J.get(k, j);
            J.set(k, j - 1, t1 * cc + t2 * ss);
            J.set(k, j, xny * (t1 + J.get(k, j - 1)) - t2);
         }
      }

      // update the number of constraints added
      iq++;
      // To update R we have to put the iq components fo the d vector into column iq - 1 of R
      for (int i = 0; i < iq; i++)
         R.set(i, iq - 1, d.get(i));

      PrintTools.debug(traceSolver, "" + iq);
      PrintTools.debug(traceSolver, "R : " + R);
      PrintTools.debug(traceSolver, "J : " + J);
      PrintTools.debug(traceSolver, "d : " + d);

      if (Math.abs(d.get(iq - 1)) < epsilon * R_norm)
      {
         // problem degenerate
         return false;
      }

      R_norm = Math.max(R_norm, Math.abs(d.get(iq - 1)));
      return true;
   }

   private void deleteConstraint(DenseMatrix64F R, DenseMatrix64F J, DenseMatrix64F A, DenseMatrix64F u, int n, int p, int iq, int l)
   {
      PrintTools.debug(traceSolver, "Delete constraint " + l + " " + iq);

      double cc, ss, h, xny, t1, t2;
      int qq = -1;

      // Find the index qq for active constraint l to be removed
      for (int i = p; i < iq; i++)
      {
         if (A.get(i) == l)
         {
            qq = i;
            break;
         }
      }

      // remove the constraint from the active set and the duals
      for (int i = qq; i < iq - 1; i++)
      {
         A.set(i, A.get(i + 1));
         u.set(i, u.get(i + 1));

         for (int j = 0; j < n; j++)
            R.set(j, i, R.get(j, i + 1));
      }

      A.set(iq - 1, A.get(iq));
      u.set(iq - 1, u.get(iq));
      A.set(iq, 0.0);
      u.set(iq, 0.0);
      for (int j = 0; j < iq; j++)
         R.set(j, iq - 1, 0.0);

      // constraint has been fully removed
      iq--;
      PrintTools.debug(traceSolver, "/ " + iq);

      if (iq == 0)
         return;

      for (int j = qq; j < iq; j++)
      {
         cc = R.get(j, j);
         ss = R.get(j + 1, j);
         h = distance(cc, ss);

         if (Math.abs(h) < epsilon)
            continue;
         ;

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
         for (int k = j + 1; k < iq; k++)
         {
            t1 = R.get(j, k);
            t2 = R.get(j + 1, k);
            R.set(j, k, t1 * cc + t2 * ss);
            R.set(j + 1, k, xny * (t1 + R.get(j, k)) - t2);
         }

         for (int k = 0; k < n; k++)
         {
            t1 = J.get(k, j);
            t2 = J.get(k, j + 1);
            J.set(k, j, t1 * cc + t2 * ss);
            J.set(k, j + 1, xny * (J.get(k, j) + t1) - t2);
         }
      }
   }


   /** Utility functions for computing the Cholesky decomposition and solving linear systems */

   /** Utility functions for computing the scalar product and the euclidean distance between two numbers */
   public void reshape()
   {
      int numberOfConstraints = numberOfEqualityConstraints + numberOfInequalityConstraints;

      R.reshape(problemSize, problemSize);
      s.reshape(numberOfConstraints, 1);
      z.reshape(problemSize, 1);
      r.reshape(numberOfConstraints, 1);
      d.reshape(problemSize, 1);
      np.reshape(problemSize, 1);
      u.reshape(numberOfConstraints, 1);
      x_old.reshape(problemSize, 1);
      u_old.reshape(numberOfConstraints, 1);

      A.fill(0, numberOfConstraints, 0);
      A_old.fill(0, numberOfConstraints, 0);
      iai.fill(0, numberOfConstraints, 0);
      iaexcl.reshape(numberOfConstraints, 1);
   }

   private void zero()
   {
      R.zero();
      s.zero();
      z.zero();
      r.zero();
      d.zero();
      np.zero();
      u.zero();
      x_old.zero();
      u_old.zero();

      iaexcl.zero();
   }



   // The solving function, implementing the Goldfarb-Idani method
   public double solveQuadprog(DenseMatrix64F costQuadraticMatrix, DenseMatrix64F costLinearVector,
         DenseMatrix64F CE, DenseMatrix64F ce0, DenseMatrix64F CI, DenseMatrix64F ci0, DenseMatrix64F solution)
   {
      problemSize = costQuadraticMatrix.getNumCols();
      numberOfEqualityConstraints = CE.getNumCols();
      numberOfInequalityConstraints = CI.getNumCols();

      if (costLinearVector.getNumCols() != 1)
         throw new RuntimeException("costLinearVector.getNumCols() != 1");
      if (costQuadraticMatrix.getNumRows() != costLinearVector.getNumRows())
         throw new RuntimeException("costQuadraticMatrix.getNumRows() != costLinearVector.getNumRows()");
      if (costQuadraticMatrix.getNumRows() != problemSize)
         throw new RuntimeException("costQuadraticMatrix.getNumRows() != costQuadraticMatrix.getNumCols()");
      if (CE.getNumRows() != problemSize)
         throw new RuntimeException("Equality constraint matrix is incompatible, wrong number of variables.");
      if (ce0.getNumRows() != numberOfEqualityConstraints)
         throw new RuntimeException("Equality constraint objective is incompatible, wrong number of constraints.");
      if (CI.getNumRows() != problemSize)
         throw new RuntimeException("Inequality constraint matrix is incompatible, wrong number of variables.");
      if (ci0.getNumRows() != numberOfInequalityConstraints)
         throw new RuntimeException("IneEquality constraint objective is incompatible, wrong number of constraints.");

      solution.reshape(problemSize, 1);
      solution.zero();

      reshape();
      zero();

      int q = 0;

      PrintTools.debug(traceSolver, "G : " + costQuadraticMatrix);
      PrintTools.debug(traceSolver, "g0 : " + costLinearVector);
      PrintTools.debug(traceSolver, "CE : " + CE);
      PrintTools.debug(traceSolver, "ce0 : " + ce0);
      PrintTools.debug(traceSolver, "CI : " + CI);
      PrintTools.debug(traceSolver, "ci0 : " + ci0);

      /** Preprocessing phase */

      double R_norm = 1.0;
      double c1 = CommonOps.trace(costQuadraticMatrix);

      costQuadraticLowerInverse.reshape(problemSize, problemSize);

      // compute the inverse of the factorized matrix G^-1, this is the initial value for H
      solver.setA(costQuadraticMatrix);
      solver.invert(costQuadraticLowerInverse);

      PrintTools.debug(traceSolver, "J : " + costQuadraticLowerInverse);

      double c2 = CommonOps.trace(costQuadraticLowerInverse);

      // c1 * c2 is an estimate for cond(G)

      // Find the unconstrained minimizer of the quadratic form 0.5 * x G x + g0 x
      // this is the feasible point in the dual space
      // x = G^-1 * g0
      solver.solve(costLinearVector, solution);
      CommonOps.scale(-1.0, solution);

      // compute the current solution value
      CommonOps.multTransA(0.5, costLinearVector, solution, scalarMatrix);
      double f_value = scalarMatrix.get(0, 0);

      PrintTools.debug(traceSolver, "Unconstrained solution : " + f_value);
      PrintTools.debug(traceSolver, "x : " + solution);

      // Add equality constraints to the working set A
      int ip; // this is the index of the constraint to be added to the active set
      int iq = 0;
      int iter = 0;
      double t, t1, t2; // t is the step length, which is the minimum of the partial step t1 and the full step length t2
      for (int i = 0; i < numberOfEqualityConstraints; i++)
      {
         for (int j = 0; j < problemSize; j++)
            np.set(j, 0, CE.get(j, i));

         compute_d(d, costQuadraticLowerInverse, np);
         update_z(z, costQuadraticLowerInverse, d, iq);
         update_r(R, r, d, iq);

         PrintTools.debug(traceSolver, "R : " +  R);
         PrintTools.debug(traceSolver, "z : " +  z);
         PrintTools.debug(traceSolver, "r : " +  r);
         PrintTools.debug(traceSolver, "d : " +  d);

         // compute full step length t2: i.e., the minimum step in primal space s.t. the constraint becomes feasible
         t2 = 0.0;
         CommonOps.multTransA(z, z, scalarMatrix);
         if (!MatrixTools.isEmptyMatrix(scalarMatrix))
         {
            CommonOps.multTransA(-1.0, np, solution, scalarMatrix);
            t2 = scalarMatrix.get(0, 0) - ce0.get(i);

            CommonOps.multTransA(z, np, scalarMatrix);
            t2 /= scalarMatrix.get(0, 0);
         }

         CommonOps.addEquals(solution, t2, z);

         // set u = u+
         u.set(iq, t2);
         for (int k = 0; k < iq; k++)
            u.set(k, u.get(k) - t2 * r.get(k));

         // compute the new solution value
         CommonOps.multTransA(z, np, scalarMatrix);
         f_value += 0.5 * Math.pow(t2, 2.0) * scalarMatrix.get(0 ,0);
         A.set(i, -i - 1);

         if (!addConstraint(R, costQuadraticLowerInverse, d, iq, R_norm))
         {
            throw new RuntimeException("Constraints are linearly dependent.");
            return f_value;
         }
      }

      // set iai = K \ A
      for (int i = 0; i < numberOfInequalityConstraints; i++)
         iai.set(i, i);

      l1: // // FIXME: 5/13/17 
      iter++;

      PrintTools.debug(traceSolver, "x : " + solution);

      // step 1: choose a violated constraint
      for (int i = numberOfEqualityConstraints; i < iq; i++)
      {
         ip = A.get(i);
         iai.set(ip, -1);
      }

      // compute s(x) = ci^t * x + ci0 for all elements of K \ A
      double ss = 0.0;
      double psi = 0.0; // this value will contain the sum of all infeasibilities
      ip = 0; // ip will be the index of the chosen violated constraint
      for (int i = 0; i < numberOfInequalityConstraints; i++)
      {
         iaexcl.set(i, 0.0);
         double sum = 0.0;
         for (int j = 0; j < problemSize; j++)
            sum += CI.get(j, i) * solution.get(j);
         sum += ci0.get(i);
         s.set(i, sum);
         psi += Math.min(0.0, sum);
      }

      PrintTools.debug(traceSolver, "s : " + s);

      if (Math.abs(psi) < numberOfInequalityConstraints * epsilon * c1 * c2 * 100.0)
      {
         // numerically there are not infeasibilities anymore
         return f_value;
      }

      // save old values for u, solution, and A
      for (int i = 0; i < iq; i++)
      {
         u_old.set(i, u.get(i));
         A_old.set(i, A.get(i));
      }
      // and for solution
      for (int i = 0; i < problemSize; i++)
         x_old.set(i, solution.get(i));
      
      l2: //// FIXME: 5/13/17
      // Step 2: check for feasibility and determine a new S-pair
      for (int i = 0; i < numberOfInequalityConstraints; i++)
      {
         if (s.get(i) < ss && iai.get(i) != -1 && iaexcl.get(i) == 1)
         {
            ss = s.get(i);
            ip = i;
         }
      }
      if (ss >= 0.0)
      {
         q = iq;
         return f_value;
      }

      // set np = n(ip)
      for (int i = 0; i < problemSize; i++)
         np.set(i, CI.get(i, ip));
      // set u = (u 0)^T
      u.set(iq, 0.0);
      // add ip to the active set A
      A.set(iq, ip);

      PrintTools.debug(traceSolver, "Trying with constraint " + ip);
      PrintTools.debug(traceSolver, "np : " + np);

      l2a: // // FIXME: 5/13/17
      // Step 2a: determine step direction
      // compute z = H np: the step direction in the primal space (through J, see the paper)
      compute_d(d, costQuadraticLowerInverse, np);
      update_z(z, costQuadraticLowerInverse, d, iq);
      // compute N* np (if q > 0): the negative of the step direction in the dual space
      update_r(R, r, d, iq);

      PrintTools.debug(traceSolver, "Step direction z.");
      PrintTools.debug(traceSolver, "z :" + z);
      PrintTools.debug(traceSolver, "r :" + r);
      PrintTools.debug(traceSolver, "u :" + u);
      PrintTools.debug(traceSolver, "d :" + d);
      PrintTools.debug(traceSolver, "A :" + A);

      // Step 2b: compute step length
      int l = 0;
      // Compute t1: partial step length (maximum step in dual space without violating dual feasibility
      t1 = Double.POSITIVE_INFINITY;
      // find the index l s.t. it reaches the minimum of u+(x) / r
      for (int k = numberOfEqualityConstraints; k < iq; k++)
      {
         double tmp;
         if (r.get(k) > 0.0 && (tmp = u.get(k) / r.get(k)) < t1)
         {
            t1 = tmp;
            l = A.get(k);
         }
      }

      // Compute t2: full step length (minimum step in primal space such that the constraint ip becomes feasible
      CommonOps.multTransA(z, z, scalarMatrix);
      if (Math.abs(z.get(0)) > epsilon)
      {
         CommonOps.multTransA(z, np, scalarMatrix);
         t2 = -s.get(ip) / scalarMatrix.get(0);
      }
      else
      {
         t2 = Double.POSITIVE_INFINITY;
      }

      // the step is chosen as the minimum of t1 and t2
      t = Math.min(t1, t2);

      PrintTools.debug(traceSolver, "Step Sizes: " + t + " (t1 = " + t1 + ", t2 = " + t2 + ") ");

      // Step 2c: determine new S-pair and take step:

      // case (i): no step in primal or dual space
      if (t >= Double.POSITIVE_INFINITY)
      {
         /* QPP is infeasible */
         // FIXME: unbounded to raise
         q = iq;
         return Double.POSITIVE_INFINITY;
      }
      // case (ii): step in dual space
      if (t2 >= Double.POSITIVE_INFINITY)
      {
         // set u = u + t * (-r 1) and drop constraint l from the active set A
         for (int k = 0; k < iq; k++)
            u.set(k, u.get(k) - t * r.get(k));
         u.set(iq, u.get(iq) + t);
         iai.set(l, l);
         deleteConstraint(R, costQuadraticLowerInverse, A, u, problemSize, numberOfEqualityConstraints, iq, l);

         PrintTools.debug(traceSolver, "in dual space: " + f_value);
         PrintTools.debug(traceSolver, "x : " + solution);
         PrintTools.debug(traceSolver, "z : " + z);
         PrintTools.debug(traceSolver, "A : " + A);
         
         goto l2a; // // FIXME: 5/13/17
      }
      // case (iii): step in primal and dual space
      for (int k = 0; k < problemSize; k++)
         solution.set(k, solution.get(k) + t * z.get(k));
      // update the solution value
      CommonOps.multTransA(z, np, scalarMatrix);
      f_value += t * scalarMatrix.get(0) * (0.5 * t + u.get(iq));
      // u = u + t * (-r 1)
      for (int k = 0; k < iq; k++)
         u.set(k, u.get(k) - t * r.get(k));
      u.set(iq, u.get(iq) + t);

      PrintTools.debug(traceSolver, "in dual space: " + f_value);
      PrintTools.debug(traceSolver, "x : " + solution);
      PrintTools.debug(traceSolver, "u : " + u);
      PrintTools.debug(traceSolver, "r : " + r);
      PrintTools.debug(traceSolver, "A : " + A);

      if (MathTools.epsilonEquals(t - t2, 0.0, epsilon))
      {
         PrintTools.debug(traceSolver, "Full step has taken " + t);
         PrintTools.debug(traceSolver, "x : " + solution);

         // full step has taken
         // add constraint ip to the active set
         if (!addConstraint(R, costQuadraticLowerInverse, d, iq, R_norm))
         {
            iaexcl.set(ip, 0.0);
            deleteConstraint(R, costQuadraticLowerInverse, A, u, problemSize, numberOfEqualityConstraints, iq, ip);

            PrintTools.debug(traceSolver, "R : " + R);
            PrintTools.debug(traceSolver, "A : " + A);
            PrintTools.debug(traceSolver, "iai : " + iai);

            for (int i = 0; i < numberOfInequalityConstraints; i++)
               iai.set(i, i);
            for (int i = numberOfEqualityConstraints; i < iq; i++)
            {
               A.set(i, A_old.get(i));
               u.set(i, u_old.get(i));
               iai.set(A.get(i), -1);
            }
            for (int i = 0; i < problemSize; i++)
               solution.set(i, x_old.get(i));
            goto l2; // // FIXME: 5/13/17
         }
         else
         {
            iai.set(ip, -1);
         }

         PrintTools.debug(traceSolver, "R : " + R);
         PrintTools.debug(traceSolver, "A : " + A);
         PrintTools.debug(traceSolver, "iai : " + iai);
         
         goto l1; // // FIXME: 5/13/17
      }

      // a partial step has taken
      PrintTools.debug(traceSolver, "Partial step has taken " + t);
      PrintTools.debug(traceSolver, "x : " + solution);

      // drop constraint l
      iai.set(l, l);
      deleteConstraint(R, costQuadraticLowerInverse, A, u, problemSize, numberOfEqualityConstraints, iq, l);

      PrintTools.debug(traceSolver, "R : " + R);
      PrintTools.debug(traceSolver, "A : " + A);

      // update s(ip) = CI * x + ci0
      double sum = 0.0;
      for (int k = 0; k < problemSize; k++)
         sum += CI.get(k, ip) * solution.get(k);
      s.set(ip, sum + ci0.get(ip));

      PrintTools.debug(traceSolver, "s : " + s);

      goto l2a; // // FIXME: 5/13/17 
   }

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
}
