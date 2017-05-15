package us.ihmc.convexOptimization.quadraticProgram;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DenseMatrix64F;

import org.ejml.ops.CommonOps;
import us.ihmc.commons.PrintTools;

public class JavaQuadProgSolver
{
   private enum QuadProgStep {step1, step2, step2a, step2b, step2c, step3};

   private QuadProgStep currentStep;
   private final DenseMatrix64F negAin = new DenseMatrix64F(0), negAeq = new DenseMatrix64F(0);

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
   private final DenseMatrix64F u_old = new DenseMatrix64F(defaultSize);
   private final DenseMatrix64F x_old = new DenseMatrix64F(defaultSize);

   private final TIntArrayList A = new TIntArrayList(defaultSize);
   private final TIntArrayList A_old = new TIntArrayList(defaultSize);
   private final TIntArrayList iai = new TIntArrayList(defaultSize);
   private final TIntArrayList iaexcl = new TIntArrayList(defaultSize);

   private final DenseMatrix64F J = new DenseMatrix64F(defaultSize, defaultSize);

   private final DenseMatrix64F quadraticCostQMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F quadraticCostQVector = new DenseMatrix64F(defaultSize);

   private int n;
   private int m;
   private int p;

   private int iq;
   private double R_norm;
   private int l;

   private int numberOfIterations;

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
      // compute d = H^T * np
      for (int i = 0; i < n; i++)
      {
         double sum = 0.0;

         for (int j = 0; j < n; j++)
            sum += J.get(j, i) * np.get(j);

         dToPack.set(i, sum);
      }
   }

   private void update_z(DenseMatrix64F zToPack, DenseMatrix64F J, DenseMatrix64F d, int iq)
   {
      // setting of z = J * d
      for (int i = 0; i < n; i++)
      {
         zToPack.set(i, 0.0);
         for (int j = iq; j < n; j++)
            zToPack.set(i, zToPack.get(i) + J.get(i, j) * d.get(j));
      }
   }

   private void update_r(DenseMatrix64F R, DenseMatrix64F rToPack, DenseMatrix64F d, int iq)
   {
      // setting of r = R^-1 d
      for (int i = iq - 1; i >= 0; i--)
      {
         double sum = 0.0;
         for (int j = i + 1; j < iq; j++)
            sum += R.get(i, j) * r.get(j);

         rToPack.set(i, (d.get(i) - sum) / R.get(i, i));
      }
   }

   private boolean addConstraint(DenseMatrix64F J)
   {
      PrintTools.debug(traceSolver, "Add constraint " + iq);

      double cc, ss, h, t1, t2, xny;

      // we have to find the Givens rotation which will reduce the element d(j) to zero.
      // if it is already zero, we don't have to do anything, except of decreasing j
      for (int j = n - 1; j >= iq + 1; j--)
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

   private void deleteConstraint(DenseMatrix64F J, TIntArrayList A, DenseMatrix64F u)
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
      A.set(iq, 0);
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


   public void reshape()
   {
      int numberOfConstraints = p + m;

      R.reshape(n, n);
      s.reshape(numberOfConstraints, 1);
      z.reshape(n, 1);
      r.reshape(numberOfConstraints, 1);
      d.reshape(n, 1);
      np.reshape(n, 1);
      u.reshape(numberOfConstraints, 1);
      x_old.reshape(n, 1);
      u_old.reshape(numberOfConstraints, 1);

      A.fill(0, numberOfConstraints, 0);
      A_old.fill(0, numberOfConstraints, 0);
      iai.fill(0, numberOfConstraints, 0);
      iaexcl.fill(0, numberOfConstraints, 0);
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


   // The solving function, implementing the Goldfarb-Idani method
   public double solveQuadprog(DenseMatrix64F G, DenseMatrix64F g0,
         DenseMatrix64F CE, DenseMatrix64F ce0, DenseMatrix64F CI, DenseMatrix64F ci0, DenseMatrix64F x, boolean initialize)
   {
      //// TODO: 5/13/17  initialize
      n = G.getNumCols();
      p = CE.getNumCols();
      m = CI.getNumCols();


      if (g0.getNumCols() != 1)
         throw new RuntimeException("g0.getNumCols() != 1");
      if (G.getNumRows() != g0.getNumRows())
         throw new RuntimeException("G.getNumRows() != g0.getNumRows()");
      if (G.getNumRows() != n)
         throw new RuntimeException("G.getNumRows() != G.getNumCols()");
      if (CE.getNumRows() != n)
         throw new RuntimeException("Equality constraint matrix is incompatible, wrong number of variables.");
      if (ce0.getNumRows() != p)
         throw new RuntimeException("Equality constraint objective is incompatible, wrong number of constraints.");
      if (CI.getNumRows() != n)
         throw new RuntimeException("Inequality constraint matrix is incompatible, wrong number of variables.");
      if (ci0.getNumRows() != m)
         throw new RuntimeException("IneEquality constraint objective is incompatible, wrong number of constraints.");

      quadraticCostQMatrix.reshape(n, n);
      quadraticCostQMatrix.set(G);

      quadraticCostQVector.reshape(n, 1);
      quadraticCostQVector.set(g0);

      x.reshape(n, 1);
      x.zero();

      reshape();
      zero();

      currentStep = QuadProgStep.step1;

      double f_value, psi, c1, c2, sum, ss;
      double t = 0.0, t1 = 0.0, t2 = 0.0; // t is the step length, which is the minimum of the partial step t1 and the full step length t2
      int ip = 0; // this is the index of the constraint to be added to the active set
      int q = 0;// Size of the active set {@link A}
      numberOfIterations = 0;
      int me = p; // number of equality constraints
      int mi = m; // number of inequality constraints

      PrintTools.debug(traceSolver, "G : " + quadraticCostQMatrix);
      PrintTools.debug(traceSolver, "g0 : " + quadraticCostQVector);
      PrintTools.debug(traceSolver, "CE : " + CE);
      PrintTools.debug(traceSolver, "ce0 : " + ce0);
      PrintTools.debug(traceSolver, "CI : " + CI);
      PrintTools.debug(traceSolver, "ci0 : " + ci0);

      J.reshape(n, n);

      /** Preprocessing phase */

      // compute the trace of the original matrix quadraticCostQMatrix
      c1 = 0.0;
      for (int i = 0; i < n; i++)
      {
         c1 += quadraticCostQMatrix.get(i, i);
      }
      //c1 = CommonOps.trace(quadraticCostQMatrix);

      // decompose the matrix quadraticCostQMatrix in the form L^T L
      choleskyDecomposition(quadraticCostQMatrix);
      PrintTools.debug(traceSolver, "G : " + quadraticCostQMatrix);

      // initialize the matrix R
      for (int i = 0; i < n; i++)
      {
         d.set(i, 0.0);
         for (int j = 0; j < n; j++)
            R.set(i, j, 0.0);
      }
      R_norm = 1.0; // this variable will hold the norm of the matrix R

      // compute the inverse of the factorized matrix G^-1, this is the initial value for H
      c2 = 0.0;
      for (int i = 0; i < n; i++)
      {
         d.set(i, 1.0);
         forwardElimination(quadraticCostQMatrix, z, d);
         for (int j = 0; j < n; j++)
            J.set(i, j, z.get(j));
         d.set(i, 0.0);
         c2 += z.get(i);
      }

      //c2 = CommonOps.trace(J);
      PrintTools.debug(traceSolver, "J : " + J);

      // c1 * c2 is an estimate for cond(G)

      // Find the unconstrained minimizer of the quadratic form 0.5 * x G x + g0 x
      // this is the feasible point in the dual space
      // x = -G^-1 * g0
      choleskySolve(quadraticCostQMatrix, x, quadraticCostQVector);
      for (int i = 0; i < n; i++)
         x.set(i, -x.get(i));

      // compute the current x value
      f_value = 0.5 * scalarProduct(quadraticCostQVector, x);

      PrintTools.debug(traceSolver, "Unconstrained x : " + f_value);
      PrintTools.debug(traceSolver, "x : " + x);

      // Add equality constraints to the working set A
      iq = 0;
      for (int i = 0; i < me; i++)
      {
         for (int j = 0; j < n; j++)
            np.set(j, CE.get(j, i));

         compute_d(d, J, np);
         update_z(z, J, d, iq);
         update_r(R, r, d, iq);

         PrintTools.debug(traceSolver, "R : " +  R);
         PrintTools.debug(traceSolver, "z : " +  z);
         PrintTools.debug(traceSolver, "r : " +  r);
         PrintTools.debug(traceSolver, "d : " +  d);

         // compute full step length t2: i.e., the minimum step in primal space s.t. the constraint becomes feasible
         t2 = 0.0;
         if (Math.abs(scalarProduct(z, z)) > epsilon) // i.e. z != 0
         {
            t2 = (-scalarProduct(np, x) - ce0.get(i)) / scalarProduct(z, np);
         }

         // set x = x + t2 * z
         //CommonOps.addEquals(x, t2, z);
         for (int k = 0; k < n; k++)
            x.set(k, x.get(k) + t2 * z.get(k));

         // set u = u+
         u.set(iq, t2);
         for (int k = 0; k < iq; k++)
            u.set(k, u.get(k) - t2 * r.get(k));

         // compute the new solution value
         //CommonOps.multTransA(z, np, scalarMatrix);
         f_value += 0.5 * Math.pow(t2, 2.0) * scalarProduct(z, np);
         A.set(i, -i - 1);

         if (!addConstraint(J))
         {
            PrintTools.info("Constraints are linearly dependent.");
            return f_value;
         }
      }

      // set iai = K \ A
      for (int i = 0; i < mi; i++)
         iai.set(i, i);

      ss = 0.0;
      psi = 0.0;
      l = 0;

      while (true)
      {
         switch(currentStep)
         {
         case step1: // ideally, step 1
            numberOfIterations++;

            PrintTools.debug(traceSolver, "x : " + x);

            // step 1: choose a violated constraint
            for (int i = me; i < iq; i++)
            {
               ip = A.get(i);
               iai.set(ip, -1);
            }

            // compute s(x) = ci^t * x + ci0 for all elements of K \ A
            ss = 0.0;
            psi = 0.0; // this value will contain the sum of all infeasibilities
            ip = 0; // ip will be the index of the chosen violated constraint
            for (int i = 0; i < mi; i++)
            {
               iaexcl.set(i, 1);
               sum = 0.0;
               for (int j = 0; j < n; j++)
                  sum += CI.get(j, i) * x.get(j);
               sum += ci0.get(i);
               s.set(i, sum);
               psi += Math.min(0.0, sum);
            }

            PrintTools.debug(traceSolver, "s : " + s);

            if (Math.abs(psi) < mi * epsilon * c1 * c2 * 100.0)
            {
               // numerically there are not infeasibilities anymore
               q = iq;

               return f_value;
            }

            // save old values for u, x, and A
            for (int i = 0; i < iq; i++)
            {
               u_old.set(i, u.get(i));
               A_old.set(i, A.get(i));
            }
            // and for x
            for (int i = 0; i < n; i++)
               x_old.set(i, x.get(i));

            currentStep = QuadProgStep.step2;
            continue;

         case step2:
            // Step 2: check for feasibility and determine a new S-pair
            for (int i = 0; i < mi; i++)
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
            for (int i = 0; i < n; i++)
               np.set(i, CI.get(i, ip));
            // set u = [u 0]^T
            u.set(iq, 0.0);
            // add ip to the active set A
            A.set(iq, ip);

            PrintTools.debug(traceSolver, "Trying with constraint " + ip);
            PrintTools.debug(traceSolver, "np : " + np);

            currentStep = QuadProgStep.step2a;
            continue;

         case step2a:
            // Step 2a: determine step direction
            // compute z = H np: the step direction in the primal space (through J, see the paper)
            compute_d(d, J, np);
            update_z(z, J, d, iq);
            // compute N* np (if q > 0): the negative of the step direction in the dual space
            update_r(R, r, d, iq);

            PrintTools.debug(traceSolver, "Step direction z.");
            PrintTools.debug(traceSolver, "z :" + z);
            PrintTools.debug(traceSolver, "r :" + r);
            PrintTools.debug(traceSolver, "u :" + u);
            PrintTools.debug(traceSolver, "d :" + d);
            PrintTools.debug(traceSolver, "A :" + A);

            currentStep = QuadProgStep.step2b;
            continue;

         case step2b:
            // Step 2b: compute step length
            l = 0;
            // Compute t1: partial step length (maximum step in dual space without violating dual feasibility
            t1 = Double.POSITIVE_INFINITY;
            // find the index l s.t. it reaches the minimum of u+(x) / r
            for (int k = me; k < iq; k++)
            {
               if (r.get(k) > 0.0)
               {
                  if (u.get(k) / r.get(k) < t1)
                  {
                     t1 = u.get(k) / r.get(k);
                     l = A.get(k);
                  }
               }
            }

            // Compute t2: full step length (minimum step in primal space such that the constraint ip becomes feasible
            if (Math.abs(scalarProduct(z, z)) > epsilon)
            {
               t2 = -s.get(ip) / scalarProduct(z, np);
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
               q = iq;
               return Double.POSITIVE_INFINITY;
            }

            // case (ii): step in dual space
            if (t2 >= Double.POSITIVE_INFINITY)
            {
               // set u = u + t * [-r 1] and drop constraint l from the active set
               for (int k = 0; k < iq; k++)
                  u.set(k, u.get(k) - t * r.get(k));
               u.set(iq, u.get(iq) + t);
               iai.set(l, l);
               deleteConstraint(J, A, u);

               PrintTools.debug(traceSolver, "in dual space: " + f_value);
               PrintTools.debug(traceSolver, "x : " + x);
               PrintTools.debug(traceSolver, "z : " + z);
               PrintTools.debug(traceSolver, "A : " + A);

               currentStep = QuadProgStep.step2a;
               continue;
            }

            // case (iii): step in primal and dual space
            for (int k = 0; k < n; k++)
               x.set(k, x.get(k) + t * z.get(k));
            // update the solution value
            //CommonOps.multTransA(z, np, scalarMatrix);
            //f_value += t * scalarMatrix.get(0) * (0.5 * t + u.get(iq));
            f_value += t * scalarProduct(z, np) * (0.5 * t + u.get(iq));

            // u = u + t * (-r 1)
            for (int k = 0; k < iq; k++)
               u.set(k, u.get(k) - t * r.get(k));
            u.set(iq, u.get(iq) + t);

            PrintTools.debug(traceSolver, "in dual space: " + f_value);
            PrintTools.debug(traceSolver, "x : " + x);
            PrintTools.debug(traceSolver, "u : " + u);
            PrintTools.debug(traceSolver, "r : " + r);
            PrintTools.debug(traceSolver, "A : " + A);

            currentStep = QuadProgStep.step3;
            continue;

         case step3:

            if (Math.abs(t - t2) < epsilon)
            {
               PrintTools.debug(traceSolver, "Full step has taken " + t);
               PrintTools.debug(traceSolver, "x : " + x);

               // full step has been taken
               // add constraint ip to the active set
               if (!addConstraint(J))
               {
                  iaexcl.set(ip, 0);
                  deleteConstraint(J, A, u);

                  PrintTools.debug(traceSolver, "R : " + R);
                  PrintTools.debug(traceSolver, "A : " + A);
                  PrintTools.debug(traceSolver, "iai : " + iai);

                  for (int i = 0; i < m; i++)
                     iai.set(i, i);

                  for (int i = 0; i < iq; i++)
                  {
                     A.set(i, A_old.get(i));
                     iai.set(A.get(i), -1);
                     u.set(i, u_old.get(i));
                  }
                  for (int i = 0; i < n; i++)
                     x.set(i, x_old.get(i));

                  currentStep = QuadProgStep.step2;
                  continue;
               }
               else
               {
                  iai.set(ip, -1);
               }

               PrintTools.debug(traceSolver, "R : " + R);
               PrintTools.debug(traceSolver, "A : " + A);
               PrintTools.debug(traceSolver, "iai : " + iai);

               currentStep = QuadProgStep.step1;
               continue;
            }

            // a partial step has taken
            PrintTools.debug(traceSolver, "Partial step has taken " + t);
            PrintTools.debug(traceSolver, "x : " + x);

            // drop constraint l
            iai.set(l, l);
            deleteConstraint(J, A, u);

            PrintTools.debug(traceSolver, "R : " + R);
            PrintTools.debug(traceSolver, "A : " + A);

            // update s[ip] = CI * x + ci0
            sum = 0.0;
            for (int k = 0; k < n; k++)
               sum += CI.get(k, ip) * x.get(k);
            s.set(ip, sum + ci0.get(ip));

            PrintTools.debug(traceSolver, "s : " + s);

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
      y.reshape(xToPack.getNumRows(), 1);

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

   private static void choleskyDecomposition(DenseMatrix64F A)
   {
      double sum;
      int n = A.getNumRows();

      for (int i = 0; i < n; i++)
      {
         for (int j = 1; j < n; j++)
         {
            sum = A.get(i, j);

            for (int k = i - 1; k >= 0; k--)
               sum -= A.get(i, k) * A.get(j, k);

            if (i == j)
            {
               if (sum < 0.0)
               {
                  // raise error
                  throw new RuntimeException("The matrix passed to the Cholesky A = L L^T decomposition is not positive definite");
               }
               A.set(i, i, Math.sqrt(sum));
            }
            else
            {
               A.set(j, i, sum / A.get(i, i));
            }
         }
         for (int k = i + 1; k < n; k++)
         {
            A.set(i, k, A.get(k, i));
         }
      }
   }

   private static double scalarProduct(DenseMatrix64F x, DenseMatrix64F y)
   {
      double sum = 0.0;

      for (int i = 0; i < x.numRows; i++)
      {
         sum += x.get(i) * y.get(i);
      }

      return sum;
   }


}
