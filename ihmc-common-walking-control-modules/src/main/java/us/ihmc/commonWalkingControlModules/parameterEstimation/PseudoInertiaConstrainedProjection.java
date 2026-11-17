package us.ihmc.commonWalkingControlModules.parameterEstimation;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

/**
 * Constrained MAP projection of one rigid body's Pi-basis estimate onto the physically consistent set, in a given metric:
 *
 * <pre>
 *    minimize   1/2 (pi - piHat)^T W (pi - piHat)
 *    subject to J(pi) - eps I  is positive semidefinite,
 * </pre>
 *
 * where J(pi) is the 4 x 4 pseudo-inertia (Wensing, Kim and Slotine, RA-L 2018), linear in pi. With W = P^-1, the inverse of
 * the Kalman filter's covariance, this is the maximum a-posteriori estimate of the Gaussian posterior restricted to the
 * consistent set -- "estimate projection" in the sense of Simon (IET CTA 2010), not the Euclidean eigenvalue clip.
 * <p>
 * Solved by a primal log-barrier interior-point method (Boyd and Vandenberghe, Ch. 11): for an increasing barrier weight t,
 * Newton on t * f0(pi) - log det(J(pi) - eps I), with backtracking that keeps the Cholesky factor of J - eps I defined. The
 * objective is a Mahalanobis distance, so the duality-gap tolerance 4 / t is dimensionless. No external solver; about a
 * hundred 10 x 10 Newton steps worst case. If piHat is already consistent the constraint is inactive and piHat is returned.
 * </p>
 */
public class PseudoInertiaConstrainedProjection
{
   public static final int N = 10;

   private final double eps;
   private final double gapTolerance;
   private final DMatrixRMaj[] basis = new DMatrixRMaj[N];

   private final DMatrixRMaj jt = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj jtInv = new DMatrixRMaj(4, 4);
   private final DMatrixRMaj[] b = new DMatrixRMaj[N];
   private final DMatrixRMaj grad = new DMatrixRMaj(N, 1);
   private final DMatrixRMaj hess = new DMatrixRMaj(N, N);
   private final DMatrixRMaj step = new DMatrixRMaj(N, 1);
   private final DMatrixRMaj diff = new DMatrixRMaj(N, 1);
   private final DMatrixRMaj wDiff = new DMatrixRMaj(N, 1);
   private final DMatrixRMaj trial = new DMatrixRMaj(N, 1);
   private final DMatrixRMaj x = new DMatrixRMaj(N, 1);
   private final LinearSolverDense<DMatrixRMaj> newtonSolver = LinearSolverFactory_DDRM.symmPosDef(N);
   private final LinearSolverDense<DMatrixRMaj> generalSolver = LinearSolverFactory_DDRM.linear(N);

   private int lastNewtonSteps;
   private double lastDistance;
   private boolean lastActive;
   private boolean lastConverged;
   private final DMatrixRMaj j0Inv = new DMatrixRMaj(4, 4);
   private final double[] ldC = new double[N];

   public PseudoInertiaConstrainedProjection(double eps, double gapTolerance)
   {
      this.eps = eps;
      this.gapTolerance = gapTolerance;
      for (int i = 0; i < N; i++)
      {
         basis[i] = new DMatrixRMaj(4, 4);
         b[i] = new DMatrixRMaj(4, 4);
      }
      // J = [S h; h^T m], S = 1/2 tr(I) 1 - I; pi = (m, hx, hy, hz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz)
      basis[0].set(3, 3, 1.0);
      for (int k = 0; k < 3; k++)
      {
         basis[1 + k].set(k, 3, 1.0);
         basis[1 + k].set(3, k, 1.0);
      }
      int[] diagIndex = {4, 7, 9};
      for (int a = 0; a < 3; a++)
         for (int k = 0; k < 3; k++)
            basis[diagIndex[a]].set(k, k, k == a ? -0.5 : 0.5);
      basis[5].set(0, 1, -1.0);
      basis[5].set(1, 0, -1.0);
      basis[6].set(0, 2, -1.0);
      basis[6].set(2, 0, -1.0);
      basis[8].set(1, 2, -1.0);
      basis[8].set(2, 1, -1.0);
   }

   public static void pseudoInertia(DMatrixRMaj pi, DMatrixRMaj jToPack)
   {
      double m = pi.get(0), ixx = pi.get(4), ixy = pi.get(5), ixz = pi.get(6), iyy = pi.get(7), iyz = pi.get(8), izz = pi.get(9);
      double halfTrace = 0.5 * (ixx + iyy + izz);
      jToPack.reshape(4, 4);
      jToPack.set(0, 0, halfTrace - ixx);
      jToPack.set(1, 1, halfTrace - iyy);
      jToPack.set(2, 2, halfTrace - izz);
      jToPack.set(0, 1, -ixy);
      jToPack.set(1, 0, -ixy);
      jToPack.set(0, 2, -ixz);
      jToPack.set(2, 0, -ixz);
      jToPack.set(1, 2, -iyz);
      jToPack.set(2, 1, -iyz);
      for (int k = 0; k < 3; k++)
      {
         jToPack.set(k, 3, pi.get(1 + k));
         jToPack.set(3, k, pi.get(1 + k));
      }
      jToPack.set(3, 3, m);
   }

   /** log det(J(pi) - eps I) via Cholesky, or NaN if J - eps I is not positive definite. Leaves (J - eps I)^-1 in jtInv. */
   private double logDetShifted(DMatrixRMaj pi)
   {
      pseudoInertia(pi, jt);
      for (int k = 0; k < 4; k++)
         jt.add(k, k, -eps);
      // 4 x 4 Cholesky by hand (no allocation)
      double[][] l = new double[4][4];
      double logDet = 0.0;
      for (int i = 0; i < 4; i++)
      {
         for (int j = 0; j <= i; j++)
         {
            double s = jt.get(i, j);
            for (int k = 0; k < j; k++)
               s -= l[i][k] * l[j][k];
            if (i == j)
            {
               if (!(s > 0.0))
                  return Double.NaN;
               l[i][i] = Math.sqrt(s);
               logDet += 2.0 * Math.log(l[i][i]);
            }
            else
            {
               l[i][j] = s / l[j][j];
            }
         }
      }
      // inverse from the factor: solve L L^T X = I column by column
      for (int c = 0; c < 4; c++)
      {
         double[] y = new double[4];
         for (int i = 0; i < 4; i++)
         {
            double s = i == c ? 1.0 : 0.0;
            for (int k = 0; k < i; k++)
               s -= l[i][k] * y[k];
            y[i] = s / l[i][i];
         }
         for (int i = 3; i >= 0; i--)
         {
            double s = y[i];
            for (int k = i + 1; k < 4; k++)
               s -= l[k][i] * jtInv.get(k, c);
            jtInv.set(i, c, s / l[i][i]);
         }
      }
      return logDet;
   }

   public boolean isStrictlyConsistent(DMatrixRMaj pi)
   {
      return !Double.isNaN(logDetShifted(pi));
   }

   private double objective(DMatrixRMaj pi, DMatrixRMaj piHat, DMatrixRMaj w, double t)
   {
      double logDet = logDetShifted(pi);
      if (Double.isNaN(logDet))
         return Double.POSITIVE_INFINITY;
      CommonOps_DDRM.subtract(pi, piHat, diff);
      CommonOps_DDRM.mult(w, diff, wDiff);
      return 0.5 * t * CommonOps_DDRM.dot(diff, wDiff) - logDet;
   }

   /**
    * Projects {@code piHat} in the metric {@code w} (symmetric positive definite, 10 x 10). {@code feasibleStart} must be strictly
    * consistent (J - eps I positive definite), e.g. the nominal body or the previous projection. Returns false (and leaves
    * {@code result} = feasibleStart) only if the start is not strictly feasible.
    */
   public boolean project(DMatrixRMaj piHat, DMatrixRMaj w, DMatrixRMaj feasibleStart, DMatrixRMaj result)
   {
      lastNewtonSteps = 0;
      if (isStrictlyConsistent(piHat))
      {
         result.set(piHat);
         lastActive = false;
         lastDistance = 0.0;
         return true;
      }
      lastActive = true;
      x.set(feasibleStart);
      result.set(feasibleStart);
      if (!isStrictlyConsistent(x))
         return false;

      // Initial barrier weight balancing the two terms at the start (Boyd and Vandenberghe 11.3.1): t0 = m / f0(x0), m = 4.
      CommonOps_DDRM.subtract(x, piHat, diff);
      CommonOps_DDRM.mult(w, diff, wDiff);
      double f0Start = 0.5 * CommonOps_DDRM.dot(diff, wDiff);
      double t0 = 4.0 / Math.max(f0Start, 1.0e-12);
      for (double t = t0; ; t *= 10.0)
      {
         for (int newton = 0; newton < 200; newton++)
         {
            double f = objective(x, piHat, w, t); // also leaves jtInv at x
            // gradient and Hessian of the barrier: g_i = -tr(Jt^-1 A_i), H_ij = tr(Jt^-1 A_i Jt^-1 A_j)
            for (int i = 0; i < N; i++)
               CommonOps_DDRM.mult(jtInv, basis[i], b[i]);
            for (int i = 0; i < N; i++)
            {
               grad.set(i, t * wDiff.get(i) - trace(b[i]));
               for (int j = i; j < N; j++)
               {
                  double h = t * w.get(i, j) + traceOfProduct(b[i], b[j]);
                  hess.set(i, j, h);
                  hess.set(j, i, h);
               }
            }
            LinearSolverDense<DMatrixRMaj> solver = newtonSolver.setA(hess.copy()) ? newtonSolver : generalSolver;
            if (solver == generalSolver && !generalSolver.setA(hess.copy()))
               break;
            solver.solve(grad, step);
            double decrement2 = CommonOps_DDRM.dot(grad, step);
            if (!(decrement2 > 1.0e-12))
               break;
            double alpha = 1.0;
            double fTrial;
            do
            {
               CommonOps_DDRM.add(x, -alpha, step, trial);
               fTrial = objective(trial, piHat, w, t);
               alpha *= 0.5;
            }
            while ((fTrial > f - 0.25 * 2.0 * alpha * decrement2 || Double.isInfinite(fTrial)) && alpha > 1.0e-12);
            if (Double.isInfinite(fTrial))
               break;
            x.set(trial);
            lastNewtonSteps++;
            if (decrement2 < 1.0e-10)
               break;
         }
         if (4.0 / t < gapTolerance)
            break;
      }
      result.set(x);
      CommonOps_DDRM.subtract(x, piHat, diff);
      CommonOps_DDRM.mult(w, diff, wDiff);
      lastDistance = Math.sqrt(Math.max(0.0, CommonOps_DDRM.dot(diff, wDiff)));
      return true;
   }

   /**
    * [TMECH] Log-determinant (entropic) MAP of Lee, Wensing and Park (T-RO 2020), solved recursively on the filter's summary:
    *
    * <pre>
    *    minimize   1/2 (pi - piHat)^T W (pi - piHat)  +  gamma * D(J(pi) || J0),
    *    D(J || J0) = -log det(J0^-1 J) + tr(J0^-1 J) - 4      (Bregman divergence of -log det; zero iff J = J0),
    * </pre>
    *
    * with (piHat, W^-1) the unconstrained linear Kalman filter's mean and covariance, i.e. the least-squares data term
    * over all measurements so far (with the filter's random-walk forgetting), and J0 the prior body. The log-det term keeps
    * the minimizer strictly inside the physically consistent set (it is +infinity on its boundary), so no separate barrier
    * or continuation is needed: the problem is strictly convex and damped Newton converges in a few steps from a warm start.
    * {@code priorBody} (Pi basis) gives J0 and must be strictly consistent; {@code feasibleStart} is the warm start (the prior
    * body is used if it is not strictly consistent). Uses this instance's eps shift in the log-det (construct with eps = 0 for
    * the exact divergence).
    */
   public boolean logDetMap(DMatrixRMaj piHat, DMatrixRMaj w, DMatrixRMaj priorBody, double gamma, DMatrixRMaj feasibleStart,
                            DMatrixRMaj result)
   {
      lastNewtonSteps = 0;
      lastActive = true;
      result.set(feasibleStart);
      if (!isStrictlyConsistent(priorBody))
         return false;
      pseudoInertia(priorBody, j0Inv);
      CommonOps_DDRM.invert(j0Inv);
      // c_i = tr(J0^-1 A_i): gradient of the linear term tr(J0^-1 J(pi))
      for (int i = 0; i < N; i++)
         ldC[i] = traceOfProduct(j0Inv, basis[i]);
      boolean warm = isStrictlyConsistent(feasibleStart);
      x.set(warm ? feasibleStart : priorBody);
      // Online the warm start (last tick's MAP) is usually within a step or two of the answer. If not (a long move along a
      // nearly singular face of the consistent set), keep the improved feasible iterate and continue next tick from it,
      // like a real-time iteration: f decreases monotonically, so the estimate handed out only gets better.
      boolean converged = logDetNewton(piHat, w, gamma, 50);
      if (!converged && !warm)
      {
         // Cold or far start: path following from the prior body (the minimizer as gamma -> infinity), dividing gamma by 10
         // per stage as in a barrier method (Boyd and Vandenberghe 11.3), so each stage starts near its own minimizer.
         x.set(priorBody);
         CommonOps_DDRM.subtract(x, piHat, diff);
         CommonOps_DDRM.mult(w, diff, wDiff);
         double gammaK = Math.max(gamma, 0.25 * 0.5 * CommonOps_DDRM.dot(diff, wDiff));
         while (true)
         {
            converged = logDetNewton(piHat, w, gammaK, 100);
            if (gammaK <= gamma)
               break;
            gammaK = Math.max(gamma, 0.1 * gammaK);
         }
      }
      result.set(x);
      CommonOps_DDRM.subtract(x, piHat, diff);
      CommonOps_DDRM.mult(w, diff, wDiff);
      lastDistance = Math.sqrt(Math.max(0.0, CommonOps_DDRM.dot(diff, wDiff)));
      lastConverged = converged;
      return true;
   }

   /** Damped Newton on the log-det MAP objective from x (in place); true if the Newton decrement fell below tolerance. */
   private boolean logDetNewton(DMatrixRMaj piHat, DMatrixRMaj w, double gamma, int maxSteps)
   {
      for (int newton = 0; newton < maxSteps; newton++)
      {
         double f = logDetMapObjective(x, piHat, w, ldC, gamma); // leaves jtInv at x and wDiff = W (x - piHat)
         for (int i = 0; i < N; i++)
            CommonOps_DDRM.mult(jtInv, basis[i], b[i]);
         for (int i = 0; i < N; i++)
         {
            grad.set(i, wDiff.get(i) + gamma * (ldC[i] - trace(b[i])));
            for (int j = i; j < N; j++)
            {
               double h = w.get(i, j) + gamma * traceOfProduct(b[i], b[j]);
               hess.set(i, j, h);
               hess.set(j, i, h);
            }
         }
         LinearSolverDense<DMatrixRMaj> solver = newtonSolver.setA(hess.copy()) ? newtonSolver : generalSolver;
         if (solver == generalSolver && !generalSolver.setA(hess.copy()))
            return false;
         solver.solve(grad, step);
         double decrement2 = CommonOps_DDRM.dot(grad, step);
         // suboptimality ~ decrement^2 / 2, in the objective's (Mahalanobis) units; relative floor for round-off in f
         double tolerance = 2.0e-10 + 1.0e-12 * Math.abs(f);
         if (!(decrement2 > tolerance))
            return true;
         double alpha = 1.0;
         double fTrial;
         do
         {
            CommonOps_DDRM.add(x, -alpha, step, trial);
            fTrial = logDetMapObjective(trial, piHat, w, ldC, gamma);
            alpha *= 0.5;
         }
         while ((fTrial > f - 0.25 * 2.0 * alpha * decrement2 || Double.isInfinite(fTrial)) && alpha > 1.0e-12);
         // line search exhausted: no representable decrease left, i.e. converged to working precision if the decrement is small
         boolean exhausted = alpha <= 1.0e-12;
         if (Double.isInfinite(fTrial) || fTrial > f)
            return decrement2 < 1.0e-6 * (1.0 + Math.abs(f));
         x.set(trial);
         lastNewtonSteps++;
         if (exhausted)
            return decrement2 < 1.0e-6 * (1.0 + Math.abs(f));
      }
      return false;
   }

   public boolean wasLastConverged()
   {
      return lastConverged;
   }

   /** D(J(pi) || J0) for J0 given by its inverse and log det; NaN if J(pi) - eps I is not positive definite. */
   public double logDetDivergence(DMatrixRMaj pi, DMatrixRMaj j0Inverse, double logDetJ0)
   {
      double logDet = logDetShifted(pi);
      if (Double.isNaN(logDet))
         return Double.NaN;
      pseudoInertia(pi, jt);
      return -(logDet - logDetJ0) + traceOfProduct(j0Inverse, jt) - 4.0;
   }

   /** log det J(pi) (eps-shifted), NaN if not positive definite. */
   public double logDet(DMatrixRMaj pi)
   {
      return logDetShifted(pi);
   }

   private double logDetMapObjective(DMatrixRMaj pi, DMatrixRMaj piHat, DMatrixRMaj w, double[] c, double gamma)
   {
      double logDet = logDetShifted(pi);
      if (Double.isNaN(logDet))
         return Double.POSITIVE_INFINITY;
      CommonOps_DDRM.subtract(pi, piHat, diff);
      CommonOps_DDRM.mult(w, diff, wDiff);
      double linear = 0.0;
      for (int i = 0; i < N; i++)
         linear += c[i] * pi.get(i);
      return 0.5 * CommonOps_DDRM.dot(diff, wDiff) + gamma * (linear - logDet);
   }

   private static double trace(DMatrixRMaj a)
   {
      return a.get(0, 0) + a.get(1, 1) + a.get(2, 2) + a.get(3, 3);
   }

   private static double traceOfProduct(DMatrixRMaj a, DMatrixRMaj c)
   {
      double s = 0.0;
      for (int i = 0; i < 4; i++)
         for (int k = 0; k < 4; k++)
            s += a.get(i, k) * c.get(k, i);
      return s;
   }

   public int getLastNewtonSteps()
   {
      return lastNewtonSteps;
   }

   /** Mahalanobis distance sqrt((pi - piHat)^T W (pi - piHat)) of the last projection (0 if the constraint was inactive). */
   public double getLastDistance()
   {
      return lastDistance;
   }

   public boolean wasLastActive()
   {
      return lastActive;
   }
}
