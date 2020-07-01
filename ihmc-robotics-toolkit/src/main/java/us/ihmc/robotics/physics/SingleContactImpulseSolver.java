package us.ihmc.robotics.physics;

import static us.ihmc.robotics.physics.ContactImpulseTools.computeSlipLambda;
import static us.ihmc.robotics.physics.ContactImpulseTools.isInsideFrictionCone;
import static us.ihmc.robotics.physics.ContactImpulseTools.isInsideFrictionEllipsoid;

import org.ejml.data.DMatrix3;
import org.ejml.data.DMatrix3x3;
import org.ejml.data.DMatrix4x4;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.fixed.CommonOps_DDF3;
import org.ejml.dense.fixed.CommonOps_DDF4;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialImpulseBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialVectorReadOnly;

public class SingleContactImpulseSolver
{
   private static final double DEGENERATE_THRESHOLD = 1.0e-6;
   private static final double NEGATIVE_NORMAL_IMPULSE_THRESHOLD = -1.0e-12;

   private double beta1 = 0.35;
   private double beta2 = 0.95;
   private double beta3 = 1.15;
   private double gamma = 1.0e-6;

   private boolean computeFrictionMoment = false;
   private int problemSize = 3;
   private double coulombMomentRatio = 0.3;

   /**
    * Contact velocity, extracted from {@link #velocitySolverInput}. When evaluating frictional moment,
    * this is a 4-element vector, the last element being the angular velocity around the z-axis.
    */
   private final DMatrixRMaj c = new DMatrixRMaj(4, 1);
   /** Linear part of the contact velocity, extracted from {@link #velocitySolverInput} */
   private final DMatrix3 c_linear = new DMatrix3();
   /**
    * The 3-by-3 matrix for the linear part of the inverse of the apparent inertia matrix.
    */
   private final DMatrix3x3 M_linear_inv = new DMatrix3x3();
   /**
    * The 3-by-3 matrix for the linear part of the apparent inertia matrix.
    */
   private final DMatrix3x3 M_linear = new DMatrix3x3();
   /**
    * The 4-by-4 matrix for the linear part of the inverse of the apparent inertia matrix.
    * <p>
    * Only used when solving for the frictional moment.
    * </p>
    * <p>
    * The last row and last column are the terms linking the the angular components of the impulse and
    * velocity.
    * </p>
    */
   private final DMatrix4x4 M_full_inv = new DMatrix4x4();
   /**
    * The 4-by-4 matrix for the linear part of the apparent inertia matrix.
    * <p>
    * Only used when solving for the frictional moment.
    * </p>
    * <p>
    * The last row and last column are the terms linking the the angular components of the impulse and
    * velocity.
    * </p>
    */
   private final DMatrix4x4 M_full = new DMatrix4x4();
   /**
    * The 3-element vector extracted from the 3 first elements of the last row of {@link #M_full}.
    * <p>
    * Only used when solving for the frictional moment.
    * </p>
    * <p>
    * These elements are the coupling part between angular and linear parts.
    * </p>
    */
   private final DMatrix3 M_lin_ang = new DMatrix3();
   /**
    * Solver only used when {@link #M_inv} is not invertible, i.e. degenerate problem.
    */
   private final LinearSolverDense<DMatrixRMaj> svdSolver = LinearSolverFactory_DDRM.pseudoInverse(true);
   /** The linear impulse that fully cancels the contact velocity and may violate the friction law. */
   private final DMatrix3 lambda_linear_v_0 = new DMatrix3();
   /** The linear impulse that best cancels the contact velocity and respect the friction law. */
   private final DMatrix3 lambda_linear = new DMatrix3();
   /**
    * The impulse that fully cancels the contact velocity and may violate the friction law.
    * <p>
    * Only used when for degenerate problems.
    * </p>
    */
   private final DMatrixRMaj lambda_v_0 = new DMatrixRMaj(4, 1);

   // Flags for keeping track of what's up to date.
   private boolean is_M_linear_inv_upToDate = false;
   private boolean is_M_full_inv_upToDate = false;

   public SingleContactImpulseSolver()
   {
   }

   public void setSlipBisectionParameters(double beta1, double beta2, double beta3)
   {
      this.beta1 = beta1;
      this.beta2 = beta2;
      this.beta3 = beta3;
   }

   public void setTolerance(double gamma)
   {
      this.gamma = gamma;
   }

   public void setEnableFrictionMoment(boolean enable)
   {
      computeFrictionMoment = enable;
      problemSize = enable ? 4 : 3;
   }

   public void setCoulombMomentRatio(double ratio)
   {
      coulombMomentRatio = ratio;
   }

   public int getProblemSize()
   {
      return problemSize;
   }

   public void reset()
   {
      is_M_linear_inv_upToDate = false;
      is_M_full_inv_upToDate = false;
   }

   public void solveImpulseGeneral(SpatialVectorReadOnly velocity, DMatrixRMaj M_inv, double mu, FixedFrameSpatialImpulseBasics impulseToPack)
   {
      if (EuclidCoreTools.isZero(mu, 1.0e-12))
      { // Trivial case, i.e. there is no friction => the impulse is along the collision axis.
         impulseToPack.getLinearPart().setZ(-velocity.getLinearPart().getZ() / M_inv.get(2, 2));
         return;
      }

      if (CommonOps_DDRM.det(M_inv) <= DEGENERATE_THRESHOLD)
      {
         solveImpulseDegenerate(velocity, M_inv, mu, impulseToPack);
         return;
      }

      velocity.getLinearPart().get(c_linear);
      boolean isSlipping = solveLinearImpulse(M_inv, mu, impulseToPack);

      /*
       * When the contact is slipping and that we're considering only the linear part of it, we skip the
       * evaluation of the frictional moment and will leave the angular impulse to zero.
       */
      if (!isSlipping && computeFrictionMoment)
      {
         solveAngularImpulse(velocity.getAngularPartZ(), M_inv, mu, impulseToPack);
      }
   }

   private boolean solveLinearImpulse(DMatrixRMaj M_inv, double mu, FixedFrameSpatialImpulseBasics impulseToPack)
   {
      if (!is_M_linear_inv_upToDate)
      {
         ContactImpulseTools.extract(M_inv, 0, 0, M_linear_inv);
         CommonOps_DDF3.invert(M_linear_inv, M_linear);
         is_M_linear_inv_upToDate = true;
      }

      CommonOps_DDF3.mult(M_linear, c_linear, lambda_linear_v_0);
      CommonOps_DDF3.changeSign(lambda_linear_v_0);

      if (lambda_linear_v_0.a3 > NEGATIVE_NORMAL_IMPULSE_THRESHOLD && isInsideFrictionCone(mu, lambda_linear_v_0))
      { // Contact is sticking, i.e. satisfies Coulomb's friction cone while canceling velocity.
         impulseToPack.getLinearPart().set(lambda_linear_v_0);
         return false;
      }
      else
      { // Contact is slipping, that's the though case.
         computeSlipLambda(beta1, beta2, beta3, gamma, mu, M_linear_inv, lambda_linear_v_0, c_linear, lambda_linear, false);
         impulseToPack.getLinearPart().set(lambda_linear);
         return true;
      }
   }

   /**
    * This method attempts to compute the linear and z-angular impulse that cancels the contact
    * velocity (linear and z-angular).
    * <p>
    * In the case that canceling the z-angular velocity cannot be achieved, i.e. resulting impulse
    * violates the generalized friction law, the linear part of the impulse is prioritized over the
    * angular. The current implementation performs a bisection on the z-angular velocity to find the
    * maximum velocity that can be cancelled while satisfying the friction law.
    * </p>
    * <p>
    * This method will not compute an optimal solution in the sense the that the solution is not
    * guaranteed to provide maximum dissipation.
    * </p>
    */
   private void solveAngularImpulse(double velocityAngularZ, DMatrixRMaj M_inv, double mu, FixedFrameSpatialImpulseBasics impulseToPack)
   {
      if (!is_M_full_inv_upToDate)
      {
         M_full_inv.set(M_inv);
         CommonOps_DDF4.invert(M_full_inv, M_full);
         is_M_full_inv_upToDate = true;
      }

      /*
       * @formatter:off
       * / lambda_linear  \ = - / M_linear    M_lin_ang \ / c_linear  \
       * \ lambda_angular /     \ M_lin_ang^T M_angular / \ c_angular /
       *
       * We first compute the part of the system that is independent from c_angular:
       * lambda_linear_decoupled = -M_linear * c_linear
       * lambda_angular_coupling = -M_lin_ang . c_linear
       * When
       * @formatter:on
       */
      DMatrix3 lambda_linear_decoupled = lambda_linear_v_0;
      lambda_linear_decoupled.a1 = -M_full.a11 * c_linear.a1 - M_full.a12 * c_linear.a2 - M_full.a13 * c_linear.a3;
      lambda_linear_decoupled.a2 = -M_full.a21 * c_linear.a1 - M_full.a22 * c_linear.a2 - M_full.a23 * c_linear.a3;
      lambda_linear_decoupled.a3 = -M_full.a31 * c_linear.a1 - M_full.a32 * c_linear.a2 - M_full.a33 * c_linear.a3;

      M_lin_ang.set(M_full.a14, M_full.a24, M_full.a34);
      double lambda_angular_coupling = -CommonOps_DDF3.dot(M_lin_ang, c_linear);

      // We first check that the contact is sticking when ignoring the angular velocity, if not we abort.
      if (lambda_linear_decoupled.a3 < NEGATIVE_NORMAL_IMPULSE_THRESHOLD
            || !isInsideFrictionEllipsoid(mu, lambda_linear_decoupled, lambda_angular_coupling, coulombMomentRatio))
         return; // Unable to solve this for now, falling back to solution without friction moment.

      double M_angular = M_full.get(3, 3);
      double c_angular = velocityAngularZ;

      ContactImpulseTools.scaleAdd(-c_angular, M_lin_ang, lambda_linear_decoupled, lambda_linear);
      double lambda_angular = lambda_angular_coupling - M_angular * c_angular;

      if (lambda_linear.a3 > NEGATIVE_NORMAL_IMPULSE_THRESHOLD && isInsideFrictionEllipsoid(mu, lambda_linear, lambda_angular, coulombMomentRatio))
      {
         // The contact is sticking, we're done.
         impulseToPack.getLinearPart().set(lambda_linear);
         impulseToPack.getAngularPart().setZ(lambda_angular);
         return;
      }

      /*
       * The contact sticking is slipping when countering 100% of c_angular. We start a bisection to find
       * the max value for c_angular for which the impulse remains within the friction ellipsoid.
       */
      // The lower bound is always sticking
      double c_angular_lo = 0.0;
      // The upper bound is always slipping
      double c_angular_hi = c_angular;
      double lambda_lo_x = lambda_linear_decoupled.a1;
      double lambda_lo_y = lambda_linear_decoupled.a2;
      double lambda_lo_z = lambda_linear_decoupled.a3;
      double lambda_lo_zz = lambda_angular_coupling;
      int iteration = 0;

      while (true)
      {
         iteration++;

         if (Math.abs(c_angular_hi - c_angular_lo) < gamma)
         {
            impulseToPack.getLinearPart().set(lambda_lo_x, lambda_lo_y, lambda_lo_z);
            impulseToPack.getAngularPart().setZ(lambda_lo_zz);
            return;
         }

         if (iteration > 1000)
         {
            throw new IllegalStateException("Failed to computed friction moment");
         }

         double c_angular_mid = 0.5 * (c_angular_lo + c_angular_hi);

         ContactImpulseTools.scaleAdd(-c_angular_mid, M_lin_ang, lambda_linear_decoupled, lambda_linear);

         if (lambda_linear.a3 < NEGATIVE_NORMAL_IMPULSE_THRESHOLD)
         { // We're slipping
            c_angular_hi = c_angular_mid;
            continue;
         }

         lambda_angular = lambda_angular_coupling - M_angular * c_angular_mid;

         if (!isInsideFrictionEllipsoid(mu, lambda_linear, lambda_angular, coulombMomentRatio))
         { // We're slipping
            c_angular_hi = c_angular_mid;
            continue;
         }

         // We're sticking
         c_angular_lo = c_angular_mid;
         lambda_lo_x = lambda_linear.a1;
         lambda_lo_y = lambda_linear.a2;
         lambda_lo_z = lambda_linear.a3;
         lambda_lo_zz = lambda_angular;
      }
   }

   /*
    * TODO This is not enough to cover the degenerate case, need to decompose the problem to work in
    * reduced space, i.e. either 2D or 1D.
    */
   private void solveImpulseDegenerate(SpatialVectorReadOnly velocity, DMatrixRMaj M_inv, double mu, FixedFrameSpatialImpulseBasics impulseToPack)
   {
      lambda_v_0.reshape(problemSize, 1);
      c.reshape(problemSize, 1);
      velocity.getLinearPart().get(c);

      if (computeFrictionMoment)
         c.set(3, velocity.getAngularPart().getZ());
      svdSolver.setA(M_inv);
      svdSolver.solve(c, lambda_v_0);
      CommonOps_DDRM.changeSign(lambda_v_0);

      if (lambda_v_0.get(2) < NEGATIVE_NORMAL_IMPULSE_THRESHOLD)
         throw new IllegalStateException("Unable to fully solve degenerate case. Need to be improved.");

      if (computeFrictionMoment)
      {
         if (!isInsideFrictionEllipsoid(mu, lambda_v_0, coulombMomentRatio))
            throw new IllegalStateException("Unable to fully solve degenerate case. Need to be improved.");
      }
      else
      {
         if (!isInsideFrictionCone(mu, lambda_v_0))
            throw new IllegalStateException("Unable to fully solve degenerate case. Need to be improved.");
      }
   }

   public void printForUnitTest(DMatrixRMaj M_inv, double mu)
   {
      System.err.println(ContactImpulseTools.toStringForUnitTest(beta1, beta2, beta3, gamma, mu, M_inv, lambda_linear_v_0, c_linear));
   }
}
