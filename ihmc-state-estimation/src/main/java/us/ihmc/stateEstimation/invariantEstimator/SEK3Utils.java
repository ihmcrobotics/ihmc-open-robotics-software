package us.ihmc.stateEstimation.invariantEstimator;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.lieGroup.SO3LieGroupTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;

/**
 * Static utility class providing SE_k(3) Lie group and Lie algebra operations.
 *
 * <p>An element X of SE_k(3) is an (3+k)×(3+k) matrix sharing a single rotation
 * across k translation-like columns:
 * <pre>
 *       [ R  p₁ p₂ … p_k ]
 *   X = [ 0  1  0  … 0   ]
 *       [ 0  0  1  … 0   ]
 *       [ ⋮              ]
 * </pre>
 * with R ∈ SO(3) and each pᵢ ∈ ℝ³. It is represented as an EJML {@link DMatrixRMaj}.</p>
 *
 * <p>The Lie algebra se_k(3) is identified with ℝ^(3+3k) via the ordering
 * {@code ξ = [φ; v₁; v₂; …; v_k]}, where φ = ξ[0..2] is the rotational component and
 * each vᵢ = ξ[3+3i .. 3+3i+2] is a translational component. This generalizes the
 * {@code [φ; ρ]} convention used in {@code SE3LieGroupTools} (the k = 1 case).</p>
 *
 * <p>k is inferred from the argument sizes: {@code ξ.length = 3 + 3k} and
 * {@code X} is {@code (3+k)×(3+k)}.</p>
 *
 * <p>All methods are static; {@code ToPack} parameters are the outputs — inputs are never
 * modified. Each of {@link #exp}, {@link #log} and {@link #adjoint} has two forms: a convenience
 * form that allocates its own scratch (fine for tests and other non-real-time callers) and an
 * allocation-free overload that takes caller-owned {@link Vector3D}/{@link RotationMatrix}/{@link Matrix3D}
 * scratch. Use the latter on hard real-time loops — the estimator's per-tick predict/update path does.</p>
 */
public class SEK3Utils
{
   private SEK3Utils()
   {
   }

   /**
    * SE_k(3) exponential map: converts a Lie algebra element ξ = [φ; v₁; …; v_k] to a group element X.
    *
    * <p>Formula: R = exp(φ); pᵢ = J_l(φ)·vᵢ for each column i. Each translation column shares
    * the same left Jacobian J_l(φ) = Γ₁(φ).</p>
    *
    * @param xi          the algebra element [φ; v₁; …; v_k]; length must be 3 + 3k with k ≥ 1. Not modified.
    * @param XToPack     the group element to pack the result into; reshaped to (3+k)×(3+k). Modified.
    */
   public static void exp(double[] xi, DMatrixRMaj XToPack)
   {
      exp(xi, XToPack, new Vector3D(), new RotationMatrix(), new Matrix3D());
   }

   /**
    * Allocation-free {@link #exp(double[], DMatrixRMaj)}: identical result, but the caller supplies the
    * three scratch objects so nothing is allocated on the per-tick estimator path.
    *
    * @param xi              the algebra element [φ; v₁; …; v_k]; length must be 3 + 3k with k ≥ 1. Not modified.
    * @param XToPack         the group element to pack the result into; reshaped to (3+k)×(3+k). Modified.
    * @param phiScratch      caller-owned scratch for φ. Overwritten.
    * @param rotationScratch caller-owned scratch for R = exp(φ). Overwritten.
    * @param jacobianScratch caller-owned scratch for the left Jacobian J_l(φ). Overwritten.
    */
   public static void exp(double[] xi, DMatrixRMaj XToPack, Vector3D phiScratch, RotationMatrix rotationScratch, Matrix3D jacobianScratch)
   {
      int k = numColumns(xi.length);
      int n = 3 + k;

      // Rotational part: R = exp(φ).
      phiScratch.set(xi[0], xi[1], xi[2]);
      SO3LieGroupTools.exp(phiScratch, rotationScratch); // feeds matrix exponential of phi into R object

      // Shared left Jacobian J_l(φ) for every translation column.
      SO3LieGroupTools.leftJacobian(phiScratch, jacobianScratch); // left Jacobian (essentially \Gamma_1) as a function of phi

      XToPack.reshape(n,n);
      XToPack.zero();

      // Top-left 3×3 block = R.
      XToPack.set(0,0, rotationScratch.getM00());
      XToPack.set(0,1, rotationScratch.getM01());
      XToPack.set(0,2, rotationScratch.getM02());

      XToPack.set(1,0, rotationScratch.getM10());
      XToPack.set(1,1, rotationScratch.getM11());
      XToPack.set(1,2, rotationScratch.getM12());

      XToPack.set(2,0, rotationScratch.getM20());
      XToPack.set(2,1, rotationScratch.getM21());
      XToPack.set(2,2, rotationScratch.getM22());

      // Bottom-right k×k block = identity.
      for (int i = 0; i < k; i++)
         XToPack.set(3 + i, 3 + i, 1.0);

      // Each translation column: pᵢ = J_l(φ)·vᵢ, placed in matrix column (3 + i).
      for (int i = 0; i < k; i++)
      {
         double vx = xi[3 + 3 * i];
         double vy = xi[3 + 3 * i + 1];
         double vz = xi[3 + 3 * i + 2];

         double px = jacobianScratch.getM00() * vx + jacobianScratch.getM01() * vy + jacobianScratch.getM02() * vz;
         double py = jacobianScratch.getM10() * vx + jacobianScratch.getM11() * vy + jacobianScratch.getM12() * vz;
         double pz = jacobianScratch.getM20() * vx + jacobianScratch.getM21() * vy + jacobianScratch.getM22() * vz;

         XToPack.set(0, 3 + i, px);
         XToPack.set(1, 3 + i, py);
         XToPack.set(2, 3 + i, pz);
      }
   }

   /**
    * SE_k(3) logarithmic map: converts a group element X to its Lie algebra element ξ = [φ; v₁; …; v_k].
    *
    * <p>Formula: φ = log(R); vᵢ = J_l⁻¹(φ)·pᵢ for each column i.</p>
    *
    * @param X           the group element, an (3+k)×(3+k) matrix with k ≥ 1. Not modified.
    * @param xiToPack    the array to pack [φ; v₁; …; v_k] into; length must be 3 + 3k. Modified.
    */
   public static void log(DMatrixRMaj X, double[] xiToPack)
   {
      log(X, xiToPack, new RotationMatrix(), new Vector3D(), new Matrix3D());
   }

   /**
    * Allocation-free {@link #log(DMatrixRMaj, double[])}: identical result, but the caller supplies the
    * three scratch objects so nothing is allocated on the per-tick estimator path.
    *
    * @param X               the group element, an (3+k)×(3+k) matrix with k ≥ 1. Not modified.
    * @param xiToPack        the array to pack [φ; v₁; …; v_k] into; length must be 3 + 3k. Modified.
    * @param rotationScratch caller-owned scratch for R (read from X). Overwritten.
    * @param phiScratch      caller-owned scratch for φ = log(R). Overwritten.
    * @param jacobianScratch caller-owned scratch for the inverse left Jacobian J_l⁻¹(φ). Overwritten.
    */
   public static void log(DMatrixRMaj X, double[] xiToPack, RotationMatrix rotationScratch, Vector3D phiScratch, Matrix3D jacobianScratch)
   {
      int n =X.getNumRows();
      int k = n-3;
      if (X.getNumCols() != n || k < 1)
         throw new IllegalArgumentException("X must be a square (3+k)x(3+k) matrix with k >= 1, was " + n + "x" + X.getNumCols());
      if (xiToPack.length != 3 + 3 * k)
         throw new IllegalArgumentException("xiToPack length must be" + (3  + 3 * k) + " for a " + n + "x" + n + " element, was " + xiToPack.length);

      // Rotational part: φ = log(R), reading R from the top-left 3×3 block.
      //NOTE: if having errors here, use `setUnsafe` method if slightly non-orthonormal basis
      rotationScratch.set(X.get(0,0), X.get(0,1), X.get(0,2),
                          X.get(1,0), X.get(1,1), X.get(1,2),
                          X.get(2,0), X.get(2,1), X.get(2,2));

      SO3LieGroupTools.log(rotationScratch, phiScratch);

      SO3LieGroupTools.leftJacobianInverse(phiScratch, jacobianScratch);

      xiToPack[0] = phiScratch.getX();
      xiToPack[1] = phiScratch.getY();
      xiToPack[2] = phiScratch.getZ();

      // Each translation column: vᵢ = J_l⁻¹(φ)·pᵢ, reading pᵢ from matrix column (3 + i).
      for (int i = 0; i < k; i++)
      {
         double px = X.get(0, 3 + i);
         double py = X.get(1, 3 + i);
         double pz = X.get(2, 3 + i);

         xiToPack[3 + 3 * i] = jacobianScratch.getM00() * px + jacobianScratch.getM01() * py + jacobianScratch.getM02() * pz;
         xiToPack[3 + 3 * i + 1] = jacobianScratch.getM10() * px + jacobianScratch.getM11() * py + jacobianScratch.getM12() * pz;
         xiToPack[3 + 3 * i + 2] = jacobianScratch.getM20() * px + jacobianScratch.getM21() * py + jacobianScratch.getM22() * pz;
      }
   }

   /**
    * SE_k(3) group adjoint Ad_X, which maps a Lie algebra element ξ = [φ; v₁; …; v_k] ∈ ℝ^(3+3k)
    * under the change of frame X: Ad_X ξ = vee(X · hat(ξ) · X⁻¹).
    *
    * <p>Block structure (each block 3×3): block (0,0) = R; for each translation column i,
    * block (i,0) = hat(pᵢ)·R and block (i,i) = R; all other blocks are zero. For k = 1 this
    * is the SE(3) adjoint {@code [R, 0; hat(t)·R, R]} from {@code SE3LieGroupTools}.</p>
    *
    * @param X              the group element, an (3+k)×(3+k) matrix with k ≥ 1. Not modified.
    * @param adjointToPack  the (3+3k)×(3+3k) matrix to pack Ad_X into; reshaped. Modified.
    */
   public static void adjoint(DMatrixRMaj X, DMatrixRMaj adjointToPack)
   {
      adjoint(X, adjointToPack, new RotationMatrix(), new Vector3D(), new Matrix3D());
   }

   /**
    * Allocation-free {@link #adjoint(DMatrixRMaj, DMatrixRMaj)}: identical result, but the caller supplies
    * the three scratch objects so nothing is allocated on the per-tick estimator path.
    *
    * @param X               the group element, an (3+k)×(3+k) matrix with k ≥ 1. Not modified.
    * @param adjointToPack   the (3+3k)×(3+3k) matrix to pack Ad_X into; reshaped. Modified.
    * @param rotationScratch caller-owned scratch for R (read from X). Overwritten.
    * @param columnScratch   caller-owned scratch for the translation columns pᵢ. Overwritten.
    * @param hatScratch      caller-owned scratch for hat(pᵢ)·R. Overwritten.
    */
   public static void adjoint(DMatrixRMaj X, DMatrixRMaj adjointToPack, RotationMatrix rotationScratch, Vector3D columnScratch, Matrix3D hatScratch)
   {
      int n = X.getNumRows();
      int k = n - 3;
      if (X.getNumCols() != n || k < 1)
         throw new IllegalArgumentException("X must be a square (3+k)x(3+k) matrix with k>= 1, was " + n + "x" + X.getNumCols());

      int m = 3 + 3 * k ; // the adjoint acts on R^(3+3k)
      adjointToPack.reshape(m,m);
      adjointToPack.zero();

      rotationScratch.set(X.get(0,0), X.get(0,1), X.get(0,2),
                          X.get(1,0), X.get(1,1), X.get(1,2),
                          X.get(2,0), X.get(2,1), X.get(2,2));

      insert3x3(adjointToPack, 0, 0, rotationScratch);

      for (int i = 0; i < k; i++)
      {
         // p_i is the ith translation column, stored in X as column (i+3)
         columnScratch.set(X.get(0, 3 + i), X.get(1, 3 + i), X.get(2, 3 + i));

         // vᵢ' = hat(pᵢ)·R·φ + R·vᵢ  →  block (i,0) = hat(pᵢ)·R, block (i,i) = R.
         SO3LieGroupTools.hat(columnScratch, hatScratch);
         hatScratch.multiply(rotationScratch);

         int rowBlock = 3 + 3 * i;
         insert3x3(adjointToPack, rowBlock, 0, hatScratch);
         insert3x3(adjointToPack, rowBlock, rowBlock, rotationScratch);
      }
   }

   /**
    * Copies a 3×3 block into {@code dest} with its top-left corner at (row0, col0).
    *
    * @param dest  the destination matrix (large enough to hold the block). Modified.
    * @param row0  the destination row of the block's (0,0) entry.
    * @param col0  the destination column of the block's (0,0) entry.
    * @param block the 3×3 source block. Not modified.
    */
   private static void insert3x3(DMatrixRMaj dest, int row0, int col0, Matrix3DReadOnly block)
   {
      for (int r = 0; r < 3; r++)
         for (int c =0; c < 3; c++)
            dest.set(row0 + r, col0 + c, block.getElement(r,c));
   }

   /**
    * Infers the number of translation columns k from an algebra-vector length, validating the size.
    *
    * @param xiLength the length of the algebra vector, expected to be 3 + 3k with k ≥ 1.
    * @return k, the number of translation columns.
    * @throws IllegalArgumentException if xiLength is not 3 + 3k for some k ≥ 1.
    */
   private static int numColumns(int xiLength)
   {
      if (xiLength < 6 || (xiLength - 3) % 3 != 0)
         throw new IllegalArgumentException("ξ length must be 3 + 3k with k ≥ 1 (i.e. 6, 9, 12, …), was" + xiLength);
      return (xiLength - 3) / 3;
   }
}
