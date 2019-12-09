package us.ihmc.robotics.linearAlgebra.careSolvers;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.DecompositionFactory;
import org.ejml.ops.CommonOps;
import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;

/**
 * The CARE is defined as
 * X A + A^T X - X B R^-1 B^T X + Q = 0
 *
 * This is an adaptation of the python algorithm found
 * https://github.com/scipy/scipy/blob/master/scipy/linalg/_solvers.py
 */
public class QZCARESolver
{
   private enum SolverType {DARE, CARE}

   private final SolverData data = new SolverData();
   private final DenseMatrix64F H = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F aTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F bTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F sTranspose = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F J = new DenseMatrix64F(0, 0);

   /**
    TODO format this java doc
   Solves the continuous-time algebraic Riccati equation (CARE).
   The CARE is defined as
      .. math::
   X A + A^H X - X B R^{-1} B^H X + Q = 0
   The limitations for a solution to exist are :
      * All eigenvalues of :math:`A` on the right half plane, should be
   controllable.
        * The associated hamiltonian pencil (See Notes), should have
   eigenvalues sufficiently away from the imaginary axis.
      Moreover, if ``e`` or ``s`` is not precisely ``None``, then the
   generalized version of CARE
    .. math::
   E<sup>T</sup> X A + A<sup>T</sup> X E - (E<sup>T</sup> X B + S) R^<sup>-1</sup> (B^HXE + S^H) + Q = 0
   is solved. When omitted, ``e`` is assumed to be the identity and ``s``
   is assumed to be the zero matrix with sizes compatible with ``a`` and
    ``b``, respectively.
      Parameters
    ----------
   a : (M, M) array_like
   Square matrix
   b : (M, N) array_like
      Input
   q : (M, M) array_like
      Input
   r : (N, N) array_like
   Nonsingular square matrix
   e : (M, M) array_like, optional
   Nonsingular square matrix
   s : (M, N) array_like, optional
      Input
   balanced : bool, optional
   The boolean that indicates whether a balancing step is performed
   on the data. The default is set to True.
   Returns
    -------
   x : (M, M) ndarray
   Solution to the continuous-time algebraic Riccati equation.
   Raises
    ------
   LinAlgError
   For cases where the stable subspace of the pencil could not be
   isolated. See Notes section and the references for details.
      See Also
    --------
   solve_discrete_are : Solves the discrete-time algebraic Riccati equation
   Notes
    -----
   The equation is solved by forming the extended hamiltonian matrix pencil,
   as described in [1]_, :math:`H - \lambda J` given by the block matrices ::
      [ A    0    B ]             [ E   0    0 ]
      [-Q  -A^H  -S ] - \lambda * [ 0  E^H   0 ]
      [ S^H B^H   R ]             [ 0   0    0 ]
   and using a QZ decomposition method.
   In this algorithm, the fail conditions are linked to the symmetry
   of the product :math:`U_2 U_1^{-1}` and condition number of
    :math:`U_1`. Here, :math:`U` is the 2m-by-m matrix that holds the
   eigenvectors spanning the stable subspace with 2-m rows and partitioned
   into two m-row matrices. See [1]_ and [2]_ for more details.
   In order to improve the QZ decomposition accuracy, the pencil goes
   through a balancing step where the sum of absolute values of
    :math:`H` and :math:`J` entries (after removing the diagonal entries of
      the sum) is balanced following the recipe given in [3]_.
    .. versionadded:: 0.11.0
   References
    ----------
          .. [1]  P. van Dooren , "A Generalized Eigenvalue Approach For Solving
   Riccati Equations.", SIAM Journal on Scientific and Statistical
   Computing, Vol.2(2), DOI: 10.1137/0902010
      .. [2] A.J. Laub, "A Schur Method for Solving Algebraic Riccati
   Equations.", Massachusetts Institute of Technology. Laboratory for
   Information and Decision Systems. LIDS-R ; 859. Available online :
   http://hdl.handle.net/1721.1/1301
      .. [3] P. Benner, "Symplectic Balancing of Hamiltonian Matrices", 2001,
   SIAM J. Sci. Comput., 2001, Vol.22(5), DOI: 10.1137/S1064827500367993
      Examples
    --------
   Given `a`, `b`, `q`, and `r` solve for `x`:
      >>> from scipy import linalg
    >>> a = np.array([[4, 3], [-4.5, -3.5]])
      >>> b = np.array([[1], [-1]])
      >>> q = np.array([[9, 6], [6, 4.]])
      >>> r = 1
      >>> x = linalg.solve_continuous_are(a, b, q, r)
      >>> x
array([[ 21.72792206,  14.48528137],
            [ 14.48528137,   9.65685425]])
    >>> np.allclose(a.T.dot(x) + x.dot(a)-x.dot(b).dot(b.T).dot(x), -q)
   True
   */
   public void solve(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R, boolean balanced)
   {
      solve(A, B, Q, R, null, balanced);
   }

   public void solve(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R, DenseMatrix64F E, boolean balanced)
   {
      solve(A, B, Q, R, null, null, balanced);
   }

   public void solve(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F Q, DenseMatrix64F R, DenseMatrix64F E, DenseMatrix64F S, boolean balanced)
   {
      data.a = A;
      data.b = B;
      data.q = Q;
      data.r = R;
      data.e = E;
      data.s = S;

      assertValidArguments(data, SolverType.CARE);
      int totalSize = 2 * data.m + data.n;
      H.reshape(totalSize, totalSize);
      aTranspose.reshape(data.a.getNumCols(), data.a.getNumRows());
      bTranspose.reshape(data.b.getNumCols(), data.b.getNumRows());
      if (data.s != null)
      {
         sTranspose.reshape(data.s.getNumCols(), data.s.getNumRows());
         CommonOps.transpose(data.s, sTranspose);
      }

      CommonOps.transpose(data.a, aTranspose);
      CommonOps.transpose(data.b, bTranspose);
      H.zero();

      // TODO set a block to zero instead of calling the above zero function.
      MatrixTools.setMatrixBlock(H, 0, 0, data.a, 0, 0, data.m, data.m, 1.0);
      MatrixTools.setMatrixBlock(H, 0, 2 * data.m, data.b, 0, 0, data.m, data.n, 1.0);
      MatrixTools.setMatrixBlock(H, data.m, 0, data.q, 0, 0, data.m, data.m, -1.0);
      MatrixTools.setMatrixBlock(H, data.m, data.m, aTranspose, 0, 0, data.m, data.m, -1.0);
      if (data.s != null)
      {
         MatrixTools.setMatrixBlock(H, data.m, 2 * data.m, data.s, 0, 0, data.m, data.n, -1.0);
         MatrixTools.setMatrixBlock(H, 2 * data.m, 0, sTranspose, 0, 0, data.n, data.m, 1.0);
      }
      MatrixTools.setMatrixBlock(H, 2 * data.m, data.m, bTranspose, 0, 0, data.n, data.m, 1.0);
      MatrixTools.setMatrixBlock(H, 2 * data.m, 2 * data.m, data.r, 0, 0, data.n, data.n, 1.0);

      // TODO what the heck do I do here?
      if (data.generalizedCase && data.e != null)
      {

      }

      // TODO
      if (balanced)
      {
         // xGEBAL does not remove the diagonals before scaling. Also, to avoid destroying the Symplectic structure, we follow Ref. 3
         DenseMatrix64F M = new DenseMatrix64F();
      }


   }

   /**
    * A helper function to validate the arguments supplied to the
   Riccati equation solvers. Any discrepancy found in the input
   matrices leads to a ``ValueError`` exception.
      Essentially, it performs:
      - a check whether the input is free of NaN and Infs
        - a pass for the data through ``numpy.atleast_2d()``
      - squareness check of the relevant arrays
        - shape consistency check of the arrays
        - singularity check of the relevant arrays
        - symmetricity check of the relevant matrices
        - a check whether the regular or the generalized version is asked.
   This function is used by ``solve_continuous_are`` and
    ``solve_discrete_are``.

   Parameters
    ----------
   a, b, q, r, e, s : array_like
   Input data
   solverType : str
   Accepted arguments are 'care' and 'dare'.

   Returns
    -------
   a, b, q, r, e, s : ndarray
   Regularized input data
   m, n : int
   shape of the problem
   r_or_c : type
   Data type of the problem, returns float or complex
   gen_or_not : bool
   Type of the equation, True for generalized and False for regular ARE.
    */
   private void assertValidArguments(SolverData data, SolverType solverType)
   {
      DenseMatrix64F a = data.a;
      DenseMatrix64F b = data.b;
      DenseMatrix64F q = data.q;
      DenseMatrix64F r = data.q;
      DenseMatrix64F e = data.e;
      DenseMatrix64F s = data.s;

      // Shape consistency checks
      assertMatrixSquare(a);
      assertMatrixSquare(q);
      assertMatrixSquare(r);

      int m = b.getNumRows();
      int n = b.getNumCols();
      if (m != a.getNumRows())
         throw new RuntimeException("Matrix a and b should have the same number of rows.");
      if (m != q.getNumRows())
         throw new RuntimeException("Matrix a and q should have the same number of rows.");
      if (n != r.getNumRows())
         throw new RuntimeException("Matrix b and r should have the same number of cols.");

      // Check if the matrices Q and R are (sufficiently) Hermitian
      assertSufficientlyHermitian(q);
      assertSufficientlyHermitian(r);

      // Continuous time ARE should have a nonsingular r matrix
      if (solverType == SolverType.CARE)
         assertNonSingular(r);

      // Check if the generalized case is required with omitted arguments, perform late shaping etc.
      if (e != null)
      {
         data.e = e;
         assertMatrixSquare(e);
         if (m != e.getNumRows())
            throw new RuntimeException("Matrix a and e should have the same shape.");
         assertNonSingular(e);
      }
      if (s != null)
      {
         if (m != s.getNumRows() || n != s.getNumCols())
            throw new RuntimeException("Matrix b and s should have the same shape.");
      }

      data.generalizedCase = e != null || s != null;
      data.m = m;
      data.n = n;
   }

   private static void assertMatrixSquare(DenseMatrix64F mat)
   {
      if (mat.getNumRows() != mat.getNumCols())
         throw new RuntimeException("Matrix is not square.");
   }

   /**
    * Since we require our matrices to be real, this is just a symmetry check
    */
   private static void assertSufficientlyHermitian(DenseMatrix64F mat)
   {
      assertSufficientlyHermitian(mat, 0.0);
   }

   private static void assertSufficientlyHermitian(DenseMatrix64F mat, double epsilon)
   {
      assertMatrixSquare(mat);
      for (int i = 0; i < mat.getNumRows(); i++)
      {
         for (int j = 0; j < i; j++)
         {
            if (!MathTools.epsilonEquals(mat.get(i, j), mat.get(j, i), epsilon))
               throw new RuntimeException("Matrix is not symmetric");
         }
      }
   }

   private static void assertNonSingular(DenseMatrix64F mat)
   {
      assertNonSingular(mat, 0.0);
   }
   private static void assertNonSingular(DenseMatrix64F mat, double epsilon)
   {
      if (MathTools.epsilonEquals(CommonOps.det(mat), 0.0, epsilon))
         throw new IllegalArgumentException("Matrix is singular.");
//      SingularValueDecomposition<DenseMatrix64F> svd = DecompositionFactory.svd(mat.getNumRows(), mat.getNumCols(), true, true, false);
//      svd.decompose(mat);
//      double[] singularValues = svd.getSingularValues();
//      double smallestSingularValues = MathTools.min(singularValues);
//      if (smallestSingularValues <= 0.0)
//         throw new RuntimeException("Matrix is singular.");
   }




   private class SolverData
   {
      public DenseMatrix64F a, b, q, r, e, s;
      boolean generalizedCase;
      int m, n;
   }
}
