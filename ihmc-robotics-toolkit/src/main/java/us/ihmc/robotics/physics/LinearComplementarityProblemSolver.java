package us.ihmc.robotics.physics;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import gnu.trove.iterator.TIntIterator;
import gnu.trove.set.TIntSet;
import gnu.trove.set.hash.TIntHashSet;

/**
 * Algorithm from: <i>"`"</i> by
 * <i>David Baraff</i>
 * <p>
 * This class solves the following problem:
 * 
 * <pre>
 * <b>a</b> = <b>A</b><b>f</b> + <b>b</b>
 * subject &forall;i&in;[0,N] to:
 *    a<sub>i</sub> &geq; 0
 *    f<sub>i</sub> &geq; 0
 *    f<sub>i</sub> a<sub>i</sub> = 0
 * </pre>
 * </p>
 * Where <tt>N</tt> is the number of contacts to solve forces for, <tt><b>a</b></tt> is
 * <tt>N</tt>-element vector representing the relative accelerations at each contact,
 * <tt><b>A</b></tt> is the <tt>N</tt>-by-<tt>N</tt> (symmetric and positive semi-definite)
 * transformation matrix from the contact force to contact accelerations, <tt><b>f</b></tt> is the
 * <tt>N</tt>-element vector representing the relative forces at each contact, and <tt><b>b</b></tt>
 * is the <tt>N</tt>-element bias vector representing the relative accelerations without contact
 * forces.
 * <p>
 * Convention on contact force and acceleration is:
 * <ul>
 * <li>positive value implies separation of the two rigid-bodies in contact.
 * <li>negative value implies inter-penetration of the two rigid-bodies.
 * </ul>
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class LinearComplementarityProblemSolver
{
   private final TIntSet clampedIndexSet = new TIntHashSet(10, 0.5f, -1);
   private final TIntSet notClampedIndexSet = new TIntHashSet(10, 0.5f, -1);

   private final DMatrixRMaj A_CC = new DMatrixRMaj(10, 10);
   private final DMatrixRMaj A_Cd = new DMatrixRMaj(10, 10);
   private final DMatrixRMaj delta_f_C = new DMatrixRMaj(10, 1);

   private final DMatrixRMaj f = new DMatrixRMaj(10, 1);
   private final DMatrixRMaj a = new DMatrixRMaj(10, 1);
   private final DMatrixRMaj delta_f = new DMatrixRMaj(10, 1);
   private final DMatrixRMaj delta_a = new DMatrixRMaj(10, 1);

   private final LinearSolverDense<DMatrixRMaj> linearSolver = LinearSolverFactory_DDRM.chol(10);

   private double tolerance = 1.0e-12;

   public LinearComplementarityProblemSolver()
   {
   }

   public DMatrixRMaj solve(DMatrixRMaj A, DMatrixRMaj b)
   {
      int problemSize = checkInputSize(A, b);
      f.reshape(problemSize, 1);
      f.zero();
      a.set(b);
      delta_f.reshape(problemSize, 1);
      delta_a.reshape(problemSize, 1);
      A_CC.reshape(0, 0);
      A_Cd.reshape(0, 1);
      delta_f_C.reshape(0, 1);
      clampedIndexSet.clear();
      notClampedIndexSet.clear();

      while (true)
      {
         int d = -1;
         for (int i = 0; i < problemSize; i++)
         {
            if (a.get(i) < -tolerance)
            {
               d = i;
               break;
            }
         }

         if (d == -1)
            break;

         driveToZeroFrictionless(d, A);
      }

      return f;
   }

   public void driveToZeroFrictionless(int d, DMatrixRMaj A)
   {
      while (true)
      {
         fDirection(d, A);
         CommonOps_DDRM.mult(A, delta_f, delta_a);
         MaxStepResult maxStep = maxStep(d);
         CommonOps_DDRM.addEquals(f, maxStep.scale, delta_f);
         CommonOps_DDRM.addEquals(a, maxStep.scale, delta_a);

         if (clampedIndexSet.remove(maxStep.j))
         {
            notClampedIndexSet.add(maxStep.j);
         }
         else if (notClampedIndexSet.remove(maxStep.j))
         {
            clampedIndexSet.add(maxStep.j);
         }
         else
         {
            clampedIndexSet.add(maxStep.j); // j must be d, implying a_d = 0
            break;
         }
      }
   }

   public void fDirection(int d, DMatrixRMaj A)
   {
      delta_f.zero();

      if (!clampedIndexSet.isEmpty())
      {
         int[] clampedIndices = clampedIndexSet.toArray();

         int numberOfClampedContacts = clampedIndexSet.size();
         A_CC.reshape(numberOfClampedContacts, numberOfClampedContacts);
         A_Cd.reshape(numberOfClampedContacts, 1);
         delta_f_C.reshape(numberOfClampedContacts, 1);

         for (int row = 0; row < numberOfClampedContacts; row++)
         {
            int clampedRow = clampedIndices[row];

            for (int column = 0; column < numberOfClampedContacts; column++)
            {
               int clampedColumn = clampedIndices[column];

               A_CC.set(row, column, A.get(clampedRow, clampedColumn));
            }

            A_Cd.set(row, A.get(clampedRow, d));
         }

         // Solving for A_CC * delta_f_C = - A_Cd
         CommonOps_DDRM.changeSign(A_Cd);

         // TODO Naive approach here, can save some time as mentioned in the paper by using an iterative method.
         linearSolver.setA(A_CC);
         linearSolver.solve(A_Cd, delta_f_C);

         for (int row = 0; row < numberOfClampedContacts; row++)
         {
            int clampedRow = clampedIndices[row];

            delta_f.set(clampedRow, delta_f_C.get(row));
         }
      }

      delta_f.set(d, 0, 1.0); // TODO The test passes whether this is add or set, kinda confused about it.
   }

   public MaxStepResult maxStep(int d)
   {
      double scale = Double.POSITIVE_INFINITY;
      int j = -1;

      if (delta_a.get(d) > 0.0)
      {
         scale = -a.get(d) / delta_a.get(d);
         assert scale > 0.0;
         assert Double.isFinite(scale);
         j = d;
      }

      TIntIterator clampedIndexIterator = clampedIndexSet.iterator();

      while (clampedIndexIterator.hasNext())
      {
         int i = clampedIndexIterator.next();

         if (delta_f.get(i) < 0.0)
         {
            double scale2 = -f.get(i) / delta_f.get(i);
            assert scale2 > 0.0;
            assert Double.isFinite(scale2);

            if (scale2 < scale)
            {
               scale = scale2;
               j = i;
            }
         }
      }

      TIntIterator notClampedIndexIterator = notClampedIndexSet.iterator();

      while (notClampedIndexIterator.hasNext())
      {
         int i = notClampedIndexIterator.next();

         if (delta_a.get(i) < 0.0)
         {
            double scale2 = -a.get(i) / delta_a.get(i);
            assert scale2 > 0.0;
            assert Double.isFinite(scale2);

            if (scale2 < scale)
            {
               scale = scale2;
               j = i;
            }
         }
      }

      return new MaxStepResult(scale, j);
   }

   private static int checkInputSize(DMatrixRMaj A, DMatrixRMaj b)
   {
      int problemSize = b.getNumRows();

      if (b.getNumCols() != 1)
         throw new IllegalArgumentException("b is not a vector, numCols: " + b.getNumCols());
      if (A.getNumCols() != A.getNumRows())
         throw new IllegalArgumentException("A is not square, numRows: " + A.getNumRows() + ", numCols: " + A.getNumCols());
      if (A.getNumRows() != problemSize)
         throw new IllegalArgumentException("Size of A does not match problem size, problem size: " + problemSize + ", size A: " + A.getNumRows());
      return problemSize;
   }

   public static class MaxStepResult
   {
      public final double scale;
      public final int j;

      public MaxStepResult(double scale, int j)
      {
         this.scale = scale;
         this.j = j;
      }
   }
}
