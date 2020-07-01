package us.ihmc.robotics.optimization;

import org.ejml.data.DMatrixRMaj;

/**
 * minimize
 *    || A * x - b||
 * subject to
 *    C * x <= d
 */
public class QuadraticProgram
{
   private final DMatrixRMaj a;
   private final DMatrixRMaj b;
   private final DMatrixRMaj c;
   private final DMatrixRMaj d;

   public QuadraticProgram(DMatrixRMaj a, DMatrixRMaj b, DMatrixRMaj c, DMatrixRMaj d)
   {
      doSizeChecks(a, b, c, d);

      this.a = a;
      this.b = b;
      this.c = c;
      this.d = d;
   }

   private void doSizeChecks(DMatrixRMaj a, DMatrixRMaj b, DMatrixRMaj c, DMatrixRMaj d)
   {
      if (a.getNumRows() != b.getNumRows())
         throw new IllegalArgumentException("a.getNumRows() != b.getNumRows()");
      if (c.getNumRows() != d.getNumRows())
         throw new IllegalArgumentException("c.getNumRows() != d.getNumRows()");
      if (a.getNumCols() != c.getNumCols())
         throw new IllegalArgumentException("a.getNumCols() != c.getNumCols()");
   }

   public DMatrixRMaj getA()
   {
      return a;
   }

   public DMatrixRMaj getB()
   {
      return b;
   }

   public DMatrixRMaj getC()
   {
      return c;
   }

   public DMatrixRMaj getD()
   {
      return d;
   }

   public int getSolutionSize()
   {
      return a.getNumCols();
   }

   public int getObjectiveSize()
   {
      return b.getNumRows();
   }

   public int getInequalityConstraintSize()
   {
      return d.getNumRows();
   }
}
