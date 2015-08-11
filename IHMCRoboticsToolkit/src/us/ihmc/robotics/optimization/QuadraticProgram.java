package us.ihmc.robotics.optimization;

import org.ejml.data.DenseMatrix64F;

/**
 * minimize
 *    || A * x - b||
 * subject to
 *    C * x <= d
 */
public class QuadraticProgram
{
   private final DenseMatrix64F a;
   private final DenseMatrix64F b;
   private final DenseMatrix64F c;
   private final DenseMatrix64F d;

   public QuadraticProgram(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c, DenseMatrix64F d)
   {
      doSizeChecks(a, b, c, d);

      this.a = a;
      this.b = b;
      this.c = c;
      this.d = d;
   }

   private void doSizeChecks(DenseMatrix64F a, DenseMatrix64F b, DenseMatrix64F c, DenseMatrix64F d)
   {
      if (a.getNumRows() != b.getNumRows())
         throw new IllegalArgumentException("a.getNumRows() != b.getNumRows()");
      if (c.getNumRows() != d.getNumRows())
         throw new IllegalArgumentException("c.getNumRows() != d.getNumRows()");
      if (a.getNumCols() != c.getNumCols())
         throw new IllegalArgumentException("a.getNumCols() != c.getNumCols()");
   }

   public DenseMatrix64F getA()
   {
      return a;
   }

   public DenseMatrix64F getB()
   {
      return b;
   }

   public DenseMatrix64F getC()
   {
      return c;
   }

   public DenseMatrix64F getD()
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
