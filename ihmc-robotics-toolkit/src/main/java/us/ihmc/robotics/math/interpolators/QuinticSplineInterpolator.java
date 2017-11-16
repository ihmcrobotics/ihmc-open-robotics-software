package us.ihmc.robotics.math.interpolators;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class QuinticSplineInterpolator
{
   private final int pointsToInterpolate;
   private final int numberOfSplines;

   private final YoVariableRegistry registry;
   private final YoBoolean initialized;

   // Matrices used internally to calculate constants
   private DenseMatrix64F h;
   private DenseMatrix64F D;
   private DenseMatrix64F C;
   private DenseMatrix64F Cblock;
   private DenseMatrix64F A;
   private final LinearSolver<DenseMatrix64F> solver;

   private DenseMatrix64F sol;
   private DenseMatrix64F s;
   private DenseMatrix64F yd;

   private DenseMatrix64F a;
   private DenseMatrix64F b;
   private DenseMatrix64F c;
   private DenseMatrix64F d;
   private DenseMatrix64F e;
   private DenseMatrix64F f;

   // Constants
   private final YoDouble[] x;
   private final QuinticSpline[] splines;

   /**
    * Create a new QuinticSplineInterpolator
    * 
    * @param name Name of the YoVariableRegistry
    * @param pointsToInterpolate Number of points the spline has to pass (>= 2)
    * @param numberOfSplines Number of splines it creates that have the same x coordinates for the points to interpolate
    */
   public QuinticSplineInterpolator(String name, int pointsToInterpolate, int numberOfSplines, YoVariableRegistry parentRegistry)
   {
      this.pointsToInterpolate = pointsToInterpolate;
      this.numberOfSplines = numberOfSplines;

      registry = new YoVariableRegistry(name);
      initialized = new YoBoolean("initialized", registry);
      initialized.set(false);

      h = new DenseMatrix64F(pointsToInterpolate - 1, 1);
      D = new DenseMatrix64F(pointsToInterpolate - 1, pointsToInterpolate + 2);
      C = new DenseMatrix64F(pointsToInterpolate, pointsToInterpolate + 2);
      Cblock = new DenseMatrix64F(pointsToInterpolate - 1, pointsToInterpolate + 2);
      A = new DenseMatrix64F(pointsToInterpolate + 2, pointsToInterpolate + 2);
      solver = LinearSolverFactory.linear(pointsToInterpolate + 2);

      s = new DenseMatrix64F(pointsToInterpolate + 2, 1);
      sol = new DenseMatrix64F(pointsToInterpolate + 2, 1);
      yd = new DenseMatrix64F(pointsToInterpolate - 1, 1);

      a = new DenseMatrix64F(pointsToInterpolate - 1, 1);
      b = new DenseMatrix64F(pointsToInterpolate - 1, 1);
      c = new DenseMatrix64F(pointsToInterpolate - 1, 1);
      d = new DenseMatrix64F(pointsToInterpolate - 1, 1);
      e = new DenseMatrix64F(pointsToInterpolate - 1, 1);
      f = new DenseMatrix64F(pointsToInterpolate - 1, 1);

      x = new YoDouble[pointsToInterpolate];
      for (int i = 0; i < pointsToInterpolate; i++)
      {
         x[i] = new YoDouble("x[" + i + "]", registry);
      }

      splines = new QuinticSpline[numberOfSplines];
      for (int i = 0; i < numberOfSplines; i++)
      {
         splines[i] = new QuinticSpline("spline[" + i + "]", pointsToInterpolate, registry);
      }

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

   }

   /**
    * Initializes the spline
    * @param xIn Array (length = pointsToInterpolate) with the x coordinates of the points to interpolate
    */
   public void initialize(double[] xIn)
   {
      if (xIn.length != pointsToInterpolate)
         throw new RuntimeException("xIn is not the correct lenght");

      for (int i = 0; i < pointsToInterpolate; i++)
      {
         x[i].set(xIn[i]);
      }

      MatrixTools.diff(xIn, h);

      // Build D DenseMatrix64F
      MatrixTools.setToZero(D);
      D.unsafe_set(0, 1, 1.0);
      for (int i = 1; i < pointsToInterpolate - 1; i++)
      {
         D.unsafe_set(i, i + 1, 2.0 * h.unsafe_get(i - 1, 0));
         D.unsafe_set(i, i + 2, 2.0 * h.unsafe_get(i - 1, 0));
         MatrixTools.addMatrixBlock(D, i, 0, D, i - 1, 0, 1, pointsToInterpolate + 2, 1.0);
      }

      // Build C DenseMatrix64F
      MatrixTools.setToZero(C);
      C.unsafe_set(0, 0, 1.0);
      for (int i = 1; i < pointsToInterpolate; i++)
      {
         C.unsafe_set(i, i + 1, 4.0 * MathTools.square(h.unsafe_get(i - 1, 0)));
         C.unsafe_set(i, i + 2, 2.0 * MathTools.square(h.unsafe_get(i - 1, 0)));
         MatrixTools.addMatrixBlock(C, i, 0, C, i - 1, 0, 1, pointsToInterpolate + 2, 1.0);
         MatrixTools.addMatrixBlock(C, i, 0, D, i - 1, 0, 1, pointsToInterpolate + 2, 3.0 * h.unsafe_get(i - 1, 0));
      }
      MatrixTools.setMatrixBlock(Cblock, 0, 0, C, 0, 0, pointsToInterpolate - 1, pointsToInterpolate + 2, 1.0);

      // Build A DenseMatrix64F
      MatrixTools.setToZero(A);
      for (int i = 0; i < pointsToInterpolate - 2; i++)
      {
         A.unsafe_set(i + 4, i + 2, 11.0 / 5.0 * MathTools.cube(h.unsafe_get(i, 0)));
         A.unsafe_set(i + 4, i + 3, 4.0 / 5.0 * MathTools.cube(h.unsafe_get(i, 0)) + 4.0 / 5.0 * MathTools.cube(h.unsafe_get(i + 1, 0)));
         A.unsafe_set(i + 4, i + 4, 1.0 / 5.0 * MathTools.cube(h.unsafe_get(i + 1, 0)));
         MatrixTools.addMatrixBlock(A, i + 4, 0, C, i, 0, 1, pointsToInterpolate + 2, h.unsafe_get(i, 0));
         MatrixTools.addMatrixBlock(A, i + 4, 0, C, i + 1, 0, 1, pointsToInterpolate + 2, h.unsafe_get(i + 1, 0));
         MatrixTools.addMatrixBlock(A, i + 4, 0, D, i, 0, 1, pointsToInterpolate + 2, 2.0 * MathTools.square(h.unsafe_get(i, 0)));
         MatrixTools.addMatrixBlock(A, i + 4, 0, D, i + 1, 0, 1, pointsToInterpolate + 2, MathTools.square(h.unsafe_get(i + 1, 0)));
      }

      // Add boundary conditions
      A.unsafe_set(0, 0, h.unsafe_get(0, 0));
      A.unsafe_set(0, 1, MathTools.square(h.unsafe_get(0, 0)));
      A.unsafe_set(0, 2, 4.0 / 5.0 * MathTools.cube(h.unsafe_get(0, 0)));
      A.unsafe_set(0, 3, 1.0 / 5.0 * MathTools.cube(h.unsafe_get(0, 0)));

      A.unsafe_set(1, 0, 2.0);

      A.unsafe_set(2, pointsToInterpolate, 11.0 / 5.0 * MathTools.cube(h.unsafe_get(pointsToInterpolate - 2, 0)));
      A.unsafe_set(2, pointsToInterpolate + 1, 4.0 / 5.0 * MathTools.cube(h.unsafe_get(pointsToInterpolate - 2, 0)));

      MatrixTools.addMatrixBlock(A, 2, 0, C, pointsToInterpolate - 2, 0, 1, pointsToInterpolate + 2, h.unsafe_get(pointsToInterpolate - 2, 0));
      MatrixTools.addMatrixBlock(A, 2, 0, D, pointsToInterpolate - 2, 0, 1, pointsToInterpolate + 2,
            2.0 * MathTools.square(h.unsafe_get(pointsToInterpolate - 2, 0)));

      MatrixTools.addMatrixBlock(A, 3, 0, C, pointsToInterpolate - 1, 0, 1, pointsToInterpolate + 2, 2.0);

      if (!solver.setA(A))
      {
         throw new IllegalArgumentException("Singular matrix");
      }
      initialized.set(true);

      for (int i = 0; i < numberOfSplines; i++)
      {
         splines[i].setCoefficientsSet(false);
      }

   }

   /**
    * 
    * Determines the coefficients for one spline
    * 
    * @param splineIndex Index for the spline ( 0 <= splineIndex < pointsToInterpolate)
    * @param yIn Array (length = pointsToInterpolate) with the y coordinates of the points to interpolate
    * @param v0 Initial velocity
    * @param vf Final velocity
    * @param a0 Initial acceleration
    * @param af Final acceleration
    */
   public void determineCoefficients(int splineIndex, double[] yIn, double v0, double vf, double a0, double af)
   {
      if (!initialized.getBooleanValue())
         throw new RuntimeException("QuinticSplineInterpolator is not initialized");

      if (splineIndex > numberOfSplines - 1 || splineIndex < 0)
         throw new RuntimeException("SplineIndex is out of bounds");

      if (yIn.length != pointsToInterpolate)
         throw new RuntimeException("y should have as many elements as points to interpolate");

      MatrixTools.setMatrixColumnFromArray(a, 0, yIn);

      MatrixTools.diff(yIn, yd);

      if (pointsToInterpolate > 2)
      {
         s.unsafe_set(0, 0, yIn[1] / h.unsafe_get(0, 0) - yIn[0] / h.unsafe_get(1, 0) - v0);
         for (int i = 0; i < pointsToInterpolate - 2; i++)
         {
            s.unsafe_set(i + 4, 0, yd.unsafe_get(i + 1, 0) / h.unsafe_get(i + 1, 0) - yd.unsafe_get(i, 0) / h.unsafe_get(i, 0));
         }
      }
      else
      {
         s.unsafe_set(0, 0, yIn[1] / h.unsafe_get(0, 0) - yIn[0] / h.unsafe_get(0, 0) - v0);
      }

      s.unsafe_set(1, 0, a0);
      s.unsafe_set(
            2,
            0,
            vf - yIn[pointsToInterpolate - 1] / h.unsafe_get(pointsToInterpolate - 2, 0) + yIn[pointsToInterpolate - 2]
                  / h.unsafe_get(pointsToInterpolate - 2, 0));
      s.unsafe_set(3, 0, af);

      /*
       * TODO: Rewrite so no new objects are created
       */

      solver.solve(s, sol);

      CommonOps.mult(Cblock, sol, c);
      CommonOps.mult(D, sol, d);

      MatrixTools.setMatrixBlock(e, 0, 0, sol, 2, 0, pointsToInterpolate - 1, 1, 1.0);

      MatrixTools.diff(sol, 2, pointsToInterpolate, f);
      CommonOps.scale(1.0 / 5.0, f);
      CommonOps.elementDiv(f, h);

      for (int i = 0; i < pointsToInterpolate - 1; i++)
      {
         double hi = h.unsafe_get(i, 0);
         double hi2 = MathTools.square(hi);
         double hi3 = hi2 * hi;
         double hi4 = hi3 * hi;

         b.unsafe_set(i, 0, yd.unsafe_get(i, 0) / hi - c.unsafe_get(i, 0) * hi - d.unsafe_get(i, 0) * hi2 - e.unsafe_get(i, 0) * hi3 - f.unsafe_get(i, 0) * hi4);
      }

      splines[splineIndex].seta(a);
      splines[splineIndex].setb(b);
      splines[splineIndex].setc(c);
      splines[splineIndex].setd(d);
      splines[splineIndex].sete(e);
      splines[splineIndex].setf(f);

      splines[splineIndex].setCoefficientsSet(true);

   }

   /**
    * Calculates the y coordinate of the spline corresponding to the x coordinate
    * 
    * resultToPack is a 2-dimensional array with 
    *    resultToPack[spline][derivative], where the 0th derivative is the position, 1st velocity etc...
    *   
    * @param xx x coordinate to calculate 
    * @param numberOfDerivatives Number of derivatives to calculate ( <= 5 )
    * @param resultToPack array[numberOfSplines][numberOfDerivatives+1] 
    */
   public void compute(double xx, int numberOfDerivatives, double[/*
                                                                   * spline
                                                                   * index
                                                                   */][/* derivative */] resultToPack)
   {
      if (numberOfDerivatives > 5)
         throw new RuntimeException("A quintic spline has only 5 derivatives");
      int expectedColumns = numberOfDerivatives + 1;

      if ((resultToPack.length != numberOfSplines) || (resultToPack[0].length != expectedColumns))
      {
         String message = "Matrix dimensions are (" + resultToPack.length + ", " + resultToPack[0].length + "), expected (" + numberOfSplines + ","
                          + expectedColumns + ")";
      
         throw new RuntimeException(message);
      }

      if (xx > x[pointsToInterpolate - 1].getDoubleValue())
         xx = x[pointsToInterpolate - 1].getDoubleValue();
      if (xx < x[0].getDoubleValue())
         xx = x[0].getDoubleValue();

      int index = determineSplineIndex(xx);

      double h = xx - x[index].getDoubleValue();

      double h2 = MathTools.square(h);
      double h3 = h2 * h;
      double h4 = h3 * h;
      double h5 = h4 * h;

      for (int i = 0; i < numberOfSplines; i++)
      {
         splines[i].value(index, h, h2, h3, h4, h5, numberOfDerivatives, resultToPack[i]);
      }

   }

   private int determineSplineIndex(double xx)
   {
      for (int i = 0; i < pointsToInterpolate - 2; i++)
      {
         if (xx >= x[i].getDoubleValue() && xx <= x[i + 1].getDoubleValue())
            return i;
      }
      return pointsToInterpolate - 2;
   }

   /*
    * Storage for quintic spline coefficients
    */
   private class QuinticSpline
   {
      private final int segments;
      private final YoVariableRegistry registry;

      private final YoDouble[] a;
      private final YoDouble[] b;
      private final YoDouble[] c;
      private final YoDouble[] d;
      private final YoDouble[] e;
      private final YoDouble[] f;

      private final YoBoolean coefficientsSet;

      public QuinticSpline(String name, int pointsToInterpolate, YoVariableRegistry parentRegistry)
      {
         this.segments = pointsToInterpolate - 1;
         this.registry = new YoVariableRegistry(name);
         parentRegistry.addChild(this.registry);
         a = new YoDouble[segments];
         b = new YoDouble[segments];
         c = new YoDouble[segments];
         d = new YoDouble[segments];
         e = new YoDouble[segments];
         f = new YoDouble[segments];

         coefficientsSet = new YoBoolean("initialized", registry);
         coefficientsSet.set(false);

         for (int i = 0; i < segments; i++)
         {
            a[i] = new YoDouble("a[" + i + "]", registry);
            b[i] = new YoDouble("b[" + i + "]", registry);
            c[i] = new YoDouble("c[" + i + "]", registry);
            d[i] = new YoDouble("d[" + i + "]", registry);
            e[i] = new YoDouble("e[" + i + "]", registry);
            f[i] = new YoDouble("f[" + i + "]", registry);
         }
      }

      private void set(YoDouble[] var, DenseMatrix64F value)
      {
         for (int i = 0; i < segments; i++)
         {
            var[i].set(value.unsafe_get(i, 0));
         }
      }

      protected void seta(DenseMatrix64F value)
      {
         set(a, value);
      }

      protected void setb(DenseMatrix64F value)
      {
         set(b, value);
      }

      protected void setc(DenseMatrix64F value)
      {
         set(c, value);
      }

      protected void setd(DenseMatrix64F value)
      {
         set(d, value);
      }

      protected void sete(DenseMatrix64F value)
      {
         set(e, value);
      }

      protected void setf(DenseMatrix64F value)
      {
         set(f, value);
      }

      protected void setCoefficientsSet(boolean coefficientsSet)
      {
         this.coefficientsSet.set(coefficientsSet);
      }

      protected void value(int index, double h, double h2, double h3, double h4, double h5, int numberOfDerivatives, double[] resultToPack)
      {
         if (!coefficientsSet.getBooleanValue())
            throw new RuntimeException("Spline coefficients not set");

         resultToPack[0] = a[index].getDoubleValue() + b[index].getDoubleValue() * h + c[index].getDoubleValue() * h2 + d[index].getDoubleValue() * h3
               + e[index].getDoubleValue() * h4 + f[index].getDoubleValue() * h5;

         switch (numberOfDerivatives)
         {
            case 5:
               resultToPack[5] = 120.0 * f[index].getDoubleValue();
            case 4:
               resultToPack[4] = 24.0 * e[index].getDoubleValue() + 120.0 * f[index].getDoubleValue() * h;
            case 3:
               resultToPack[3] = 6.0 * d[index].getDoubleValue() + 24.0 * e[index].getDoubleValue() * h + 60 * f[index].getDoubleValue() * h2;
            case 2:
               resultToPack[2] = 2.0 * c[index].getDoubleValue() + 6.0 * d[index].getDoubleValue() * h + 12.0 * e[index].getDoubleValue() * h2 + 20.0
                     * f[index].getDoubleValue() * h3;
            case 1:
               resultToPack[1] = b[index].getDoubleValue() + 2.0 * c[index].getDoubleValue() * h + 3.0 * d[index].getDoubleValue() * h2 + 4.0
                     * e[index].getDoubleValue() * h3 + 5.0 * f[index].getDoubleValue() * h4;
         }

      }

   }

}
