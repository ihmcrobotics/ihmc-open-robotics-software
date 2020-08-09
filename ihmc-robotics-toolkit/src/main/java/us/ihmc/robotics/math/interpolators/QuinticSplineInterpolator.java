package us.ihmc.robotics.math.interpolators;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.commons.MathTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.math.trajectories.TrajectoryGenerator;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * Quintic spline interpolator This class calculates a spline through multiple waypoints, minimizing
 * the jerk and a finite 5th derivative of position. Not realtime safe. Not rewindable. It will
 * result in too many intermediate variables.
 * 
 * @author Jesper Smith
 */
public class QuinticSplineInterpolator implements TrajectoryGenerator
{
   private final int maximumNumberOfPoints;
   private final int numberOfSplines;

   private final YoRegistry registry;
   private final YoBoolean initialized;
   private final YoDouble currentTime;

   // Matrices used internally to calculate constants
   private DMatrixRMaj h;
   private DMatrixRMaj D;
   private DMatrixRMaj C;
   private DMatrixRMaj Cblock;
   private DMatrixRMaj A;
   private final LinearSolverDense<DMatrixRMaj> solver;

   private DMatrixRMaj sol;
   private DMatrixRMaj s;
   private DMatrixRMaj yd;

   private DMatrixRMaj a;
   private DMatrixRMaj b;
   private DMatrixRMaj c;
   private DMatrixRMaj d;
   private DMatrixRMaj e;
   private DMatrixRMaj f;

   // Constants
   private final double[] x;
   private final YoInteger numberOfPoints;
   private final QuinticSpline[] splines;

   // Result

   private final YoDouble[] position;
   private final YoDouble[] velocity;
   private final YoDouble[] acceleration;
   private final YoDouble[] jerk;

   /**
    * Create a new QuinticSplineInterpolator
    * 
    * @param name                  Name of the YoRegistry
    * @param maximumNumberOfPoints Maximum number of points the spline can interpolate (>= 2)
    * @param numberOfSplines       Number of splines it creates that have the same x coordinates for
    *                              the points to interpolate. This allows re-use of memory and saves
    *                              some computational time.
    */
   public QuinticSplineInterpolator(String name, int maximumNumberOfPoints, int numberOfSplines, YoRegistry parentRegistry)
   {
      this.maximumNumberOfPoints = maximumNumberOfPoints;
      this.numberOfSplines = numberOfSplines;

      registry = new YoRegistry(name);
      initialized = new YoBoolean(name + "_initialized", registry);
      initialized.set(false);
      numberOfPoints = new YoInteger(name + "_numberOfPoints", registry);
      currentTime = new YoDouble(name + "_currentTime", registry);

      h = new DMatrixRMaj(maximumNumberOfPoints - 1, 1);
      D = new DMatrixRMaj(maximumNumberOfPoints - 1, maximumNumberOfPoints + 2);
      C = new DMatrixRMaj(maximumNumberOfPoints, maximumNumberOfPoints + 2);
      Cblock = new DMatrixRMaj(maximumNumberOfPoints - 1, maximumNumberOfPoints + 2);
      A = new DMatrixRMaj(maximumNumberOfPoints + 2, maximumNumberOfPoints + 2);
      solver = LinearSolverFactory_DDRM.linear(maximumNumberOfPoints + 2);

      s = new DMatrixRMaj(maximumNumberOfPoints + 2, 1);
      sol = new DMatrixRMaj(maximumNumberOfPoints + 2, 1);
      yd = new DMatrixRMaj(maximumNumberOfPoints - 1, 1);

      a = new DMatrixRMaj(maximumNumberOfPoints - 1, 1);
      b = new DMatrixRMaj(maximumNumberOfPoints - 1, 1);
      c = new DMatrixRMaj(maximumNumberOfPoints - 1, 1);
      d = new DMatrixRMaj(maximumNumberOfPoints - 1, 1);
      e = new DMatrixRMaj(maximumNumberOfPoints - 1, 1);
      f = new DMatrixRMaj(maximumNumberOfPoints - 1, 1);

      x = new double[maximumNumberOfPoints];

      position = new YoDouble[numberOfSplines];
      velocity = new YoDouble[numberOfSplines];
      acceleration = new YoDouble[numberOfSplines];
      jerk = new YoDouble[numberOfSplines];

      splines = new QuinticSpline[numberOfSplines];
      for (int i = 0; i < numberOfSplines; i++)
      {
         String eName = name + "-" + i;
         splines[i] = new QuinticSpline(eName, maximumNumberOfPoints, registry);

         position[i] = new YoDouble(eName + "_position", registry);
         velocity[i] = new YoDouble(eName + "_velocity", registry);
         acceleration[i] = new YoDouble(eName + "_acceleration", registry);
         jerk[i] = new YoDouble(eName + "_jerk", registry);
      }

      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }

   }

   public int getMaximumNumberOfWaypoints()
   {
      return this.maximumNumberOfPoints;
   }

   /**
    * Initializes the spline.
    * 
    * @param numberOfPointsIn Number of waypoints for this trajectory.
    * @param timeIn           Array (length >= numberOfPointsIn) with the time for the points to
    *                         interpolate. If the array is longer than numberOfPointsIn, points past
    *                         numberOfPointsIn are ignored
    */
   public void initialize(int numberOfPointsIn, double[] timeIn)
   {
      if (timeIn.length > maximumNumberOfPoints)
         throw new RuntimeException("xIn exceeds the maximum number of points");

      numberOfPoints.set(numberOfPointsIn);

      for (int i = 0; i < numberOfPoints.getValue(); i++)
      {
         x[i] = timeIn[i];
      }

      h.reshape(numberOfPoints.getValue() - 1, 1);
      D.reshape(numberOfPoints.getValue() - 1, numberOfPoints.getValue() + 2);
      C.reshape(numberOfPoints.getValue(), numberOfPoints.getValue() + 2);
      Cblock.reshape(numberOfPoints.getValue() - 1, numberOfPoints.getValue() + 2);
      A.reshape(numberOfPoints.getValue() + 2, numberOfPoints.getValue() + 2);

      s.reshape(numberOfPoints.getValue() + 2, 1);
      sol.reshape(numberOfPoints.getValue() + 2, 1);
      yd.reshape(numberOfPoints.getValue() - 1, 1);

      a.reshape(numberOfPoints.getValue() - 1, 1);
      b.reshape(numberOfPoints.getValue() - 1, 1);
      c.reshape(numberOfPoints.getValue() - 1, 1);
      d.reshape(numberOfPoints.getValue() - 1, 1);
      e.reshape(numberOfPoints.getValue() - 1, 1);
      f.reshape(numberOfPoints.getValue() - 1, 1);

      MatrixTools.diff(timeIn, h);

      // Build D DMatrixRMaj
      MatrixTools.setToZero(D);
      D.unsafe_set(0, 1, 1.0);
      for (int i = 1; i < numberOfPoints.getValue() - 1; i++)
      {
         D.unsafe_set(i, i + 1, 2.0 * h.unsafe_get(i - 1, 0));
         D.unsafe_set(i, i + 2, 2.0 * h.unsafe_get(i - 1, 0));
         MatrixTools.addMatrixBlock(D, i, 0, D, i - 1, 0, 1, numberOfPoints.getValue() + 2, 1.0);
      }

      // Build C DMatrixRMaj
      MatrixTools.setToZero(C);
      C.unsafe_set(0, 0, 1.0);
      for (int i = 1; i < numberOfPoints.getValue(); i++)
      {
         C.unsafe_set(i, i + 1, 4.0 * MathTools.square(h.unsafe_get(i - 1, 0)));
         C.unsafe_set(i, i + 2, 2.0 * MathTools.square(h.unsafe_get(i - 1, 0)));
         MatrixTools.addMatrixBlock(C, i, 0, C, i - 1, 0, 1, numberOfPoints.getValue() + 2, 1.0);
         MatrixTools.addMatrixBlock(C, i, 0, D, i - 1, 0, 1, numberOfPoints.getValue() + 2, 3.0 * h.unsafe_get(i - 1, 0));
      }
      MatrixTools.setMatrixBlock(Cblock, 0, 0, C, 0, 0, numberOfPoints.getValue() - 1, numberOfPoints.getValue() + 2, 1.0);

      // Build A DMatrixRMaj
      MatrixTools.setToZero(A);
      for (int i = 0; i < numberOfPoints.getValue() - 2; i++)
      {
         A.unsafe_set(i + 4, i + 2, 11.0 / 5.0 * MathTools.cube(h.unsafe_get(i, 0)));
         A.unsafe_set(i + 4, i + 3, 4.0 / 5.0 * MathTools.cube(h.unsafe_get(i, 0)) + 4.0 / 5.0 * MathTools.cube(h.unsafe_get(i + 1, 0)));
         A.unsafe_set(i + 4, i + 4, 1.0 / 5.0 * MathTools.cube(h.unsafe_get(i + 1, 0)));
         MatrixTools.addMatrixBlock(A, i + 4, 0, C, i, 0, 1, numberOfPoints.getValue() + 2, h.unsafe_get(i, 0));
         MatrixTools.addMatrixBlock(A, i + 4, 0, C, i + 1, 0, 1, numberOfPoints.getValue() + 2, h.unsafe_get(i + 1, 0));
         MatrixTools.addMatrixBlock(A, i + 4, 0, D, i, 0, 1, numberOfPoints.getValue() + 2, 2.0 * MathTools.square(h.unsafe_get(i, 0)));
         MatrixTools.addMatrixBlock(A, i + 4, 0, D, i + 1, 0, 1, numberOfPoints.getValue() + 2, MathTools.square(h.unsafe_get(i + 1, 0)));
      }

      // Add boundary conditions
      A.unsafe_set(0, 0, h.unsafe_get(0, 0));
      A.unsafe_set(0, 1, MathTools.square(h.unsafe_get(0, 0)));
      A.unsafe_set(0, 2, 4.0 / 5.0 * MathTools.cube(h.unsafe_get(0, 0)));
      A.unsafe_set(0, 3, 1.0 / 5.0 * MathTools.cube(h.unsafe_get(0, 0)));

      A.unsafe_set(1, 0, 2.0);

      A.unsafe_set(2, numberOfPoints.getValue(), 11.0 / 5.0 * MathTools.cube(h.unsafe_get(numberOfPoints.getValue() - 2, 0)));
      A.unsafe_set(2, numberOfPoints.getValue() + 1, 4.0 / 5.0 * MathTools.cube(h.unsafe_get(numberOfPoints.getValue() - 2, 0)));

      MatrixTools.addMatrixBlock(A,
                                 2,
                                 0,
                                 C,
                                 numberOfPoints.getValue() - 2,
                                 0,
                                 1,
                                 numberOfPoints.getValue() + 2,
                                 h.unsafe_get(numberOfPoints.getValue() - 2, 0));
      MatrixTools.addMatrixBlock(A,
                                 2,
                                 0,
                                 D,
                                 numberOfPoints.getValue() - 2,
                                 0,
                                 1,
                                 numberOfPoints.getValue() + 2,
                                 2.0 * MathTools.square(h.unsafe_get(numberOfPoints.getValue() - 2, 0)));

      MatrixTools.addMatrixBlock(A, 3, 0, C, numberOfPoints.getValue() - 1, 0, 1, numberOfPoints.getValue() + 2, 2.0);

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
    * Determines the coefficients for one spline
    * 
    * @param splineIndex Index for the spline ( 0 <= splineIndex < pointsToInterpolate)
    * @param positionIn  Array (length = pointsToInterpolate) with the positions of the points to
    *                    interpolate
    * @param v0          Initial velocity
    * @param vf          Final velocity
    * @param a0          Initial acceleration
    * @param af          Final acceleration
    */
   public void determineCoefficients(int splineIndex, double[] positionIn, double v0, double vf, double a0, double af)
   {
      if (!initialized.getBooleanValue())
         throw new RuntimeException("QuinticSplineInterpolator is not initialized");

      if (splineIndex > numberOfSplines - 1 || splineIndex < 0)
         throw new RuntimeException("SplineIndex is out of bounds");

      if (positionIn.length < numberOfPoints.getValue())
         throw new RuntimeException("Length of positionIn is less than the number of points");

      MatrixTools.setMatrixColumnFromArray(a, 0, positionIn, 0, a.getNumRows());

      MatrixTools.diff(positionIn, yd);

      if (numberOfPoints.getValue() > 2)
      {
         s.unsafe_set(0, 0, positionIn[1] / h.unsafe_get(0, 0) - positionIn[0] / h.unsafe_get(0, 0) - v0);
         for (int i = 0; i < numberOfPoints.getValue() - 2; i++)
         {
            s.unsafe_set(i + 4, 0, yd.unsafe_get(i + 1, 0) / h.unsafe_get(i + 1, 0) - yd.unsafe_get(i, 0) / h.unsafe_get(i, 0));
         }
      }
      else
      {
         s.unsafe_set(0, 0, positionIn[1] / h.unsafe_get(0, 0) - positionIn[0] / h.unsafe_get(0, 0) - v0);
      }

      s.unsafe_set(1, 0, a0);
      s.unsafe_set(2,
                   0,
                   vf - positionIn[numberOfPoints.getValue() - 1] / h.unsafe_get(numberOfPoints.getValue() - 2, 0)
                         + positionIn[numberOfPoints.getValue() - 2] / h.unsafe_get(numberOfPoints.getValue() - 2, 0));
      s.unsafe_set(3, 0, af);

      /*
       * TODO: Rewrite so no new objects are created
       */

      solver.solve(s, sol);

      CommonOps_DDRM.mult(Cblock, sol, c);
      CommonOps_DDRM.mult(D, sol, d);

      MatrixTools.setMatrixBlock(e, 0, 0, sol, 2, 0, numberOfPoints.getValue() - 1, 1, 1.0);

      MatrixTools.diff(sol, 2, numberOfPoints.getValue(), f);
      CommonOps_DDRM.scale(1.0 / 5.0, f);
      CommonOps_DDRM.elementDiv(f, h);

      for (int i = 0; i < numberOfPoints.getValue() - 1; i++)
      {
         double hi = h.unsafe_get(i, 0);
         double hi2 = MathTools.square(hi);
         double hi3 = hi2 * hi;
         double hi4 = hi3 * hi;

         b.unsafe_set(i,
                      0,
                      yd.unsafe_get(i, 0) / hi - c.unsafe_get(i, 0) * hi - d.unsafe_get(i, 0) * hi2 - e.unsafe_get(i, 0) * hi3 - f.unsafe_get(i, 0) * hi4);
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
    * Calculates the y coordinate of the spline corresponding to the x coordinate After calling
    * compute, the resulting position, velocity, acceleration and jerk can be queried using
    * getPosition(spline), getVelocity(spline), getAcceleration(spline) and getJerk(spline)
    * respectively
    * 
    * @param time x coordinate to calculate
    */
   public void compute(double timeIn)
   {
      this.currentTime.set(timeIn);

      double time = timeIn;
      if (time > x[numberOfPoints.getValue() - 1])
         time = x[numberOfPoints.getValue() - 1];
      if (time < x[0])
         time = x[0];

      int index = determineSplineIndex(time);

      double h = time - x[index];

      double h2 = MathTools.square(h);
      double h3 = h2 * h;
      double h4 = h3 * h;
      double h5 = h4 * h;

      for (int i = 0; i < numberOfSplines; i++)
      {
         splines[i].value(index, h, h2, h3, h4, h5, position[i], velocity[i], acceleration[i], jerk[i], null, null);
      }

   }

   public double getPosition(int spline)
   {
      return position[spline].getValue();
   }

   public double getVelocity(int spline)
   {
      return velocity[spline].getValue();
   }

   public double getAcceleration(int spline)
   {
      return acceleration[spline].getValue();
   }

   public double getJerk(int spline)
   {
      return jerk[spline].getValue();
   }

   private int determineSplineIndex(double xx)
   {
      for (int i = 0; i < numberOfPoints.getValue() - 2; i++)
      {
         if (xx >= x[i] && xx <= x[i + 1])
            return i;
      }
      return numberOfPoints.getValue() - 2;
   }

   /*
    * Storage for quintic spline coefficients
    */
   private class QuinticSpline
   {
      private final int segments;
      private final YoRegistry registry;

      private final double[] a;
      private final double[] b;
      private final double[] c;
      private final double[] d;
      private final double[] e;
      private final double[] f;

      private final YoBoolean coefficientsSet;

      public QuinticSpline(String name, int pointsToInterpolate, YoRegistry parentRegistry)
      {
         this.segments = pointsToInterpolate - 1;
         this.registry = new YoRegistry(name);
         parentRegistry.addChild(this.registry);
         a = new double[segments];
         b = new double[segments];
         c = new double[segments];
         d = new double[segments];
         e = new double[segments];
         f = new double[segments];

         coefficientsSet = new YoBoolean(name + "_initialized", registry);
         coefficientsSet.set(false);
      }

      private void set(double[] var, DMatrixRMaj value)
      {
         for (int i = 0; i < segments; i++)
         {
            var[i] = value.unsafe_get(i, 0);
         }
      }

      protected void seta(DMatrixRMaj value)
      {
         set(a, value);
      }

      protected void setb(DMatrixRMaj value)
      {
         set(b, value);
      }

      protected void setc(DMatrixRMaj value)
      {
         set(c, value);
      }

      protected void setd(DMatrixRMaj value)
      {
         set(d, value);
      }

      protected void sete(DMatrixRMaj value)
      {
         set(e, value);
      }

      protected void setf(DMatrixRMaj value)
      {
         set(f, value);
      }

      protected void setCoefficientsSet(boolean coefficientsSet)
      {
         this.coefficientsSet.set(coefficientsSet);
      }

      protected void value(int index, double h, double h2, double h3, double h4, double h5, YoDouble pos, YoDouble vel, YoDouble acc, YoDouble jerk,
                           YoDouble snap, YoDouble crackle)
      {
         if (!coefficientsSet.getBooleanValue())
            throw new RuntimeException("Spline coefficients not set");

         pos.set(a[index] + b[index] * h + c[index] * h2 + d[index] * h3 + e[index] * h4 + f[index] * h5);

         if (crackle != null)
         {
            crackle.set(120.0 * f[index]);
         }
         if (snap != null)
         {
            snap.set(24.0 * e[index] + 120.0 * f[index] * h);
         }
         jerk.set(6.0 * d[index] + 24.0 * e[index] * h + 60 * f[index] * h2);
         acc.set(2.0 * c[index] + 6.0 * d[index] * h + 12.0 * e[index] * h2 + 20.0 * f[index] * h3);
         vel.set(b[index] + 2.0 * c[index] * h + 3.0 * d[index] * h2 + 4.0 * e[index] * h3 + 5.0 * f[index] * h4);
      }

   }

   @Override
   public boolean isDone()
   {
      return currentTime.getValue() > x[numberOfPoints.getValue() - 1];
   }

   @Override
   public void initialize()
   {
      throw new RuntimeException("Use initialize(double[] xIn) and determineCoefficients() to initialize the quintic spline interpolator");
   }

}
