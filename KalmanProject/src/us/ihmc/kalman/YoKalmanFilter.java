package us.ihmc.kalman;

import static org.ejml.ops.CommonOps.addEquals;
import static org.ejml.ops.CommonOps.sub;
import static org.ejml.ops.CommonOps.subEquals;

import org.ejml.alg.dense.linsol.LinearSolver;
import org.ejml.alg.dense.linsol.LinearSolverFactory;
import org.ejml.alg.dense.mult.MatrixMatrixMult;
import org.ejml.alg.dense.mult.MatrixVectorMult;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixFeatures;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;


/**
 * Adapted from http://code.google.com/p/efficient-java-matrix-library/wiki/KalmanFilterExamples
 *
 * @author Peter Abeles, Twan Koolen
 */
public class YoKalmanFilter implements KalmanFilter
{
   private final YoVariableRegistry registry;

   // dynamics
   private final DenseMatrix64F F;
   private final DenseMatrix64F G;
   private final DenseMatrix64F H;

   // noise model
   private final DenseMatrix64F Q;
   private final DenseMatrix64F R;

   // sytem state estimate
   private final DenseMatrix64F x;
   private final DenseMatrix64F P;

   // these are predeclared for efficiency reasons
   private final DenseMatrix64F a, b;
   private final DenseMatrix64F r, S, S_inv, c, d;
   private final DenseMatrix64F K;

   private final LinearSolver<DenseMatrix64F> solver;

   // YoVariables
   private final DoubleYoVariable[][] yoF;
   private final DoubleYoVariable[][] yoG;
   private final DoubleYoVariable[][] yoH;
   private final DoubleYoVariable[] yoQ;
   private final DoubleYoVariable[] yoR;
   private final DoubleYoVariable[] yoX;
   private final DoubleYoVariable[] yoP;

   private boolean doChecks = false;

   public YoKalmanFilter(String name, int nStates, int nInputs, int nMeasurements, YoVariableRegistry parentRegistry)
   {
      F = new DenseMatrix64F(nStates, nStates);
      G = new DenseMatrix64F(nStates, nInputs);
      H = new DenseMatrix64F(nMeasurements, nStates);

      Q = new DenseMatrix64F(nStates, nStates);
      R = new DenseMatrix64F(nMeasurements, nMeasurements);

      a = new DenseMatrix64F(nStates, 1);
      b = new DenseMatrix64F(nStates, nStates);
      r = new DenseMatrix64F(nMeasurements, 1);
      S = new DenseMatrix64F(nMeasurements, nMeasurements);
      S_inv = new DenseMatrix64F(nMeasurements, nMeasurements);
      c = new DenseMatrix64F(nMeasurements, nStates);
      d = new DenseMatrix64F(nStates, nMeasurements);
      K = new DenseMatrix64F(nStates, nMeasurements);
      x = new DenseMatrix64F(nStates, 1);
      P = new DenseMatrix64F(nStates, nStates);

      // covariance matrices are symmetric positive semi-definite
      solver = LinearSolverFactory.symmPosDef(nStates);

      // wrap the solver so that it doesn't modify the input
//    solver = new LinearSolverSafe<DenseMatrix64F>(solver);
      // A little bit more performance can be gained by letting S be modified.  In some
      // applications S should not be modified.

      registry = new YoVariableRegistry(name);
      yoF = new DoubleYoVariable[F.getNumRows()][F.getNumCols()];
      yoG = new DoubleYoVariable[G.getNumRows()][G.getNumCols()];
      yoH = new DoubleYoVariable[H.getNumRows()][H.getNumCols()];
      yoR = new DoubleYoVariable[getNumberOfElementsForSymmetricMatrix(R.getNumRows())];
      yoQ = new DoubleYoVariable[getNumberOfElementsForSymmetricMatrix(Q.getNumRows())];
      yoX = new DoubleYoVariable[x.getNumRows()];
      yoP = new DoubleYoVariable[getNumberOfElementsForSymmetricMatrix(P.getNumRows())];

      populateYoVariables(nStates, nMeasurements);
      parentRegistry.addChild(registry);
   }

   public void configure(DenseMatrix64F F, DenseMatrix64F G, DenseMatrix64F H)
   {
      if (doChecks)
      {
         checkSize(F, this.F);
         checkSize(G, this.G);
         checkSize(H, this.H);
      }

      storeInYoVariables(F, yoF);
      storeInYoVariables(G, yoG);
      storeInYoVariables(H, yoH);
   }

   public void setProcessNoiseCovariance(DenseMatrix64F Q)
   {
      if (doChecks)
      {
         checkSize(Q, this.Q);
         checkPositiveDefinite(Q);
      }

      storeInYoVariablesSymmetric(Q, yoQ);
   }

   public void setMeasurementNoiseCovariance(DenseMatrix64F R)    // not checking for positive definiteness
   {
      if (doChecks)
      {
         checkSize(R, this.R);
         checkPositiveDefinite(R);
      }

      storeInYoVariablesSymmetric(R, yoR);
   }

   public void setState(DenseMatrix64F x, DenseMatrix64F P)
   {
      storeInYoVariables(x, yoX);
      storeInYoVariablesSymmetric(P, yoP);
   }

   public void predict(DenseMatrix64F u)
   {
      if (doChecks)
      {
         if (MatrixFeatures.hasNaN(u))
            throw new RuntimeException("u contains NaN: " + u);
      }

      getFromYoVariables(F, yoF);
      getFromYoVariables(G, yoG);
      getFromYoVariablesSymmetric(Q, yoQ);
      getFromYoVariables(x, yoX);
      getFromYoVariablesSymmetric(P, yoP);

      // x = F x + G u
      MatrixVectorMult.mult(F, x, a);
      x.set(a);
      MatrixVectorMult.mult(G, u, a);
      addEquals(x, a);

      // P = F P F' + Q
      MatrixMatrixMult.mult_small(F, P, b);
      MatrixMatrixMult.multTransB(b, F, P);
      addEquals(P, Q);

      storeInYoVariables(x, yoX);
      storeInYoVariablesSymmetric(P, yoP);
   }

   public void update(DenseMatrix64F y)
   {
      if (doChecks)
      {
         if (MatrixFeatures.hasNaN(y))
            throw new RuntimeException("y contains NaN: " + y);
      }

      getFromYoVariables(H, yoH);
      getFromYoVariablesSymmetric(R, yoR);
      getFromYoVariables(x, yoX);
      getFromYoVariablesSymmetric(P, yoP);

      // r = y - H x
      MatrixVectorMult.mult(H, x, r);
      sub(y, r, r);

      // S = H P H' + R
      MatrixMatrixMult.mult_small(H, P, c);
      MatrixMatrixMult.multTransB(c, H, S);
      addEquals(S, R);

      // K = PH'S^(-1)
      if (!solver.setA(S))
         throw new RuntimeException("Invert failed");
      solver.invert(S_inv);
      MatrixMatrixMult.multTransA_small(H, S_inv, d);
      MatrixMatrixMult.mult_small(P, d, K);

      // x = x + Kr
      MatrixVectorMult.mult(K, r, a);
      addEquals(x, a);

      // P = (I-KH)P = P - (KH)P = P-K(HP)
      MatrixMatrixMult.mult_small(H, P, c);
      MatrixMatrixMult.mult_small(K, c, b);
      subEquals(P, b);

      storeInYoVariables(x, yoX);
      storeInYoVariablesSymmetric(P, yoP);
   }

   public DenseMatrix64F getState()
   {
      getFromYoVariables(x, yoX);

      return x;
   }

   public DenseMatrix64F getCovariance()
   {
      getFromYoVariablesSymmetric(P, yoP);

      return P;
   }

   public void setDoChecks(boolean doChecks)
   {
      this.doChecks = doChecks;
   }

   private void populateYoVariables(int nStates, int nMeasurements)
   {
      populateYoVariables(yoF, "F");
      populateYoVariables(yoG, "G");
      populateYoVariables(yoH, "H");
      populateYoVariablesSymmetric(yoQ, "Q", nStates);
      populateYoVariablesSymmetric(yoR, "R", nMeasurements);
      populateYoVariables(yoX, "x");
      populateYoVariablesSymmetric(yoP, "P", nStates);
   }

   private void populateYoVariables(DoubleYoVariable[][] yoVariableArray, String prefix)
   {
      for (int i = 0; i < yoVariableArray.length; i++)
      {
         for (int j = 0; j < yoVariableArray[0].length; j++)
         {
            yoVariableArray[i][j] = new DoubleYoVariable(prefix + Integer.toString(i) + Integer.toString(j), registry);
         }
      }
   }

   private void populateYoVariablesSymmetric(DoubleYoVariable[] yoVariableArray, String prefix, int size)
   {
      int n = 0;
      for (int i = 0; i < size; i++)
      {
         for (int j = i; j < size; j++)
         {
            yoVariableArray[n] = new DoubleYoVariable(prefix + Integer.toString(i) + Integer.toString(j), registry);
            n++;
         }
      }
   }

   private void populateYoVariables(DoubleYoVariable[] yoVariableArray, String prefix)
   {
      for (int i = 0; i < yoVariableArray.length; i++)
      {
         yoVariableArray[i] = new DoubleYoVariable(prefix + Integer.toString(i), registry);
      }
   }

   private void storeInYoVariables(DenseMatrix64F m, DoubleYoVariable[][] yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         for (int j = 0; j < m.getNumCols(); j++)
         {
            yoM[i][j].set(m.get(i, j));
         }
      }
   }

   private void storeInYoVariablesSymmetric(DenseMatrix64F m, DoubleYoVariable[] yoM)
   {
      int size = m.getNumRows();

      int n = 0;
      for (int i = 0; i < size; i++)
      {
         for (int j = i; j < size; j++)
         {
            yoM[n].set(m.get(i, j));
            n++;
         }
      }
   }

   private void storeInYoVariables(DenseMatrix64F v, DoubleYoVariable[] yoV)
   {
      for (int i = 0; i < v.getNumRows(); i++)
      {
         yoV[i].set(v.get(i));
      }
   }

   private void getFromYoVariables(DenseMatrix64F m, DoubleYoVariable[][] yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         for (int j = 0; j < m.getNumCols(); j++)
         {
            m.set(i, j, yoM[i][j].getDoubleValue());
         }
      }
   }

   private void getFromYoVariablesSymmetric(DenseMatrix64F m, DoubleYoVariable[] yoM)
   {
      int size = m.getNumRows();

      int n = 0;
      for (int i = 0; i < size; i++)
      {
         for (int j = i; j < size; j++)
         {
            m.set(i, j, yoM[n].getDoubleValue());
            m.set(j, i, yoM[n].getDoubleValue());
            n++;
         }
      }
   }

   private void getFromYoVariables(DenseMatrix64F m, DoubleYoVariable[] yoM)
   {
      for (int i = 0; i < m.getNumRows(); i++)
      {
         m.set(i, 0, yoM[i].getDoubleValue());
      }
   }

   private static int getNumberOfElementsForSymmetricMatrix(int size)
   {
      return size * (size + 1) / 2;
   }

   private static void checkPositiveDefinite(DenseMatrix64F m)
   {
      if (!MatrixFeatures.isPositiveSemidefinite(m))
         throw new RuntimeException("Matrix is not positive semidefinite: " + m);
   }

   private static void checkSize(DenseMatrix64F m1, DenseMatrix64F m2)
   {
      boolean nRowsNotEqual = m1.getNumRows() != m2.getNumRows();
      boolean nColsNotEqual = m1.getNumCols() != m2.getNumCols();

      if (nRowsNotEqual || nColsNotEqual)
      {
         throw new RuntimeException("Matrix sizes not equal: " + m1 + "\n\n" + m2);
      }
   }
}
