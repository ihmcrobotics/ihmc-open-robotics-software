package us.ihmc.kalman;

import static com.yobotics.simulationconstructionset.util.MatrixYoVariableConversionTools.checkPositiveDefinite;
import static com.yobotics.simulationconstructionset.util.MatrixYoVariableConversionTools.checkSize;
import static com.yobotics.simulationconstructionset.util.MatrixYoVariableConversionTools.getFromYoVariables;
import static com.yobotics.simulationconstructionset.util.MatrixYoVariableConversionTools.getFromYoVariablesSymmetric;
import static com.yobotics.simulationconstructionset.util.MatrixYoVariableConversionTools.getNumberOfElementsForSymmetricMatrix;
import static com.yobotics.simulationconstructionset.util.MatrixYoVariableConversionTools.storeInYoVariables;
import static com.yobotics.simulationconstructionset.util.MatrixYoVariableConversionTools.storeInYoVariablesSymmetric;
import static org.ejml.ops.CommonOps.addEquals;
import static org.ejml.ops.CommonOps.sub;
import static org.ejml.ops.CommonOps.subEquals;

import org.ejml.alg.dense.mult.MatrixMatrixMult;
import org.ejml.alg.dense.mult.MatrixVectorMult;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolver;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.factory.SingularMatrixException;
import org.ejml.ops.MatrixFeatures;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.MatrixYoVariableConversionTools;


/**
 * Adapted from http://code.google.com/p/efficient-java-matrix-library/wiki/KalmanFilterExamples
 *
 * @author Peter Abeles, Twan Koolen
 */
public class YoKalmanFilter implements KalmanFilter
{
   protected final YoVariableRegistry registry;

   // Dynamics (x = F x + G u + w; y = H x + v)
   // x is the state, u is the input, w is process noise.
   // v is sensor noise.
   private final DenseMatrix64F F;
   private final DenseMatrix64F G;
   private final DenseMatrix64F H;

   // Noise model
   // Q is the covariance matrix for the process noise.
   // R is the covariance matrix for the sensor noise.
   private final DenseMatrix64F Q;
   private final DenseMatrix64F R;

   // System state estimate (x is state, P is state noise covariance matrix)
   private final DenseMatrix64F x;
   private final DenseMatrix64F P;

   // These are pre-declared for efficiency reasons
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

   private final BooleanYoVariable updateCovarianceAndGain;

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

      updateCovarianceAndGain = new BooleanYoVariable(name + "UpdateCovarianceAndGain",
              "Whether or not to update the state covariance matrix and the kalman gain K matrix each update", registry);
      updateCovarianceAndGain.set(true);

      populateYoVariables(nStates, nMeasurements);
      parentRegistry.addChild(registry);
   }

   public void setUpdateCovarianceAndKalmanGain(boolean propagateCovariance)
   {
      this.updateCovarianceAndGain.set(propagateCovariance);
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

   public void setMeasurementNoiseCovariance(DenseMatrix64F R)
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
      checkForNaN(u);

      getVariablesForPredictFromYoVariables();

      updateAPrioriState(x, u);
      storeInYoVariables(x, yoX);

      if (updateCovarianceAndGain.getBooleanValue())
      {
         updateAPrioriCovariance();
         storeInYoVariablesSymmetric(P, yoP);
      }
   }

   protected void updateAPrioriState(DenseMatrix64F x, DenseMatrix64F u)
   {
      // x = F x + G u
      MatrixVectorMult.mult(F, x, a);
      x.set(a);

      if (u.getNumRows() > 0)
      {
         MatrixVectorMult.mult(G, u, a);
         addEquals(x, a);
      }
   }

   private void updateAPrioriCovariance()
   {
      // P = F P F' + Q
      MatrixMatrixMult.mult_small(F, P, b);
      MatrixMatrixMult.multTransB(b, F, P);
      addEquals(P, Q);
   }

   public void update(DenseMatrix64F y)
   {
      checkForNaN(y);

      getVariablesForUpdateFromYoVariables();

      if (updateCovarianceAndGain.getBooleanValue())
      {
         updateKalmanGainMatrixK();
      }

      updateAPosterioriState(x, y, K);
      storeInYoVariables(x, yoX);

      if (updateCovarianceAndGain.getBooleanValue())
      {
         updateAPosterioriStateCovariance();
         storeInYoVariablesSymmetric(P, yoP);
      }
   }

   private void getVariablesForPredictFromYoVariables()
   {
      getFromYoVariables(F, yoF);
      getFromYoVariables(G, yoG);
      getFromYoVariablesSymmetric(Q, yoQ);
      getFromYoVariables(x, yoX);
      getFromYoVariablesSymmetric(P, yoP);
   }

   private void getVariablesForUpdateFromYoVariables()
   {
      getFromYoVariables(H, yoH);
      getFromYoVariablesSymmetric(R, yoR);
      getFromYoVariables(x, yoX);
      getFromYoVariablesSymmetric(P, yoP);
   }

   private void checkForNaN(DenseMatrix64F matrix)
   {
      if (doChecks)
      {
         if ((matrix != null) && MatrixFeatures.hasNaN(matrix))
         {
            throw new RuntimeException("Matrix contains NaN: " + matrix);
         }
      }
   }

   private void updateKalmanGainMatrixK()
   {
      // S = H P H' + R
      MatrixMatrixMult.mult_small(H, P, c);
      MatrixMatrixMult.multTransB(c, H, S);
      addEquals(S, R);

      // K = PH'S^(-1)
      if (!solver.setA(S))
         throw new SingularMatrixException();
      solver.invert(S_inv);
      MatrixMatrixMult.multTransA_small(H, S_inv, d);
      MatrixMatrixMult.mult_small(P, d, K);
   }

   private void updateAPosterioriStateCovariance()
   {
      // P = (I-KH)P = P - (KH)P = P-K(HP)
      MatrixMatrixMult.mult_small(H, P, c);
      MatrixMatrixMult.mult_small(K, c, b);
      subEquals(P, b);
   }

   protected void updateAPosterioriState(DenseMatrix64F x, DenseMatrix64F y, DenseMatrix64F K)
   {
      // r = y - H x
      MatrixVectorMult.mult(H, x, r);
      sub(y, r, r);

      // x = x + Kr
      MatrixVectorMult.mult(K, r, a);
      addEquals(x, a);
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

   public DenseMatrix64F getKGain()
   {
      return K;
   }

   public void setDoChecks(boolean doChecks)
   {
      this.doChecks = doChecks;
   }

   private void populateYoVariables(int nStates, int nMeasurements)
   {
      MatrixYoVariableConversionTools.populateYoVariables(yoF, "F", registry);
      MatrixYoVariableConversionTools.populateYoVariables(yoG, "G", registry);
      MatrixYoVariableConversionTools.populateYoVariables(yoH, "H", registry);
      MatrixYoVariableConversionTools.populateYoVariablesSymmetric(yoQ, "Q", nStates, registry);
      MatrixYoVariableConversionTools.populateYoVariablesSymmetric(yoR, "R", nMeasurements, registry);
      MatrixYoVariableConversionTools.populateYoVariables(yoX, "x", registry);
      MatrixYoVariableConversionTools.populateYoVariablesSymmetric(yoP, "P", nStates, registry);
   }

   public int getNumberOfStates()
   {
      return F.getNumRows();
   }

   public int getNumberOfInputs()
   {
      return G.getNumCols();
   }

   public int getNumberOfMeasurements()
   {
      return H.getNumRows();
   }

   // Iteratively computes the K Matrix. Assumes the process and measurement covariances are already set.
   public void computeSteadyStateGainAndCovariance(int numberOfIterations)
   {
      getVariablesForPredictFromYoVariables();
      getVariablesForUpdateFromYoVariables();

      for (int i = 0; i < numberOfIterations; i++)
      {
         updateAPrioriCovariance();
         updateKalmanGainMatrixK();
         updateAPosterioriStateCovariance();
      }

      storeInYoVariablesSymmetric(P, yoP);
   }
}
