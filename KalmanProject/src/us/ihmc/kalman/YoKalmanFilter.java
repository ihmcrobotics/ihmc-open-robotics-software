package us.ihmc.kalman;

import static org.ejml.ops.CommonOps.addEquals;
import static us.ihmc.yoUtilities.math.MatrixYoVariableConversionTools.checkPositiveSemiDefinite;
import static us.ihmc.yoUtilities.math.MatrixYoVariableConversionTools.getFromYoVariablesMatrix;
import static us.ihmc.yoUtilities.math.MatrixYoVariableConversionTools.getFromYoVariablesSymmetric;
import static us.ihmc.yoUtilities.math.MatrixYoVariableConversionTools.getFromYoVariablesVector;
import static us.ihmc.yoUtilities.math.MatrixYoVariableConversionTools.populateYoVariablesMatrix;
import static us.ihmc.yoUtilities.math.MatrixYoVariableConversionTools.populateYoVariablesSymmetricMatrix;
import static us.ihmc.yoUtilities.math.MatrixYoVariableConversionTools.populateYoVariablesVector;
import static us.ihmc.yoUtilities.math.MatrixYoVariableConversionTools.storeInYoVariablesMatrix;
import static us.ihmc.yoUtilities.math.MatrixYoVariableConversionTools.storeInYoVariablesSymmetric;
import static us.ihmc.yoUtilities.math.MatrixYoVariableConversionTools.storeInYoVariablesVector;

import java.util.ArrayList;
import java.util.List;

import org.ejml.alg.dense.mult.MatrixMatrixMult;
import org.ejml.alg.dense.mult.MatrixVectorMult;
import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.factory.SingularMatrixException;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;
import org.ejml.ops.MatrixFeatures;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.linearAlgebra.MatrixTools;



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
   private final DenseMatrix64F F = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F G = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F H = new DenseMatrix64F(1, 1);

   // Noise model
   // Q is the covariance matrix for the process noise.
   // R is the covariance matrix for the sensor noise.
   private final DenseMatrix64F Q = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F R = new DenseMatrix64F(1, 1);

   // System state estimate (x is state, P is state noise covariance matrix)
   private final DenseMatrix64F x = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F P = new DenseMatrix64F(1, 1);

   // These are pre-declared for efficiency reasons
   private final DenseMatrix64F a = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F b = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F r = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F S = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F S_inv = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F c = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F d = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F K = new DenseMatrix64F(1, 1);

   private final LinearSolver<DenseMatrix64F> solver;

   // YoVariables
   private final List<List<DoubleYoVariable>> yoF = new ArrayList<List<DoubleYoVariable>>();
   private final List<List<DoubleYoVariable>> yoG = new ArrayList<List<DoubleYoVariable>>();
   private final List<List<DoubleYoVariable>> yoH = new ArrayList<List<DoubleYoVariable>>();
   private final List<List<DoubleYoVariable>> yoQ = new ArrayList<List<DoubleYoVariable>>();
   private final List<List<DoubleYoVariable>> yoR = new ArrayList<List<DoubleYoVariable>>();
   private final List<List<DoubleYoVariable>> yoP = new ArrayList<List<DoubleYoVariable>>();
   private final List<DoubleYoVariable> yoX = new ArrayList<DoubleYoVariable>();

   private final IntegerYoVariable nStates;
   private final IntegerYoVariable nInputs;
   private final IntegerYoVariable nMeasurements;

   private final BooleanYoVariable updateCovarianceAndGain;

   private boolean doChecks = false;

   public YoKalmanFilter(String name, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name);
      nStates = new IntegerYoVariable("nStates", registry);
      nInputs = new IntegerYoVariable("nInputs", registry);
      nMeasurements = new IntegerYoVariable("nMeasurements", registry);


      // covariance matrices are symmetric positive semi-definite
      int matrixWidth = 0; // only used to decide which algorithm to use. we typically use small matrices, so this is fine
      solver = LinearSolverFactory.symmPosDef(matrixWidth);

      // wrap the solver so that it doesn't modify the input
//    solver = new LinearSolverSafe<DenseMatrix64F>(solver);
      // A little bit more performance can be gained by letting S be modified.  In some
      // applications S should not be modified.


      updateCovarianceAndGain = new BooleanYoVariable(name + "UpdateCovarianceAndGain",
              "Whether or not to update the state covariance matrix and the kalman gain K matrix each update", registry);
      updateCovarianceAndGain.set(true);

      parentRegistry.addChild(registry);
   }

   public void setUpdateCovarianceAndKalmanGain(boolean propagateCovariance)
   {
      this.updateCovarianceAndGain.set(propagateCovariance);
   }

   public void configure(DenseMatrix64F F, DenseMatrix64F G, DenseMatrix64F H)
   {
      nStates.set(F.getNumRows());
      nInputs.set(G.getNumCols());
      nMeasurements.set(H.getNumRows());

      if (doChecks)
      {
         MatrixTools.checkMatrixDimensions(F, nStates.getIntegerValue(), nStates.getIntegerValue());
         MatrixTools.checkMatrixDimensions(G, nStates.getIntegerValue(), nInputs.getIntegerValue());
         MatrixTools.checkMatrixDimensions(H, nMeasurements.getIntegerValue(), nStates.getIntegerValue());
      }

      populateYoVariablesMatrix(yoF, F.getNumRows(), F.getNumCols(), "F", registry);
      populateYoVariablesMatrix(yoG, G.getNumRows(), G.getNumCols(), "G", registry);
      populateYoVariablesMatrix(yoH, H.getNumRows(), H.getNumCols(), "H", registry);
      populateYoVariablesVector(yoX, nStates.getIntegerValue(), "x", registry);
      populateYoVariablesSymmetricMatrix(yoP, nStates.getIntegerValue(), "P", registry);

      storeInYoVariablesMatrix(F, yoF);
      storeInYoVariablesMatrix(G, yoG);
      storeInYoVariablesMatrix(H, yoH);
   }


   public void setProcessNoiseCovariance(DenseMatrix64F Q)
   {
      if (doChecks)
      {
         checkPositiveSemiDefinite(Q);
         MatrixTools.checkMatrixDimensions(Q, nStates.getIntegerValue(), nStates.getIntegerValue());
      }

      populateYoVariablesSymmetricMatrix(yoQ, Q.getNumRows(), "Q", registry);

      storeInYoVariablesSymmetric(Q, yoQ);
   }

   public void setMeasurementNoiseCovariance(DenseMatrix64F R)
   {
      if (doChecks)
      {
         checkPositiveSemiDefinite(R);
         MatrixTools.checkMatrixDimensions(R, nMeasurements.getIntegerValue(), nMeasurements.getIntegerValue());
      }

      populateYoVariablesSymmetricMatrix(yoR, R.getNumRows(), "R", registry);

      storeInYoVariablesSymmetric(R, yoR);
   }

   public void setState(DenseMatrix64F x, DenseMatrix64F P)
   {
      if (doChecks)
      {
         MatrixTools.checkMatrixDimensions(x, nStates.getIntegerValue(), 1);
         MatrixTools.checkMatrixDimensions(P, nStates.getIntegerValue(), nStates.getIntegerValue());
      }

      storeInYoVariablesVector(x, yoX);
      storeInYoVariablesSymmetric(P, yoP);
   }

   public void predict(DenseMatrix64F u)
   {
      if (doChecks)
      {
         MatrixTools.checkMatrixDimensions(u, nInputs.getIntegerValue(), 1);
      }

      checkForNaN(u);

      getVariablesForPredictFromYoVariables();

      updateAPrioriState(x, u);
      storeInYoVariablesVector(x, yoX);

      if (updateCovarianceAndGain.getBooleanValue())
      {
         updateAPrioriCovariance();
         storeInYoVariablesSymmetric(P, yoP);
      }
   }

   protected void updateAPrioriState(DenseMatrix64F x, DenseMatrix64F u)
   {
      a.reshape(nStates.getIntegerValue(), 1);

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
      int nStates = this.nStates.getIntegerValue();
      P.reshape(nStates, nStates);
      b.reshape(nStates, nStates);

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
      storeInYoVariablesVector(x, yoX);

      if (updateCovarianceAndGain.getBooleanValue())
      {
         updateAPosterioriStateCovariance();
         storeInYoVariablesSymmetric(P, yoP);
      }
   }

   private void getVariablesForPredictFromYoVariables()
   {
      int nStates = this.nStates.getIntegerValue();
      int nInputs = this.nInputs.getIntegerValue();

      F.reshape(nStates, nStates);
      G.reshape(nStates, nInputs);
      Q.reshape(nStates, nStates);
      x.reshape(nStates, 1);
      P.reshape(nStates, nStates);

      getFromYoVariablesMatrix(F, yoF);
      getFromYoVariablesMatrix(G, yoG);
      getFromYoVariablesSymmetric(Q, yoQ);
      getFromYoVariablesVector(x, yoX);
      getFromYoVariablesSymmetric(P, yoP);
   }

   private void getVariablesForUpdateFromYoVariables()
   {
      int nMeasurements = this.nMeasurements.getIntegerValue();
      int nStates = this.nStates.getIntegerValue();
      H.reshape(nMeasurements, nStates);
      R.reshape(nMeasurements, nMeasurements);
      x.reshape(nStates, 1);
      P.reshape(nStates, nStates);

      getFromYoVariablesMatrix(H, yoH);
      getFromYoVariablesSymmetric(R, yoR);
      getFromYoVariablesVector(x, yoX);
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
      int nMeasurements = this.nMeasurements.getIntegerValue();
      int nStates = this.nStates.getIntegerValue();
      c.reshape(nMeasurements, nStates);
      S.reshape(nMeasurements, nMeasurements);
      S_inv.reshape(nMeasurements, nMeasurements);
      d.reshape(nStates, nMeasurements);
      K.reshape(nStates, nMeasurements);

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
      CommonOps.subtractEquals(P, b);
   }

   protected void updateAPosterioriState(DenseMatrix64F x, DenseMatrix64F y, DenseMatrix64F K)
   {
      r.reshape(nMeasurements.getIntegerValue(), 1);

      // r = y - H x
      MatrixVectorMult.mult(H, x, r);
      CommonOps.subtract(y, r, r);

      // x = x + Kr
      MatrixVectorMult.mult(K, r, a);
      addEquals(x, a);
   }

   public DenseMatrix64F getState()
   {
      int nStates = this.nStates.getIntegerValue();
      x.reshape(nStates, 1);
      getFromYoVariablesVector(x, yoX);

      return x;
   }

   public DenseMatrix64F getCovariance()
   {
      int nStates = this.nStates.getIntegerValue();
      P.reshape(nStates, nStates);
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

   public int getNumberOfStates()
   {
      return nStates.getIntegerValue();
   }

   public int getNumberOfInputs()
   {
      return nInputs.getIntegerValue();
   }

   public int getNumberOfMeasurements()
   {
      return nMeasurements.getIntegerValue();
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
