package us.ihmc.kalman;

import static org.junit.Assert.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.EjmlUnitTests;
import org.ejml.ops.RandomMatrices;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.listener.YoVariableRegistryChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.DataBuffer.RepeatDataBufferEntryException;

public class YoKalmanFilterTest
{
   private static final boolean DEBUG = false;

   private Random random;
   private YoVariableRegistry parentRegistry;

   private int nStates;
   private int nInputs;
   private int nMeasurements;

   private DenseMatrix64F F;
   private DenseMatrix64F G;
   private DenseMatrix64F H;
   private DenseMatrix64F Q;
   private DenseMatrix64F R;
   private DenseMatrix64F x;
   private DenseMatrix64F P;
   private DenseMatrix64F u;
   private DenseMatrix64F y;

   @Before
   public void setUp()
   {
      random = new Random(1776L);
      parentRegistry = new YoVariableRegistry("testRegistry");
      nStates = 8;
      nInputs = 5;
      nMeasurements = 3;
      createRandomParameters(nStates, nInputs, nMeasurements);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCompareToSimple()
   {
      KalmanFilterSimple kalmanFilterSimple = new KalmanFilterSimple();
      YoKalmanFilter yoKalmanFilter = new YoKalmanFilter("yo", parentRegistry);
      KalmanFilter[] kalmanFilters = {kalmanFilterSimple, yoKalmanFilter};

      for (KalmanFilter kalmanFilter : kalmanFilters)
      {
         configureFilter(kalmanFilter);
         kalmanFilter.predict(u);
         kalmanFilter.update(y);
      }

      EjmlUnitTests.assertEquals(kalmanFilters[0].getState(), kalmanFilters[1].getState(), 1e-8);
      EjmlUnitTests.assertEquals(kalmanFilters[0].getCovariance(), kalmanFilters[1].getCovariance(), 1e-8);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testNotProcessCovarianceSymmetricPositiveDefinite1()
   {
      YoKalmanFilter yoKalmanFilter = null;
      try
      {
         yoKalmanFilter = new YoKalmanFilter("yo", parentRegistry);
         configureFilter(yoKalmanFilter);
         yoKalmanFilter.setDoChecks(true);
         Q = RandomMatrices.createSymmPosDef(nStates, random);
         yoKalmanFilter.setProcessNoiseCovariance(Q);
         Q = RandomMatrices.createRandom(nStates, nStates, random);
      }
      catch (RuntimeException e)
      {
         fail();
      }

      yoKalmanFilter.setProcessNoiseCovariance(Q);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testMeasurementCovarianceNotSymmetricPositiveDefinite1()
   {
      YoKalmanFilter yoKalmanFilter = null;
      try
      {
         yoKalmanFilter = new YoKalmanFilter("yo", parentRegistry);
         configureFilter(yoKalmanFilter);
         yoKalmanFilter.setDoChecks(true);
         R = RandomMatrices.createSymmPosDef(nMeasurements, random);
         yoKalmanFilter.setMeasurementNoiseCovariance(R);
         R = RandomMatrices.createRandom(nMeasurements, nMeasurements, random);
      }
      catch (RuntimeException e)
      {
         fail();
      }

      yoKalmanFilter.setMeasurementNoiseCovariance(R);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000,expected = RuntimeException.class)
   public void testWrongSize()
   {
      YoKalmanFilter yoKalmanFilter = new YoKalmanFilter("yo", parentRegistry);
      yoKalmanFilter.setDoChecks(true);
      yoKalmanFilter.setProcessNoiseCovariance(new DenseMatrix64F(nStates + 1, nStates + 1));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testRewindability() throws RepeatDataBufferEntryException
   {
      int nTicks = 2;
      DenseMatrix64F[] Fs = new DenseMatrix64F[nTicks + 1];
      DenseMatrix64F[] Gs = new DenseMatrix64F[nTicks + 1];
      DenseMatrix64F[] Hs = new DenseMatrix64F[nTicks + 1];
      DenseMatrix64F[] Qs = new DenseMatrix64F[nTicks + 1];
      DenseMatrix64F[] Rs = new DenseMatrix64F[nTicks + 1];
      DenseMatrix64F[] xs = new DenseMatrix64F[nTicks + 1];
      DenseMatrix64F[] Ps = new DenseMatrix64F[nTicks + 1];
      DenseMatrix64F[] us = new DenseMatrix64F[nTicks + 1];
      DenseMatrix64F[] ys = new DenseMatrix64F[nTicks + 1];

      for (int i = 0; i < nTicks + 1; i++)
      {
         Fs[i] = RandomMatrices.createRandom(nStates, nStates, random);
         Gs[i] = RandomMatrices.createRandom(nStates, nInputs, random);
         Hs[i] = RandomMatrices.createRandom(nMeasurements, nStates, random);
         Qs[i] = RandomMatrices.createSymmPosDef(nStates, random);
         Rs[i] = RandomMatrices.createSymmPosDef(nMeasurements, random);
         xs[i] = RandomMatrices.createRandom(nStates, 1, random);
         Ps[i] = RandomMatrices.createSymmPosDef(nStates, random);
         us[i] = RandomMatrices.createRandom(nInputs, 1, random);
         ys[i] = RandomMatrices.createRandom(nMeasurements, 1, random);
      }

      final DataBuffer dataBuffer = new DataBuffer(1024);
      parentRegistry.attachYoVariableRegistryChangedListener(new YoVariableToDataBufferAdder(dataBuffer));

      YoKalmanFilter yoKalmanFilter1 = new YoKalmanFilter("yo1", parentRegistry);
      YoKalmanFilter yoKalmanFilter2 = new YoKalmanFilter("yo2", new YoVariableRegistry("testRegistry2"));
      KalmanFilter[] kalmanFilters = {yoKalmanFilter1, yoKalmanFilter2};

      for (KalmanFilter kalmanFilter : kalmanFilters)
      {
         kalmanFilter.configure(Fs[0], Gs[0], Hs[0]);
         kalmanFilter.setProcessNoiseCovariance(Qs[0]);
         kalmanFilter.setMeasurementNoiseCovariance(Rs[0]);
         kalmanFilter.setState(xs[0], Ps[0]);
      }

      dataBuffer.tickAndUpdate();
      dataBuffer.setInPoint();

      for (int i = 0; i < nTicks; i++)
      {
         for (KalmanFilter kalmanFilter : kalmanFilters)
         {
            int j = i + 1;
            kalmanFilter.configure(Fs[j], Gs[j], Hs[j]);
            kalmanFilter.setProcessNoiseCovariance(Qs[j]);
            kalmanFilter.setMeasurementNoiseCovariance(Rs[j]);
            kalmanFilter.predict(us[j]);
            kalmanFilter.update(ys[j]);
         }

         dataBuffer.tickAndUpdate();
      }

      dataBuffer.gotoInPoint();
      KalmanFilter kalmanFilter = kalmanFilters[0];
      for (int i = 0; i < nTicks; i++)
      {
         int j = i + 1;
         kalmanFilter.configure(Fs[j], Gs[j], Hs[j]);
         kalmanFilter.setProcessNoiseCovariance(Qs[j]);
         kalmanFilter.setMeasurementNoiseCovariance(Rs[j]);
         kalmanFilter.predict(us[j]);
         kalmanFilter.update(ys[j]);
      }

      EjmlUnitTests.assertEquals(kalmanFilter.getState(), kalmanFilters[1].getState(), 1e-8);
      EjmlUnitTests.assertEquals(kalmanFilter.getCovariance(), kalmanFilters[1].getCovariance(), 1e-8);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testInfiniteMeasurementNoise()
   {
      nStates = 5;
      nInputs = 1;
      nMeasurements = 2;
      createRandomParameters(nStates, nInputs, nMeasurements);
      YoKalmanFilter kalmanFilter = new YoKalmanFilter("yo", parentRegistry);
      R.set(0, 0, Double.POSITIVE_INFINITY);
      R.set(1, 1, Double.POSITIVE_INFINITY);

      configureFilter(kalmanFilter);

      kalmanFilter.predict(u);
      DenseMatrix64F stateBeforeUpdate = new DenseMatrix64F(kalmanFilter.getState());
      DenseMatrix64F covarianceBeforeUpdate = new DenseMatrix64F(kalmanFilter.getCovariance());

      kalmanFilter.update(y);
      DenseMatrix64F stateAfterUpdate = new DenseMatrix64F(kalmanFilter.getState());
      DenseMatrix64F covarianceAfterUpdate = new DenseMatrix64F(kalmanFilter.getCovariance());

      EjmlUnitTests.assertEquals(stateBeforeUpdate, stateAfterUpdate, 1e-8);
      EjmlUnitTests.assertEquals(covarianceBeforeUpdate, covarianceAfterUpdate, 1e-8);
   }

   private void configureFilter(KalmanFilter kalmanFilter)
   {
      kalmanFilter.configure(F, G, H);
      kalmanFilter.setProcessNoiseCovariance(Q);
      kalmanFilter.setMeasurementNoiseCovariance(R);
      kalmanFilter.setState(x, P);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTwoUpdatesVersusOne()
   {
      nStates = 8;
      nInputs = 1;
      nMeasurements = 5;
      createRandomParameters(nStates, nInputs, nMeasurements);
      R = RandomMatrices.createDiagonal(nMeasurements, 0.0, 1.0, random);

      YoKalmanFilter yoKalmanFilter0 = new YoKalmanFilter("yo0", parentRegistry);
      YoKalmanFilter yoKalmanFilter1 = new YoKalmanFilter("yo1", parentRegistry);
      KalmanFilter[] kalmanFilters = {yoKalmanFilter0, yoKalmanFilter1};

      for (KalmanFilter kalmanFilter : kalmanFilters)
      {
         kalmanFilter.configure(F, G, H);
         kalmanFilter.setProcessNoiseCovariance(Q);
         kalmanFilter.setMeasurementNoiseCovariance(R);
         kalmanFilter.setState(x, P);
         kalmanFilter.predict(u);
      }

      yoKalmanFilter0.update(y);

      for (int i = 0; i < nMeasurements; i++)
      {
         DenseMatrix64F RPrime = new DenseMatrix64F(R);
         for (int j = 0; j < nMeasurements; j++)
         {
            if (i != j)
            {
               RPrime.set(j, j, Double.POSITIVE_INFINITY);
            }
         }

         yoKalmanFilter1.setMeasurementNoiseCovariance(RPrime);
         yoKalmanFilter1.update(y);
      }

      EjmlUnitTests.assertEquals(kalmanFilters[0].getState(), kalmanFilters[1].getState(), 1e-8);
      EjmlUnitTests.assertEquals(kalmanFilters[0].getCovariance(), kalmanFilters[1].getCovariance(), 1e-8);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testViaSimulatingATrueSystemAndCheckingErrorDynamics()
   {
      nStates = 8;
      nInputs = 3;
      nMeasurements = 4;
      createRandomParameters(nStates, nInputs, nMeasurements);

      YoKalmanFilter kalmanFilter = new YoKalmanFilter("yo", parentRegistry);

      configureFilter(kalmanFilter);

      DenseMatrix64F trueState = new DenseMatrix64F(x);
      DenseMatrix64F estimatedState = kalmanFilter.getState();

      DenseMatrix64F stateError = new DenseMatrix64F(nStates, 1);
      CommonOps.subtract(trueState, estimatedState, stateError);

      EjmlUnitTests.assertEquals(stateError, new DenseMatrix64F(nStates, 1), 1e-6);

      int numberOfTicksToTest = 10;

      for (int i = 0; i < numberOfTicksToTest; i++)
      {
         // Create random inputs, process noise, and sensor noise:
         u = RandomMatrices.createRandom(nInputs, 1, random);    // new DenseMatrix64F(nInputs, 1); //
         DenseMatrix64F wProcessNoise = RandomMatrices.createRandom(nStates, 1, random);    // new DenseMatrix64F(nStates, 1); //
         DenseMatrix64F vSensorNoise = RandomMatrices.createRandom(nMeasurements, 1, random);    // new DenseMatrix64F(nMeasurements, 1); //

         // Compute the true state dynamics:
         // trueState = F * trueState + G * u + w
         DenseMatrix64F nextTrueState = new DenseMatrix64F(nStates, 1);
         CommonOps.multAdd(F, trueState, nextTrueState);
         CommonOps.multAdd(G, u, nextTrueState);
         CommonOps.add(nextTrueState, wProcessNoise, nextTrueState);

         printIfDebug("nextTrueState = " + nextTrueState);

         // Compute the true measurments:
         // y = H * trueState + v
         CommonOps.mult(H, nextTrueState, y);
         CommonOps.add(y, vSensorNoise, y);

         // Update the Kalman filter with the input and the measurement and get the estimated state:

         Q = RandomMatrices.createSymmPosDef(nStates, random);
         R = RandomMatrices.createSymmPosDef(nMeasurements, random);
         kalmanFilter.setMeasurementNoiseCovariance(R);
         kalmanFilter.setProcessNoiseCovariance(Q);

         kalmanFilter.predict(u);
         kalmanFilter.update(y);

         estimatedState = kalmanFilter.getState();
         printIfDebug("next estimated state = " + estimatedState);

         // Compute the next true state error:
         DenseMatrix64F nextTrueStateError = new DenseMatrix64F(nStates, 1);
         CommonOps.subtract(nextTrueState, estimatedState, nextTrueStateError);

         printIfDebug("nextTrueStateError = " + nextTrueStateError);

         // Check the error dynamics:
         DenseMatrix64F kalmanKGain = kalmanFilter.getKGain();
         printIfDebug("kalmanKGain = " + kalmanKGain);

         DenseMatrix64F KH = new DenseMatrix64F(nStates, nStates);
         CommonOps.mult(kalmanKGain, H, KH);
         DenseMatrix64F Identity = new DenseMatrix64F(nStates, nStates);
         CommonOps.setIdentity(Identity);
         DenseMatrix64F IMinusKH = new DenseMatrix64F(Identity);
         CommonOps.subtract(IMinusKH, KH, IMinusKH);

         printIfDebug("IMinusKH = " + IMinusKH);

         DenseMatrix64F IMinusKHTimesF = new DenseMatrix64F(nStates, nStates);
         CommonOps.mult(IMinusKH, F, IMinusKHTimesF);

         DenseMatrix64F computedNextError = new DenseMatrix64F(nStates, 1);
         CommonOps.multAdd(IMinusKHTimesF, stateError, computedNextError);
         CommonOps.multAdd(IMinusKH, wProcessNoise, computedNextError);

         DenseMatrix64F negativeVSensorNoise = new DenseMatrix64F(vSensorNoise);
         CommonOps.scale(-1.0, negativeVSensorNoise);
         CommonOps.multAdd(kalmanKGain, negativeVSensorNoise, computedNextError);

         printIfDebug("computedNextError = " + computedNextError);

         EjmlUnitTests.assertEquals(nextTrueStateError, computedNextError, 1e-6);

         trueState = nextTrueState;
         stateError = nextTrueStateError;
      }

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.5)
	@Test(timeout = 30000)
   public void testViaKeepingCovariancesConstantAndCheckingKMatrixConvergence()
   {
      nStates = 6;
      nInputs = 1;
      nMeasurements = 8;
      createRandomParameters(nStates, nInputs, nMeasurements);

      YoKalmanFilter kalmanFilter = new YoKalmanFilter("yo", parentRegistry);

      configureFilter(kalmanFilter);

      int numberOfTicksToTest = 1000;

      for (int i = 0; i < numberOfTicksToTest; i++)
      {
         // Update the Kalman filter with the input and the measurement and get the estimated state:
         kalmanFilter.predict(u);
         kalmanFilter.update(y);
         DenseMatrix64F kalmanKGain = kalmanFilter.getKGain();
         printIfDebug("kalmanKGain = " + kalmanKGain);
         printIfDebug("Covariance = " + kalmanFilter.getCovariance());
      }

      {
         // Check that, after prediction but before update, P = FPF' - FPH'(HPH'+R)^-1 HPF' + Q
         kalmanFilter.predict(u);

         DenseMatrix64F P = kalmanFilter.getCovariance();

         DenseMatrix64F FTranspose = new DenseMatrix64F(F);
         CommonOps.transpose(FTranspose);

         DenseMatrix64F HTranspose = new DenseMatrix64F(H);
         CommonOps.transpose(HTranspose);

         DenseMatrix64F FP = new DenseMatrix64F(nStates, nStates);
         CommonOps.mult(F, P, FP);

         DenseMatrix64F FPFTranspose = new DenseMatrix64F(nStates, nStates);
         CommonOps.mult(FP, FTranspose, FPFTranspose);

         DenseMatrix64F FPHTranspose = new DenseMatrix64F(nStates, nMeasurements);
         CommonOps.mult(FP, HTranspose, FPHTranspose);

         DenseMatrix64F HP = new DenseMatrix64F(nMeasurements, nStates);
         CommonOps.mult(H, P, HP);

         DenseMatrix64F HPHTranspose = new DenseMatrix64F(nMeasurements, nMeasurements);
         CommonOps.mult(HP, HTranspose, HPHTranspose);

         DenseMatrix64F HPFTranspose = new DenseMatrix64F(nMeasurements, nStates);
         CommonOps.mult(HP, FTranspose, HPFTranspose);

         DenseMatrix64F HPHTransposePlusRInverse = new DenseMatrix64F(HPHTranspose);
         CommonOps.add(HPHTransposePlusRInverse, R, HPHTransposePlusRInverse);
         CommonOps.invert(HPHTransposePlusRInverse);

         DenseMatrix64F leftMiddleBlock = new DenseMatrix64F(nStates, nMeasurements);
         CommonOps.mult(FPHTranspose, HPHTransposePlusRInverse, leftMiddleBlock);
         DenseMatrix64F middleBlock = new DenseMatrix64F(nStates, nStates);
         CommonOps.mult(leftMiddleBlock, HPFTranspose, middleBlock);
         CommonOps.scale(-1.0, middleBlock);

         DenseMatrix64F convergedP = new DenseMatrix64F(FPFTranspose);
         CommonOps.add(convergedP, middleBlock, convergedP);
         CommonOps.add(convergedP, Q, convergedP);

         printIfDebug("P = " + P);
         printIfDebug("convergedP = " + convergedP);

         EjmlUnitTests.assertEquals(P, convergedP, 1e-6);
      }

      {
         // Check that, after update, P = (I - KH) * (FPF' + Q)
         kalmanFilter.update(y);

         DenseMatrix64F P = kalmanFilter.getCovariance();

         DenseMatrix64F FP = new DenseMatrix64F(nStates, nStates);
         CommonOps.mult(F, P, FP);

         DenseMatrix64F FTranspose = new DenseMatrix64F(F);
         CommonOps.transpose(FTranspose);

         DenseMatrix64F FPFTranspose = new DenseMatrix64F(nStates, nStates);
         CommonOps.mult(FP, FTranspose, FPFTranspose);

         DenseMatrix64F K = kalmanFilter.getKGain();

         printIfDebug("K = " + K);
         printIfDebug("H = " + H);

         DenseMatrix64F KH = new DenseMatrix64F(nStates, nStates);
         CommonOps.mult(K, H, KH);

         DenseMatrix64F IMinusKH = CommonOps.identity(nStates);
         CommonOps.subtract(IMinusKH, KH, IMinusKH);

         DenseMatrix64F FPFTransposePlusQ = new DenseMatrix64F(nStates, nStates);
         CommonOps.add(FPFTranspose, Q, FPFTransposePlusQ);

         DenseMatrix64F convergedP = new DenseMatrix64F(nStates, nStates);
         CommonOps.mult(IMinusKH, FPFTransposePlusQ, convergedP);

         printIfDebug("P = " + P);
         printIfDebug("convergedP = " + convergedP);

         EjmlUnitTests.assertEquals(P, convergedP, 1e-6);
      }

      {
         // Do it again using the built in method:

         DenseMatrix64F covariance = kalmanFilter.getCovariance();
         DenseMatrix64F kMatrix = kalmanFilter.getKGain();

         x = RandomMatrices.createRandom(nStates, 1, random);
         P = RandomMatrices.createSymmPosDef(nStates, random);

         kalmanFilter.setState(x, P);

         kalmanFilter.computeSteadyStateGainAndCovariance(1000);

         DenseMatrix64F convergedCovarianceAgain = kalmanFilter.getCovariance();
         DenseMatrix64F convergedKAgain = kalmanFilter.getKGain();

         EjmlUnitTests.assertEquals(covariance, convergedCovarianceAgain, 1e-6);
         EjmlUnitTests.assertEquals(kMatrix, convergedKAgain, 1e-6);
      }
   }

   private void createRandomParameters(int nStates, int nInputs, int nMeasurements)
   {
      F = RandomMatrices.createRandom(nStates, nStates, random);
      G = RandomMatrices.createRandom(nStates, nInputs, random);
      H = RandomMatrices.createRandom(nMeasurements, nStates, random);
      Q = RandomMatrices.createSymmPosDef(nStates, random);
      R = RandomMatrices.createSymmPosDef(nMeasurements, random);
      x = RandomMatrices.createRandom(nStates, 1, random);
      P = RandomMatrices.createSymmPosDef(nStates, random);
      u = RandomMatrices.createRandom(nInputs, 1, random);
      y = RandomMatrices.createRandom(nMeasurements, 1, random);
   }

   private void printIfDebug(String toPrint)
   {
      if (DEBUG)
      {
         System.out.println(toPrint);
      }
   }

   private static class YoVariableToDataBufferAdder implements YoVariableRegistryChangedListener
   {
      private final DataBuffer dataBuffer;

      public YoVariableToDataBufferAdder(DataBuffer dataBuffer)
      {
         this.dataBuffer = dataBuffer;
      }

      public void yoVariableWasRegistered(YoVariableRegistry registry, YoVariable registeredYoVariable)
      {
         try
         {
            dataBuffer.addVariable(registeredYoVariable);
         }
         catch (RepeatDataBufferEntryException e)
         {
            e.printStackTrace();
         }
      }

      public void yoVariableRegistryWasAdded(YoVariableRegistry addedYoVariableRegistry)
      {
         try
         {
            dataBuffer.addVariables(addedYoVariableRegistry.getAllVariablesIncludingDescendants());
         }
         catch (RepeatDataBufferEntryException e)
         {
            e.printStackTrace();
         }
      }

      public void yoVariableRegistryWasCleared(YoVariableRegistry clearedYoVariableRegistry)
      {
      }
   }
}
