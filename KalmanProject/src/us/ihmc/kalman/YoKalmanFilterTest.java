package us.ihmc.kalman;

import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Before;
import org.junit.Test;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.EjmlUnitTests;
import org.ejml.ops.RandomMatrices;

import com.yobotics.simulationconstructionset.DataBuffer;
import com.yobotics.simulationconstructionset.DataBuffer.RepeatDataBufferEntryException;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class YoKalmanFilterTest
{
   private Random random;
   private YoVariableRegistry parentRegistry;
   
   private final int nStates = 8;
   private final int nInputs = 5;
   private final int nMeasurements = 3;

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

   @Test
   public void testCompareToSimple()
   {
      KalmanFilterSimple kalmanFilterSimple = new KalmanFilterSimple();
      YoKalmanFilter yoKalmanFilter = new YoKalmanFilter("yo", nStates, nInputs, nMeasurements, parentRegistry);
      KalmanFilter[] kalmanFilters = {kalmanFilterSimple, yoKalmanFilter};

      for (KalmanFilter kalmanFilter : kalmanFilters)
      {
         kalmanFilter.configure(F, G, H);
         kalmanFilter.setProcessNoiseCovariance(Q);
         kalmanFilter.setMeasurementNoiseCovariance(R);
         kalmanFilter.setState(x, P);
         kalmanFilter.predict(u);
         kalmanFilter.update(y);
      }

      EjmlUnitTests.assertEquals(kalmanFilters[0].getState(), kalmanFilters[1].getState(), 1e-8);
      EjmlUnitTests.assertEquals(kalmanFilters[0].getCovariance(), kalmanFilters[1].getCovariance(), 1e-8);
   }

   @Test(expected=RuntimeException.class)
   public void testNotProcessCovarianceSymmetricPositiveDefinite1()
   {
      YoKalmanFilter yoKalmanFilter = null;
      try
      {
         yoKalmanFilter = new YoKalmanFilter("yo", nStates, nInputs, nMeasurements, parentRegistry);
         yoKalmanFilter.setCheckPositiveDefiniteness(true);
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
   
   @Test(expected=RuntimeException.class)
   public void testMeasurementCovarianceNotSymmetricPositiveDefinite1()
   {
      YoKalmanFilter yoKalmanFilter = null;
      try
      {
         yoKalmanFilter = new YoKalmanFilter("yo", nStates, nInputs, nMeasurements, parentRegistry);
         yoKalmanFilter.setCheckPositiveDefiniteness(true);
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

   @Test(expected=ArrayIndexOutOfBoundsException.class)
   public void testWrongSize()
   {
      YoKalmanFilter yoKalmanFilter = new YoKalmanFilter("yo", nStates, nInputs, nMeasurements, parentRegistry);
      yoKalmanFilter.setProcessNoiseCovariance(new DenseMatrix64F(nStates + 1, nStates + 1));
   }
   
   @Test
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

      YoKalmanFilter yoKalmanFilter1 = new YoKalmanFilter("yo1", nStates, nInputs, nMeasurements, parentRegistry);
      YoKalmanFilter yoKalmanFilter2 = new YoKalmanFilter("yo2", nStates, nInputs, nMeasurements, new YoVariableRegistry("testRegistry2"));
      KalmanFilter[] kalmanFilters = {yoKalmanFilter1, yoKalmanFilter2};

      DataBuffer dataBuffer = new DataBuffer();
      dataBuffer.addVariables(parentRegistry.getAllVariablesIncludingDescendants());

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
      dataBuffer.goToInPoint();
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
}
