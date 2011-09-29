package us.ihmc.kalman;

import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Before;
import org.junit.Test;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.EjmlUnitTests;
import org.ejml.ops.RandomMatrices;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class YoKalmanFilterTest
{
   private Random random;
   private YoVariableRegistry parentRegistry;

   @Before
   public void setUp()
   {
      random = new Random(1776L);
      parentRegistry = new YoVariableRegistry("testRegistry");
   }

   @Test
   public void testCompareToSimple()
   {
      int nStates = 8;
      int nInputs = 5;
      int nMeasurements = 3;

      KalmanFilterSimple kalmanFilterSimple = new KalmanFilterSimple();
      YoKalmanFilter yoKalmanFilter = new YoKalmanFilter("yo", nStates, nInputs, nMeasurements, parentRegistry);
      KalmanFilter[] kalmanFilters = {kalmanFilterSimple, yoKalmanFilter};

      DenseMatrix64F F = RandomMatrices.createRandom(nStates, nStates, random);
      DenseMatrix64F G = RandomMatrices.createRandom(nStates, nInputs, random);
      DenseMatrix64F H = RandomMatrices.createRandom(nMeasurements, nStates, random);
      DenseMatrix64F Q = RandomMatrices.createSymmPosDef(nStates, random);
      DenseMatrix64F R = RandomMatrices.createSymmPosDef(nMeasurements, random);
      DenseMatrix64F x = RandomMatrices.createRandom(nStates, 1, random);
      DenseMatrix64F P = RandomMatrices.createSymmPosDef(nStates, random);
      DenseMatrix64F u = RandomMatrices.createRandom(nInputs, 1, random);
      DenseMatrix64F y = RandomMatrices.createRandom(nMeasurements, 1, random);

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
      DenseMatrix64F Q = null;
      try
      {
         int nStates = 8;
         int nInputs = 5;
         int nMeasurements = 3;
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
      DenseMatrix64F R = null;
      try
      {
         int nStates = 8;
         int nInputs = 5;
         int nMeasurements = 3;
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
}
