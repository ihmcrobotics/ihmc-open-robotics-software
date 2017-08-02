package us.ihmc.commonWalkingControlModules.dynamicReachability;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.SmoothCapturePointTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SmoothCoMIntegrationToolsTest
{
   private static final int nTests = 20;
   private static final double omega0 = 3.4;
   private static final double EPSILON = 10e-6;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private Random random = new Random();
   
   String namePrefix = "SmoothCoMIntegrationToolsTest";
   
   SmoothCoMIntegrationTools comM = new SmoothCoMIntegrationTools();
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMatricesCoMPrime3DLinear()
   {
      // Linear polynomial: y(x) = a0 + a1*x
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 2;
      YoFrameTrajectory3D linear3D = new YoFrameTrajectory3D(namePrefix + "Linear", numberOfCoefficients, worldFrame, registry);
      
      for(int i = 0; i < nTests; i++)
      {
         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();
                    
         FramePoint cmp0 = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FramePoint cmpFinal = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         
         linear3D.setLinear(t0, tFinal, cmp0, cmpFinal);
         
         double time = t0 + Math.random() * (tFinal - t0);
         
         DenseMatrix64F alphaCoMPrimeAutomatic = new DenseMatrix64F(3, 3 * numberOfCoefficients);
         DenseMatrix64F alphaCoMPrimeManual = new DenseMatrix64F(3, 3 * numberOfCoefficients);
      
         SmoothCoMIntegrationTools.calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(omega0, time, alphaCoMPrimeAutomatic, 0, linear3D);
         calculateAlphaCoMPrime3DByHandLinear(omega0 , time, tFinal, alphaCoMPrimeManual);
         
//         PrintTools.debug("A linear calc: " + alphaCoMPrimeAutomatic.toString());
//         PrintTools.debug("A linear test: " + alphaCoMPrimeManual.toString());

         for(int j = 0; j < alphaCoMPrimeAutomatic.getNumCols(); j++)
         {
            for(int k = 0; k < alphaCoMPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(alphaCoMPrimeAutomatic.get(k, j), alphaCoMPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F betaCoMPrimeAutomatic = new DenseMatrix64F(3, 3 * numberOfCoefficients);
         DenseMatrix64F betaCoMPrimeManual = new DenseMatrix64F(3, 3 * numberOfCoefficients);
      
         SmoothCoMIntegrationTools.calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(omega0, time, betaCoMPrimeAutomatic, 0, linear3D);
         calculateBetaCoMPrime3DByHandLinear(omega0 , time, tFinal, betaCoMPrimeManual);
         
//         PrintTools.debug("B linear calc: " + betaCoMPrimeAutomatic.toString());
//         PrintTools.debug("B linear test: " + betaCoMPrimeManual.toString());
         
         for(int j = 0; j < betaCoMPrimeAutomatic.getNumCols(); j++)
         {
            for(int k = 0; k < betaCoMPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(betaCoMPrimeAutomatic.get(k, j), betaCoMPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F gammaCoMPrimeAutomatic = new DenseMatrix64F(1, 1);
         DenseMatrix64F gammaCoMPrimeManual = new DenseMatrix64F(1, 1);
      
         SmoothCoMIntegrationTools.calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(omega0, time, gammaCoMPrimeAutomatic, 0, linear3D);
         calculateGammaCoMPrime3DByHandLinear(omega0 , time, tFinal, gammaCoMPrimeManual);
         
//         PrintTools.debug("C linear calc: " + gammaCoMPrimeAutomatic.toString());
//         PrintTools.debug("C linear test: " + gammaCoMPrimeManual.toString());
         
         assertEquals(gammaCoMPrimeAutomatic.get(0), gammaCoMPrimeManual.get(0), EPSILON);
         
         DenseMatrix64F deltaCoMPrimeAutomatic = new DenseMatrix64F(3, 3 * numberOfCoefficients);
         DenseMatrix64F deltaCoMPrimeManual = new DenseMatrix64F(3, 3 * numberOfCoefficients);
    
         SmoothCoMIntegrationTools.calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(omega0, time, deltaCoMPrimeAutomatic, 0, linear3D);
         calculateDeltaCoMPrime3DByHandLinear(omega0 , time, tFinal, deltaCoMPrimeManual);
       
//         PrintTools.debug("D linear calc: " + deltaCoMPrimeAutomatic.toString());
//         PrintTools.debug("D linear test: " + deltaCoMPrimeManual.toString());
       
         for(int j = 0; j < deltaCoMPrimeAutomatic.getNumCols(); j++)
         {
            for(int k = 0; k < deltaCoMPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(deltaCoMPrimeAutomatic.get(k, j), deltaCoMPrimeManual.get(k, j), EPSILON);
            }
         }
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDMatricesCoMPrime3DLinear()
   {
      // Linear polynomial: y(x) = a0 + a1*x
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 2;
      YoFrameTrajectory3D linear3D = new YoFrameTrajectory3D(namePrefix + "Linear", numberOfCoefficients, worldFrame, registry);
      
      for(int i = 0; i < nTests; i++)
      {
         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();
                    
         FramePoint cmp0 = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FramePoint cmpFinal = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         
         linear3D.setLinear(t0, tFinal, cmp0, cmpFinal);
         
         double time = t0 + Math.random() * (tFinal - t0);
         
         DenseMatrix64F dAlphaCoMPrimeAutomatic = new DenseMatrix64F(3, 3 * numberOfCoefficients);
         DenseMatrix64F dAlphaCoMPrimeManual = new DenseMatrix64F(3, 3 * numberOfCoefficients);
      
         SmoothCoMIntegrationTools.calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(omega0, time, dAlphaCoMPrimeAutomatic, 1, linear3D);
         calculateDAlphaCoMPrime3DByHandLinear(omega0 , time, tFinal, dAlphaCoMPrimeManual);
         
//         PrintTools.debug("dA linear calc: " + dAlphaCoMPrimeAutomatic.toString());
//         PrintTools.debug("dA linear test: " + dAlphaCoMPrimeManual.toString());

         for(int j = 0; j < dAlphaCoMPrimeAutomatic.getNumCols(); j++)
         {
            for(int k = 0; k < dAlphaCoMPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(dAlphaCoMPrimeAutomatic.get(k, j), dAlphaCoMPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F dBetaCoMPrimeAutomatic = new DenseMatrix64F(3, 3 * numberOfCoefficients);
         DenseMatrix64F dBetaCoMPrimeManual = new DenseMatrix64F(3, 3 * numberOfCoefficients);
      
         SmoothCoMIntegrationTools.calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(omega0, time, dBetaCoMPrimeAutomatic, 1, linear3D);
         calculateDBetaCoMPrime3DByHandLinear(omega0 , time, tFinal, dBetaCoMPrimeManual);
         
//         PrintTools.debug("dB linear calc: " + dBetaCoMPrimeAutomatic.toString());
//         PrintTools.debug("dB linear test: " + dBetaCoMPrimeManual.toString());
         
         for(int j = 0; j < dBetaCoMPrimeAutomatic.getNumCols(); j++)
         {
            for(int k = 0; k < dBetaCoMPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(dBetaCoMPrimeAutomatic.get(k, j), dBetaCoMPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F dGammaCoMPrimeAutomatic = new DenseMatrix64F(1, 1);
         DenseMatrix64F dGammaCoMPrimeManual = new DenseMatrix64F(1, 1);
      
         SmoothCoMIntegrationTools.calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(omega0, time, dGammaCoMPrimeAutomatic, 1, linear3D);
         calculateDGammaCoMPrime3DByHandLinear(omega0 , time, tFinal, dGammaCoMPrimeManual);
         
//         PrintTools.debug("dC linear calc: " + dGammaCoMPrimeAutomatic.toString());
//         PrintTools.debug("dC linear test: " + dGammaCoMPrimeManual.toString());
         
         assertEquals(dGammaCoMPrimeAutomatic.get(0), dGammaCoMPrimeManual.get(0), EPSILON);
         
         DenseMatrix64F dDeltaCoMPrimeAutomatic = new DenseMatrix64F(1, 1);
         DenseMatrix64F dDeltaCoMPrimeManual = new DenseMatrix64F(1, 1);
      
         SmoothCoMIntegrationTools.calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(omega0, time, dDeltaCoMPrimeAutomatic, 1, linear3D);
         calculateDDeltaCoMPrime3DByHandLinear(omega0 , time, tFinal, dDeltaCoMPrimeManual);
         
//         PrintTools.debug("dD linear calc: " + dDeltaCoMPrimeAutomatic.toString());
//         PrintTools.debug("dD linear test: " + dDeltaCoMPrimeManual.toString());
         
         assertEquals(dDeltaCoMPrimeAutomatic.get(0), dDeltaCoMPrimeManual.get(0), EPSILON);
      }
   }
   
//   @ContinuousIntegrationTest(estimatedDuration = 0.0)
//   @Test(timeout = 30000)
//   public void testMatricesCoMPrime3DCubic()
//   {
//      // Cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
//      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
//      int numberOfCoefficients = 4;
//      YoFrameTrajectory3D cubic3D = new YoFrameTrajectory3D(namePrefix + "Cubic", numberOfCoefficients, worldFrame, registry);
//      
//      for(int i = 0; i < nTests; i++)
//      {
//         double scaleTFinal = 1.0 / Math.random();
//         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();
//                    
//         FramePoint cmp0 = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
//         FramePoint cmpFinal = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
//         
//         FrameVector cmpD0 = new FrameVector(worldFrame, new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
//         FrameVector cmpDFinal = new FrameVector(worldFrame, new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
//         
//         cubic3D.setCubic(t0, tFinal, cmp0, cmpD0, cmpFinal,cmpDFinal);
//         
//         double time = t0 + Math.random() * (tFinal - t0);
//         
//         DenseMatrix64F alphaCoMPrimeAutomatic = new DenseMatrix64F(3, 3 * numberOfCoefficients);
//         DenseMatrix64F alphaCoMPrimeManual = new DenseMatrix64F(3, 3 * numberOfCoefficients);
//      
//         SmoothCoMIntegrationTools.calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(omega0, time, alphaCoMPrimeAutomatic, 0, cubic3D);
//         calculateAlphaCoMPrime3DByHandCubic(omega0 , time, tFinal, alphaCoMPrimeManual);
//         
////         PrintTools.debug("A cubic calc: " + alphaCoMPrimeAutomatic.toString());
////         PrintTools.debug("A cubic test: " + alphaCoMPrimeManual.toString());
//
//         for(int j = 0; j < alphaCoMPrimeAutomatic.getNumCols(); j++)
//         {
//            for(int k = 0; k < alphaCoMPrimeAutomatic.getNumRows(); k++)
//            {
//               assertEquals(alphaCoMPrimeAutomatic.get(k, j), alphaCoMPrimeManual.get(k, j), EPSILON);
//            }
//         }
//         
//         DenseMatrix64F betaCoMPrimeAutomatic = new DenseMatrix64F(3, 3 * numberOfCoefficients);
//         DenseMatrix64F betaCoMPrimeManual = new DenseMatrix64F(3, 3 * numberOfCoefficients);
//      
//         SmoothCoMIntegrationTools.calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(omega0, time, betaCoMPrimeAutomatic, 0, cubic3D);
//         calculateBetaCoMPrime3DByHandCubic(omega0 , time, tFinal, betaCoMPrimeManual);
//         
////         PrintTools.debug("B cubic calc: " + betaCoMPrimeAutomatic.toString());
////         PrintTools.debug("B cubic test: " + betaCoMPrimeManual.toString());
//         
//         for(int j = 0; j < betaCoMPrimeAutomatic.getNumCols(); j++)
//         {
//            for(int k = 0; k < betaCoMPrimeAutomatic.getNumRows(); k++)
//            {
//               assertEquals(betaCoMPrimeAutomatic.get(k, j), betaCoMPrimeManual.get(k, j), EPSILON);
//            }
//         }
//         
//         DenseMatrix64F gammaCoMPrimeAutomatic = new DenseMatrix64F(1, 1);
//         DenseMatrix64F gammaCoMPrimeManual = new DenseMatrix64F(1, 1);
//      
//         SmoothCoMIntegrationTools.calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(omega0, time, gammaCoMPrimeAutomatic, 0, cubic3D);
//         calculateGammaCoMPrime3DByHandCubic(omega0 , time, tFinal, gammaCoMPrimeManual);
//         
////         PrintTools.debug("C cubic calc: " + gammaCoMPrimeAutomatic.toString());
////         PrintTools.debug("C cubic test: " + gammaCoMPrimeManual.toString());
//         
//         assertEquals(gammaCoMPrimeAutomatic.get(0), gammaCoMPrimeManual.get(0), EPSILON);
//      }
//   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCalculateCoMPositionAndVelocityOnSegment3DLinear()
   {
   // Linear polynomial: y(x) = a0 + a1*x
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 2;
      YoFrameTrajectory3D linear3D = new YoFrameTrajectory3D(namePrefix + "Linear", numberOfCoefficients, worldFrame, registry);
      
      for(int i = 0; i < nTests; i++)
      {
//         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + Math.random(); //scaleTFinal * Math.random();
                    
         FramePoint cmp0 = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FramePoint cmpFinal = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         
         linear3D.setLinear(t0, tFinal, cmp0, cmpFinal);
         
         double time = t0 + Math.random() * (tFinal - t0);                
         
         FramePoint icpPositionDesiredFinal = new FramePoint(worldFrame, cmpFinal.getPoint());
         FramePoint comPositionDesiredFinal = new FramePoint(worldFrame, cmpFinal.getPoint());
         
         FramePoint icpPositionDesiredInitial = new FramePoint(worldFrame);
         CapturePointTools.computeDesiredCapturePointPosition(omega0, t0, icpPositionDesiredFinal, linear3D, icpPositionDesiredInitial);

         // Position
         FramePoint comPositionDesiredCurrent = new FramePoint(worldFrame);
         FramePoint comPositionDesiredInitial = new FramePoint(worldFrame);
         FramePoint comPositionDesiredCurrentByHand = new FramePoint(worldFrame);
         FramePoint comPositionDesiredInitialByHand = new FramePoint(worldFrame);
         
         SmoothCoMIntegrationTools.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, linear3D, icpPositionDesiredFinal, comPositionDesiredFinal, comPositionDesiredCurrent);
         SmoothCoMIntegrationTools.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, t0, 0, linear3D, icpPositionDesiredFinal, comPositionDesiredFinal, comPositionDesiredInitial);
         calculateCoMPositionByHand3DLinear(omega0, time, linear3D, icpPositionDesiredFinal, comPositionDesiredFinal, comPositionDesiredCurrentByHand);
         calculateCoMPositionByHand3DLinear(omega0, t0, linear3D, icpPositionDesiredFinal, comPositionDesiredFinal, comPositionDesiredInitialByHand);
         
//         PrintTools.debug("Test " + i);
//         PrintTools.debug("Time = " + time + ", [" + t0 + ", " + tFinal + "]");
//         PrintTools.debug("Ini CMP: " + cmp0.toString());
//         PrintTools.debug("End CMP: " + cmpFinal.toString());
//         PrintTools.debug("Ini ICP: " + icpPositionDesiredInitial.toString());
//         PrintTools.debug("End ICP: " + icpPositionDesiredFinal.toString());
//         PrintTools.debug("End CoM: " + comPositionDesiredFinal.toString());
//         PrintTools.debug("Ini CoM calc: " + comPositionDesiredInitial.toString());
//         PrintTools.debug("Ini CoM hand: " + comPositionDesiredInitialByHand.toString());
//         PrintTools.debug("CoM pos calc: " + comPositionDesiredCurrent.toString());
//         PrintTools.debug("CoM pos hand: " + comPositionDesiredCurrentByHand.toString());
//         PrintTools.debug("");

         EuclidCoreTestTools.assertTuple3DEquals("", comPositionDesiredCurrent.getPoint(), comPositionDesiredCurrentByHand.getPoint(), EPSILON);
         
         // Velocity
         FrameVector comVelocityDesiredCurrent = new FrameVector(worldFrame);
         FrameVector comVelocityDesiredCurrentByHand = new FrameVector(worldFrame);
         
         SmoothCoMIntegrationTools.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 1, linear3D, icpPositionDesiredFinal, comPositionDesiredFinal, comVelocityDesiredCurrent);
         calculateCoMVelocityByHand3DLinear(omega0, time, linear3D, icpPositionDesiredFinal, comPositionDesiredFinal, comVelocityDesiredCurrentByHand);
         
//         PrintTools.debug("CoM vel calc: " + comVelocityDesiredCurrent.toString());
//         PrintTools.debug("CoM vel hand: " + comVelocityDesiredCurrentByHand.toString());
//         PrintTools.debug("");
         
         EuclidCoreTestTools.assertTuple3DEquals("", comVelocityDesiredCurrent.getVectorCopy(), comVelocityDesiredCurrentByHand.getVectorCopy(), EPSILON);
         
         // Dynamics
         linear3D.compute(time);
         FramePoint icpPositionDesiredCurrent = new FramePoint(worldFrame);
         SmoothCapturePointTools.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, linear3D, icpPositionDesiredFinal, icpPositionDesiredCurrent);
         
         FrameVector comVelocityDesiredCurrentDynamics = new FrameVector(worldFrame);
         comVelocityDesiredCurrentDynamics.subAndScale(omega0, icpPositionDesiredCurrent, comPositionDesiredCurrent);
         
         EuclidCoreTestTools.assertTuple3DEquals("", comVelocityDesiredCurrent.getVectorCopy(), comVelocityDesiredCurrentDynamics.getVectorCopy(), EPSILON);
         
         FrameVector comVelocityDesiredCurrentDynamicsByHand = new FrameVector(worldFrame);
         comVelocityDesiredCurrentDynamicsByHand.subAndScale(omega0, icpPositionDesiredCurrent, comPositionDesiredCurrentByHand);
         
         EuclidCoreTestTools.assertTuple3DEquals("", comVelocityDesiredCurrentByHand.getVectorCopy(), comVelocityDesiredCurrentDynamicsByHand.getVectorCopy(), EPSILON);
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCalculateCoMInsanityCheck()
   {
   // Linear polynomial: y(x) = a0 + a1*x
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 2;
      YoFrameTrajectory3D linear3D = new YoFrameTrajectory3D(namePrefix + "Linear", numberOfCoefficients, worldFrame, registry);
      
      double t0 = 0.0, tFinal = 1.0;
                 
      FramePoint cmp0 = new FramePoint(worldFrame, new Point3D(0.0, 0.0, 0.0));
      FramePoint cmpFinal = new FramePoint(worldFrame, new Point3D(1.0, 1.0, 0.0));
      
      linear3D.setLinear(t0, tFinal, cmp0, cmpFinal);
            
      FramePoint icpPositionDesiredFinal = new FramePoint(worldFrame, cmpFinal.getPoint());
      FramePoint comPositionDesiredInitial = new FramePoint(worldFrame, cmp0.getPoint());
      
      FramePoint icpPositionDesiredInitial = new FramePoint(worldFrame);
      CapturePointTools.computeDesiredCapturePointPosition(omega0, t0, icpPositionDesiredFinal, linear3D, icpPositionDesiredInitial);

      // Position
      FramePoint comPositionDesiredFinal = new FramePoint(worldFrame);
      FramePoint comPositionDesiredInitialByHand = new FramePoint(worldFrame);
      
      SmoothCoMIntegrationTools.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, linear3D.getFinalTime(), 0, linear3D, icpPositionDesiredFinal, comPositionDesiredInitial, comPositionDesiredFinal);
//      calculateCoMPositionByHand3DLinear(omega0, linear3D.getInitialTime(), linear3D, icpPositionDesiredFinal, comPositionDesiredFinal, comPositionDesiredInitialByHand);
      
      PrintTools.debug("Insanity Check");
      PrintTools.debug("Time = " + "[" + t0 + ", " + tFinal + "]");
      PrintTools.debug("CMP: " + cmp0.getPoint().toString() + " --> " + cmpFinal.getPoint().toString());
      PrintTools.debug("ICP: " + icpPositionDesiredInitial.getPoint().toString() + " --> " + icpPositionDesiredFinal.getPoint().toString());
      PrintTools.debug("CoM: " + comPositionDesiredInitial.getPoint().toString() + " --> " + comPositionDesiredFinal.getPoint().toString());
      PrintTools.debug("CoM: " + comPositionDesiredInitialByHand.getPoint().toString() + " --> " + comPositionDesiredInitial.getPoint().toString());
      PrintTools.debug("");

      EuclidCoreTestTools.assertTuple3DEquals("", comPositionDesiredInitial.getPoint(), comPositionDesiredInitialByHand.getPoint(), EPSILON);
   }
   
//   @ContinuousIntegrationTest(estimatedDuration = 0.0)
//   @Test(timeout = 30000)
//   public void testCalculateICPPositionAndVelocityOnSegment3DCubic()
//   {
//      // Cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
//      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
//      int numberOfCoefficients = 4;
//      YoFrameTrajectory3D cubic3D = new YoFrameTrajectory3D(namePrefix + "Cubic", numberOfCoefficients, worldFrame, registry);
//      
//      for(int i = 0; i < nTests; i++)
//      {
//         double scaleTFinal = 1.0 / Math.random();
//         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();
//                    
//         FramePoint cmp0 = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
//         FramePoint cmpFinal = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
//         
//         // !!! TESTING WITH 0 VELOCITIES !!!
//         FrameVector cmpD0 = new FrameVector(worldFrame);
//         FrameVector cmpDFinal = new FrameVector(worldFrame);
//         
//         cubic3D.setCubic(t0, tFinal, cmp0, cmpD0, cmpFinal, cmpDFinal);
//         
//         double time = t0 + Math.random() * (tFinal - t0);
//                  
//         FramePoint icpPositionDesiredFinal = new FramePoint(worldFrame, cmpFinal.getPoint());
//         FramePoint comPositionDesiredFinal = new FramePoint(worldFrame, cmpFinal.getPoint());
//         
//         // Position
//         FramePoint comPositionDesiredCurrent = new FramePoint(worldFrame);
//         FramePoint comPositionDesiredCurrentByHand = new FramePoint(worldFrame);
//         
//         SmoothCoMIntegrationTools.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, cubic3D, icpPositionDesiredFinal, comPositionDesiredFinal, comPositionDesiredCurrent);
//         calculateCoMPositionByHand3DCubic(omega0, time, cubic3D, icpPositionDesiredFinal, comPositionDesiredCurrentByHand);
//         
////         PrintTools.debug("CoM pos calc: " + comPositionDesiredCurrent.toString());
////         PrintTools.debug("CoM pos hand: " + comPositionDesiredCurrentByHand.toString());
//
//         EuclidCoreTestTools.assertTuple3DEquals("", comPositionDesiredCurrent.getPoint(), comPositionDesiredCurrentByHand.getPoint(), EPSILON);
//         
//         // Velocity
//         FrameVector comVelocityDesiredCurrent = new FrameVector(worldFrame);
//         FrameVector comVelocityDesiredCurrentByHand = new FrameVector(worldFrame);
//         
//         SmoothCoMIntegrationTools.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 1, cubic3D, icpPositionDesiredFinal, comPositionDesiredFinal, comVelocityDesiredCurrent);
//         calculateCoMVelocityByHand3DCubic(omega0, time, cubic3D, icpPositionDesiredFinal, comVelocityDesiredCurrentByHand);
//         
////         PrintTools.debug("CoM vel calc: " + comVelocityDesiredCurrent.toString());
////         PrintTools.debug("CoM vel hand: " + comVelocityDesiredCurrentByHand.toString());
//         
//         EuclidCoreTestTools.assertTuple3DEquals("", comVelocityDesiredCurrent.getVectorCopy(), comVelocityDesiredCurrentByHand.getVectorCopy(), EPSILON);
//         
//         // Dynamics
////         cubic3D.compute(time);
////         FramePoint cmpPositionDesiredCurrent = new FramePoint(worldFrame, cubic3D.getPosition());
////         
////         FrameVector icpVelocityDesiredCurrentDynamics = new FrameVector(worldFrame);
////         icpVelocityDesiredCurrentDynamics.subAndScale(omega0, comPositionDesiredCurrent, cmpPositionDesiredCurrent);
////         
////         EuclidCoreTestTools.assertTuple3DEquals("", comVelocityDesiredCurrent.getVectorCopy(), icpVelocityDesiredCurrentDynamics.getVectorCopy(), EPSILON);
//      }
//   }
   
   public static void calculateCoMPositionByHand3DLinear(double omega0, double time, YoFrameTrajectory3D linear3D, FramePoint icpPositionDesiredFinal, FramePoint comPositionDesiredFinal, FramePoint comPositionDesiredCurrent)
   {      
      linear3D.compute(linear3D.getInitialTime());
      FramePoint cmpRefInit = new FramePoint(linear3D.getFramePosition());
      
      linear3D.compute(linear3D.getFinalTime());
      FramePoint cmpRefFinal = new FramePoint(linear3D.getFramePosition());
      
      
      double timeFinal = linear3D.getFinalTime();
      
      double at = Math.exp(omega0 * (timeFinal - time)) - 1;
      double bt = timeFinal * Math.exp(omega0 * (timeFinal - time)) - time - 1.0/omega0 * Math.exp(omega0 * (timeFinal - time)) + 1.0/omega0;
      double ct = 0.5 * Math.exp(omega0 * (timeFinal - time)) - 0.5 * Math.exp(omega0 * (time - timeFinal));
      
//      PrintTools.debug("at = " + at);
//      PrintTools.debug("bt = " + bt);
//      PrintTools.debug("ct = " + ct);
      
      comPositionDesiredCurrent.setToZero();
      for(int i = 0; i < 3; i++)
      {
         double a0 = (1.0 - 1.0/(omega0*timeFinal)) * cmpRefInit.getElement(i) + 1.0/(omega0*timeFinal) * cmpRefFinal.getElement(i);
         double b0 = 1.0/timeFinal * (cmpRefFinal.getElement(i) - cmpRefInit.getElement(i));
         double c0 = 1.0/(omega0*timeFinal) * cmpRefInit.getElement(i) - (1.0 + 1.0/(omega0*timeFinal)) * cmpRefFinal.getElement(i) + icpPositionDesiredFinal.getElement(i);
         
//         PrintTools.debug("a0 = " + a0);
//         PrintTools.debug("b0 = " + b0);
//         PrintTools.debug("c0 = " + c0);
         comPositionDesiredCurrent.setElement(i, Math.exp(omega0 * (timeFinal - time)) * comPositionDesiredFinal.getElement(i) - (a0*at + b0*bt + c0*ct));
      }
   }
   
//   public static void calculateCoMPositionByHand3DCubic(double omega0, double time, YoFrameTrajectory3D cubic3D, FramePoint icpPositionDesiredFinal, FramePoint icpPositionDesiredCurrent)
//   {      
//      cubic3D.compute(cubic3D.getInitialTime());
//      FramePoint cmpRefInit = new FramePoint(cubic3D.getFramePosition());
//      
//      cubic3D.compute(cubic3D.getFinalTime());
//      FramePoint cmpRefFinal = new FramePoint(cubic3D.getFramePosition());
//      
//      double timeFinal = cubic3D.getFinalTime();
//      
//      double sigmat = calculateSigmaCubic(time, timeFinal, omega0);
//      double sigmaT = calculateSigmaCubic(timeFinal, timeFinal, omega0);
//      
//      double alpha = (1.0 - sigmat - Math.exp(omega0*(time-timeFinal)) * (1.0 - sigmaT));
//      double beta = (sigmat - Math.exp(omega0*(time-timeFinal)) * sigmaT);
//      double gamma =  Math.exp(omega0*(time-timeFinal));
//      
//      icpPositionDesiredCurrent.setToZero();
//      icpPositionDesiredCurrent.scaleAdd(1.0, icpPositionDesiredCurrent.getPointCopy(), alpha, cmpRefInit.getPointCopy());
//      icpPositionDesiredCurrent.scaleAdd(1.0, icpPositionDesiredCurrent.getPointCopy(), beta, cmpRefFinal.getPointCopy());
//      icpPositionDesiredCurrent.scaleAdd(1.0, icpPositionDesiredCurrent.getPointCopy(), gamma, icpPositionDesiredFinal.getPointCopy());
//   }
   
   public static void calculateCoMVelocityByHand3DLinear(double omega0, double time, YoFrameTrajectory3D linear3D, FramePoint icpPositionDesiredFinal, FramePoint comPositionDesiredFinal, FrameVector comVelocityDesiredCurrent)
   {      
      linear3D.compute(linear3D.getInitialTime());
      FramePoint cmpRefInit = new FramePoint(linear3D.getFramePosition());
      
      linear3D.compute(linear3D.getFinalTime());
      FramePoint cmpRefFinal = new FramePoint(linear3D.getFramePosition());
      
      
      double timeFinal = linear3D.getFinalTime();
      
      double dat = -omega0 * Math.exp(omega0 * (timeFinal - time));
      double dbt = -omega0 * timeFinal * Math.exp(omega0 * (timeFinal - time)) - 1 + Math.exp(omega0 * (timeFinal - time));
      double dct = -0.5 * omega0 * Math.exp(omega0 * (timeFinal - time)) - 0.5 * omega0 * Math.exp(omega0 * (time - timeFinal));
      
//      PrintTools.debug("dat = " + dat);
//      PrintTools.debug("dbt = " + dbt);
//      PrintTools.debug("dct = " + dct);
      
      comVelocityDesiredCurrent.setToZero();
      for(int i = 0; i < 3; i++)
      {
         double a0 = (1.0 - 1.0/(omega0*timeFinal)) * cmpRefInit.getElement(i) + 1.0/(omega0*timeFinal) * cmpRefFinal.getElement(i);
         double b0 = 1.0/timeFinal * (cmpRefFinal.getElement(i) - cmpRefInit.getElement(i));
         double c0 = 1.0/(omega0*timeFinal) * cmpRefInit.getElement(i) - (1.0 + 1.0/(omega0*timeFinal)) * cmpRefFinal.getElement(i) + icpPositionDesiredFinal.getElement(i);
         
//         PrintTools.debug("a0 = " + a0);
//         PrintTools.debug("b0 = " + b0);
//         PrintTools.debug("c0 = " + c0);
         comVelocityDesiredCurrent.setElement(i, -omega0 * Math.exp(omega0 * (timeFinal - time)) * comPositionDesiredFinal.getElement(i) - (a0*dat + b0*dbt + c0*dct));
      }
   }
   
//   public static void calculateCoMVelocityByHand3DCubic(double omega0, double time, YoFrameTrajectory3D cubic3D, FramePoint icpPositionDesiredFinal, FrameVector icpVelocityDesiredCurrent)
//   {      
//      cubic3D.compute(cubic3D.getInitialTime());
//      FramePoint cmpRefInit = new FramePoint(cubic3D.getFramePosition());
//      
//      cubic3D.compute(cubic3D.getFinalTime());
//      FramePoint cmpRefFinal = new FramePoint(cubic3D.getFramePosition());
//      
//      double timeFinal = cubic3D.getFinalTime();
//      
//      double dSigmat = calculateSigmaDotCubic(time, timeFinal, omega0);
//      double sigmaT = calculateSigmaCubic(timeFinal, timeFinal, omega0);
//      
//      double dAlpha = (-dSigmat - omega0 * Math.exp(omega0*(time-timeFinal)) * (1.0 - sigmaT));
//      double dBeta = (dSigmat - omega0 * Math.exp(omega0*(time-timeFinal)) * sigmaT);
//      double dGamma = omega0 * Math.exp(omega0*(time-timeFinal));
//      
//      icpVelocityDesiredCurrent.setToZero();
//      icpVelocityDesiredCurrent.scaleAdd(1.0, icpVelocityDesiredCurrent.getVectorCopy(), dAlpha, cmpRefInit.getPointCopy());
//      icpVelocityDesiredCurrent.scaleAdd(1.0, icpVelocityDesiredCurrent.getVectorCopy(), dBeta, cmpRefFinal.getPointCopy());
//      icpVelocityDesiredCurrent.scaleAdd(1.0, icpVelocityDesiredCurrent.getVectorCopy(), dGamma, icpPositionDesiredFinal.getPointCopy());
//   }
   
//   public static double calculateSigmaLinear(double t, double T, double omega0)
//   {
//      double sigmaLinear = t/T + 1.0/omega0 * 1/T;
//      return sigmaLinear;
//   }
//   
//   public static double calculateSigmaCubic(double t, double T, double omega0)
//   {
//      double f = 3.0 * Math.pow(t, 2) / Math.pow(T, 2) - 2.0 * Math.pow(t, 3) / Math.pow(T, 3);
//      double df = 6.0 * Math.pow(t, 1) / Math.pow(T, 2) - 6.0 * Math.pow(t, 2) / Math.pow(T, 3);
//      double ddf = 6.0 * 1.0 / Math.pow(T, 2) - 12.0 * Math.pow(t, 1) / Math.pow(T, 3);
//      double dddf = -12.0 / Math.pow(T, 3);
//      
//      double sigmaCubic = f + Math.pow(1.0/omega0, 1) * df + Math.pow(1.0/omega0, 2) * ddf + Math.pow(1.0/omega0, 3) * dddf;
//      return sigmaCubic;
//   }
//   
//   public static double calculateSigmaDotLinear(double t, double T, double omega0)
//   {
//      double dSigmaLinear = 1/T;
//      return dSigmaLinear;
//   }
//   
//   public static double calculateSigmaDotCubic(double t, double T, double omega0)
//   {
//      double df = 6.0 * Math.pow(t, 1) / Math.pow(T, 2) - 6.0 * Math.pow(t, 2) / Math.pow(T, 3);
//      double ddf = 6.0 * 1.0 / Math.pow(T, 2) - 12.0 * Math.pow(t, 1) / Math.pow(T, 3);
//      double dddf = -12.0 / Math.pow(T, 3);
//      
//      double dSigmaCubic = df + Math.pow(1.0/omega0, 1) * ddf + Math.pow(1.0/omega0, 2) * dddf;
//      return dSigmaCubic;
//   }
      
      
   public static void calculateAlphaCoMPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F alphaCoMPrimeLinear)
   {
      alphaCoMPrimeLinear.set(0, 0, 1);
      alphaCoMPrimeLinear.set(0, 1, time);
      
      alphaCoMPrimeLinear.set(1, 2, 1);
      alphaCoMPrimeLinear.set(1, 3, time);
      
      alphaCoMPrimeLinear.set(2, 4, 1);
      alphaCoMPrimeLinear.set(2, 5, time);
   }
   
   public static void calculateDAlphaCoMPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F alphaCoMPrimeLinear)
   {
      alphaCoMPrimeLinear.set(0, 0, 0);
      alphaCoMPrimeLinear.set(0, 1, 1);
      
      alphaCoMPrimeLinear.set(1, 2, 0);
      alphaCoMPrimeLinear.set(1, 3, 1);
      
      alphaCoMPrimeLinear.set(2, 4, 0);
      alphaCoMPrimeLinear.set(2, 5, 1);
   }
   
   public static void calculateBetaCoMPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F betaCoMPrimeLinear)
   {
      betaCoMPrimeLinear.set(0, 0, Math.exp(omega0 * (timeTotal - time)) * 1);
      betaCoMPrimeLinear.set(0, 1, Math.exp(omega0 * (timeTotal - time)) * timeTotal);
      
      betaCoMPrimeLinear.set(1, 2, Math.exp(omega0 * (timeTotal - time)) * 1);
      betaCoMPrimeLinear.set(1, 3, Math.exp(omega0 * (timeTotal - time)) * timeTotal);
      
      betaCoMPrimeLinear.set(2, 4, Math.exp(omega0 * (timeTotal - time)) * 1);
      betaCoMPrimeLinear.set(2, 5, Math.exp(omega0 * (timeTotal - time)) * timeTotal);
   }
   
   public static void calculateDBetaCoMPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F betaCoMPrimeLinear)
   {
      betaCoMPrimeLinear.set(0, 0, -omega0 * Math.exp(omega0 * (timeTotal - time)) * 1);
      betaCoMPrimeLinear.set(0, 1, -omega0 * Math.exp(omega0 * (timeTotal - time)) * timeTotal);
      
      betaCoMPrimeLinear.set(1, 2, -omega0 * Math.exp(omega0 * (timeTotal - time)) * 1);
      betaCoMPrimeLinear.set(1, 3, -omega0 * Math.exp(omega0 * (timeTotal - time)) * timeTotal);
      
      betaCoMPrimeLinear.set(2, 4, -omega0 * Math.exp(omega0 * (timeTotal - time)) * 1);
      betaCoMPrimeLinear.set(2, 5, -omega0 * Math.exp(omega0 * (timeTotal - time)) * timeTotal);
   }
   
   public static void calculateGammaCoMPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F gammaCoMPrimeLinear)
   {
      gammaCoMPrimeLinear.set(0, 0, Math.exp(omega0 * (timeTotal - time)));
   }
   
   public static void calculateDGammaCoMPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F gammaCoMPrimeLinear)
   {
      gammaCoMPrimeLinear.set(0, 0, -omega0 * Math.exp(omega0 * (timeTotal - time)));
   }
   
   public static void calculateDeltaCoMPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F deltaCoMPrimeLinear)
   {
      deltaCoMPrimeLinear.set(0, 0, 0.5 * (Math.exp(omega0 * (timeTotal - time)) - Math.exp(omega0 * (time - timeTotal))));
   }
   
   public static void calculateDDeltaCoMPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F deltaCoMPrimeLinear)
   {
      deltaCoMPrimeLinear.set(0, 0, 0.5 * (-omega0 * Math.exp(omega0 * (timeTotal - time)) - omega0 * Math.exp(omega0 * (time - timeTotal))));
   }
   
//   public static void calculateAlphaCoMPrime3DByHandCubic(double omega0, double time, double timeTotal, DenseMatrix64F alphaPrimeLinear)
//   {
//      alphaPrimeLinear.set(0, 0, 1);
//      alphaPrimeLinear.set(0, 1, time + 1.0/omega0);
//      alphaPrimeLinear.set(0, 2, Math.pow(time, 2) + 2.0 * time/omega0 + 2.0/Math.pow(omega0, 2));
//      alphaPrimeLinear.set(0, 3, Math.pow(time, 3) + 3.0 * Math.pow(time, 2)/omega0 + 6.0 * time/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3));
//      
//      alphaPrimeLinear.set(1, 4, 1);
//      alphaPrimeLinear.set(1, 5, time + 1.0/omega0);
//      alphaPrimeLinear.set(1, 6, Math.pow(time, 2) + 2.0 * time/omega0 + 2.0/Math.pow(omega0, 2));
//      alphaPrimeLinear.set(1, 7, Math.pow(time, 3) + 3.0 * Math.pow(time, 2)/omega0 + 6.0 * time/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3));
//      
//      alphaPrimeLinear.set(2, 8, 1);
//      alphaPrimeLinear.set(2, 9, time + 1.0/omega0);
//      alphaPrimeLinear.set(2, 10, Math.pow(time, 2) + 2.0 * time/omega0 + 2.0/Math.pow(omega0, 2));
//      alphaPrimeLinear.set(2, 11, Math.pow(time, 3) + 3.0 * Math.pow(time, 2)/omega0 + 6.0 * time/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3));
//   }
//   
//   public static void calculateBetaCoMPrime3DByHandCubic(double omega0, double time, double timeTotal, DenseMatrix64F betaPrimeLinear)
//   {
//      betaPrimeLinear.set(0, 0, Math.exp(omega0 * (time - timeTotal))*1);
//      betaPrimeLinear.set(0, 1, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
//      betaPrimeLinear.set(0, 2, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 2) + 2.0 * timeTotal/omega0 + 2.0/Math.pow(omega0, 2)));
//      betaPrimeLinear.set(0, 3, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 3) + 3.0 * Math.pow(timeTotal, 2)/omega0 + 6.0 * timeTotal/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3)));
//      
//      betaPrimeLinear.set(1, 4, Math.exp(omega0 * (time - timeTotal))*1);
//      betaPrimeLinear.set(1, 5, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
//      betaPrimeLinear.set(1, 6, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 2) + 2.0 * timeTotal/omega0 + 2.0/Math.pow(omega0, 2)));
//      betaPrimeLinear.set(1, 7, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 3) + 3.0 * Math.pow(timeTotal, 2)/omega0 + 6.0 * timeTotal/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3)));
//      
//      betaPrimeLinear.set(2, 8, Math.exp(omega0 * (time - timeTotal))*1);
//      betaPrimeLinear.set(2, 9, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
//      betaPrimeLinear.set(2, 10, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 2) + 2.0 * timeTotal/omega0 + 2.0/Math.pow(omega0, 2)));
//      betaPrimeLinear.set(2, 11, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 3) + 3.0 * Math.pow(timeTotal, 2)/omega0 + 6.0 * timeTotal/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3)));
//   }
//   
//   public static void calculateGammaCoMPrime3DByHandCubic(double omega0, double time, double timeTotal, DenseMatrix64F gammaPrimeLinear)
//   {
//      gammaPrimeLinear.set(0, 0, Math.exp(omega0 * (time - timeTotal)));
//   }
}
