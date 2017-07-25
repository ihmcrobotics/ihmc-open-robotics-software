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
         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();
                    
         FramePoint cmp0 = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FramePoint cmpFinal = new FramePoint(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         
         linear3D.setLinear(t0, tFinal, cmp0, cmpFinal);
         
         double time = t0 + Math.random() * (tFinal - t0);                
         
         FramePoint icpPositionDesiredFinal = new FramePoint(worldFrame, cmpFinal.getPoint());
         FramePoint comPositionDesiredFinal = new FramePoint(worldFrame, cmpFinal.getPoint());
         
         FramePoint icpPositionDesiredInitial = new FramePoint(worldFrame);
         CapturePointTools.computeDesiredCapturePointPosition(omega0, time, icpPositionDesiredFinal, linear3D, icpPositionDesiredInitial);

         // Position
         FramePoint comPositionDesiredCurrent = new FramePoint(worldFrame);
         FramePoint comPositionDesiredCurrentByHand = new FramePoint(worldFrame);
         
         SmoothCoMIntegrationTools.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, linear3D, icpPositionDesiredFinal, comPositionDesiredFinal, comPositionDesiredCurrent);
         calculateCoMPositionByHand3DLinear(omega0, time, linear3D, icpPositionDesiredFinal, comPositionDesiredFinal, comPositionDesiredCurrentByHand);
         
         PrintTools.debug("Time = " + time + " [" + t0 + ", " + tFinal + "]");
         PrintTools.debug("Ini ICP: " + icpPositionDesiredInitial.toString());
         PrintTools.debug("End ICP: " + icpPositionDesiredFinal.toString());
         PrintTools.debug("End CoM: " + comPositionDesiredFinal.toString());
         PrintTools.debug("CoM pos calc: " + comPositionDesiredCurrent.toString());
         PrintTools.debug("CoM pos hand: " + comPositionDesiredCurrentByHand.toString());

         EuclidCoreTestTools.assertTuple3DEquals("", comPositionDesiredCurrent.getPoint(), comPositionDesiredCurrentByHand.getPoint(), EPSILON);
         
//         //Velocity
//         FrameVector comVelocityDesiredCurrent = new FrameVector(worldFrame);
//         FrameVector comVelocityDesiredCurrentByHand = new FrameVector(worldFrame);
//         
//         SmoothCoMIntegrationTools.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 1, linear3D, icpPositionDesiredFinal, comPositionDesiredFinal, comVelocityDesiredCurrent);
//         calculateCoMVelocityByHand3DLinear(omega0, time, linear3D, comPositionDesiredFinal, comVelocityDesiredCurrentByHand);
//         
////         PrintTools.debug("CoM vel calc: " + comVelocityDesiredCurrent.toString());
////         PrintTools.debug("CoM vel hand: " + comVelocityDesiredCurrentByHand.toString());
//         
//         EuclidCoreTestTools.assertTuple3DEquals("", comVelocityDesiredCurrent.getVectorCopy(), comVelocityDesiredCurrentByHand.getVectorCopy(), EPSILON);
         
         // Dynamics
//         linear3D.compute(time);
//         FramePoint cmpPositionDesiredCurrent = new FramePoint(worldFrame, linear3D.getPosition());
//         
//         FrameVector comVelocityDesiredCurrentDynamics = new FrameVector(worldFrame);
//         comVelocityDesiredCurrentDynamics.subAndScale(omega0, comPositionDesiredCurrent, cmpPositionDesiredCurrent);
//         
//         EuclidCoreTestTools.assertTuple3DEquals("", comVelocityDesiredCurrent.getVectorCopy(), comVelocityDesiredCurrentDynamics.getVectorCopy(), EPSILON);
      }
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
      
      linear3D.compute(time);
      FramePoint cmpRefCurrent = new FramePoint(linear3D.getFramePosition());
      
      FramePoint deltaPoint = cmpRefFinal;
      deltaPoint.add(linear3D.getYoTrajectoryX().getCoefficient(1) / omega0, linear3D.getYoTrajectoryY().getCoefficient(1) / omega0, linear3D.getYoTrajectoryZ().getCoefficient(1) / omega0);
      deltaPoint.sub(icpPositionDesiredFinal);
      
      double timeFinal = linear3D.getFinalTime();
      
//      double alpha = 1.0;
//      double beta = -Math.exp(omega0 * (timeFinal - time));
//      double gamma =  Math.exp(omega0 * (timeFinal - time));
//      double delta = 0.5 * (Math.exp(omega0 * (timeFinal - time)) - Math.exp(omega0 * (time - timeFinal)));
//      
//      PrintTools.debug("alpha = " + alpha);
//      PrintTools.debug("beta = " + beta);
//      PrintTools.debug("gamma = " + gamma);
//      PrintTools.debug("delta = " + delta);
//      PrintTools.debug("CMP curr = " + cmpRefCurrent.toString());
//      PrintTools.debug("CMP finl = " + cmpRefFinal.toString());
//      PrintTools.debug("CoM finl = " + comPositionDesiredFinal.toString());
//      PrintTools.debug("Delta pt = " + deltaPoint.toString());
//      
//      comPositionDesiredCurrent.setToZero();
//      comPositionDesiredCurrent.scaleAdd(1.0, comPositionDesiredCurrent.getPointCopy(), alpha, cmpRefCurrent.getPointCopy());
//      comPositionDesiredCurrent.scaleAdd(1.0, comPositionDesiredCurrent.getPointCopy(), beta, cmpRefFinal.getPointCopy());
//      comPositionDesiredCurrent.scaleAdd(1.0, comPositionDesiredCurrent.getPointCopy(), gamma, comPositionDesiredFinal.getPointCopy());
//      comPositionDesiredCurrent.scaleAdd(1.0, comPositionDesiredCurrent.getPointCopy(), delta, deltaPoint.getPointCopy());
      
      double alpha = 1.0 - time/timeFinal - 1/(omega0*timeFinal) * Math.sinh(omega0)*(timeFinal - time);
      double beta = time/timeFinal - Math.cosh(omega0)*(timeFinal - time) + 1/(omega0*timeFinal) * Math.sinh(omega0)*(timeFinal - time);
      double gamma =  Math.exp(omega0 * (timeFinal - time));
      double delta = -Math.sinh(omega0)*(timeFinal - time);
      
      PrintTools.debug("alpha = " + alpha);
      PrintTools.debug("beta = " + beta);
      PrintTools.debug("gamma = " + gamma);
      PrintTools.debug("delta = " + delta);
      PrintTools.debug("CMP curr = " + cmpRefCurrent.toString());
      PrintTools.debug("CMP finl = " + cmpRefFinal.toString());
      PrintTools.debug("CoM finl = " + comPositionDesiredFinal.toString());
      PrintTools.debug("Delta pt = " + deltaPoint.toString());
      
      comPositionDesiredCurrent.setToZero();
      comPositionDesiredCurrent.scaleAdd(1.0, comPositionDesiredCurrent.getPointCopy(), alpha, cmpRefInit.getPointCopy());
      comPositionDesiredCurrent.scaleAdd(1.0, comPositionDesiredCurrent.getPointCopy(), beta, cmpRefFinal.getPointCopy());
      comPositionDesiredCurrent.scaleAdd(1.0, comPositionDesiredCurrent.getPointCopy(), gamma, comPositionDesiredFinal.getPointCopy());
      comPositionDesiredCurrent.scaleAdd(1.0, comPositionDesiredCurrent.getPointCopy(), delta, icpPositionDesiredFinal.getPointCopy());
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
   
//   public static void calculateCoMVelocityByHand3DLinear(double omega0, double time, YoFrameTrajectory3D linear3D, FramePoint icpPositionDesiredFinal, FrameVector icpVelocityDesiredCurrent)
//   {      
//      linear3D.compute(linear3D.getInitialTime());
//      FramePoint cmpRefInit = new FramePoint(linear3D.getFramePosition());
//      
//      linear3D.compute(linear3D.getFinalTime());
//      FramePoint cmpRefFinal = new FramePoint(linear3D.getFramePosition());
//      
//      double timeFinal = linear3D.getFinalTime();
//      
//      double dSigmat = calculateSigmaDotLinear(time, timeFinal, omega0);
//      double sigmaT = calculateSigmaLinear(timeFinal, timeFinal, omega0);
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
   
   public static void calculateBetaCoMPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F betaCoMPrimeLinear)
   {
      betaCoMPrimeLinear.set(0, 0, Math.exp(omega0 * (timeTotal - time)) * 1);
      betaCoMPrimeLinear.set(0, 1, Math.exp(omega0 * (timeTotal - time)) * timeTotal);
      
      betaCoMPrimeLinear.set(1, 2, Math.exp(omega0 * (timeTotal - time)) * 1);
      betaCoMPrimeLinear.set(1, 3, Math.exp(omega0 * (timeTotal - time)) * timeTotal);
      
      betaCoMPrimeLinear.set(2, 4, Math.exp(omega0 * (timeTotal - time)) * 1);
      betaCoMPrimeLinear.set(2, 5, Math.exp(omega0 * (timeTotal - time)) * timeTotal);
   }
   
   public static void calculateGammaCoMPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F gammaCoMPrimeLinear)
   {
      gammaCoMPrimeLinear.set(0, 0, Math.exp(omega0 * (timeTotal - time)));
   }
   
   public static void calculateDeltaCoMPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F deltaCoMPrimeLinear)
   {
      deltaCoMPrimeLinear.set(0, 0, 1);
      deltaCoMPrimeLinear.set(0, 1, timeTotal + 1.0/omega0);
      
      deltaCoMPrimeLinear.set(1, 2, 1);
      deltaCoMPrimeLinear.set(1, 3, timeTotal + 1.0/omega0);
      
      deltaCoMPrimeLinear.set(2, 4, 1);
      deltaCoMPrimeLinear.set(2, 5, timeTotal + 1.0/omega0);
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
