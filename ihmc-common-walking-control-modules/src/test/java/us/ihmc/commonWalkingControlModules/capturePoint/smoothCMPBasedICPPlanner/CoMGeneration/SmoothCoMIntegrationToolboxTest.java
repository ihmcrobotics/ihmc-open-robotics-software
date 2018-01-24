package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoMGeneration;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.SmoothCapturePointToolbox;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SmoothCoMIntegrationToolboxTest
{
   private static final int nTests = 20;
   private static final double omega0 = 3.4;
   private static final double EPSILON = 10e-6;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private Random random = new Random();
   
   String namePrefix = "SmoothCoMIntegrationToolboxTest";
   
   private final SmoothCapturePointToolbox icpToolbox = new SmoothCapturePointToolbox();
   private final SmoothCoMIntegrationToolbox comToolbox = new SmoothCoMIntegrationToolbox(icpToolbox);
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMatricesCoMPrime3DLinear()
   {
      // Linear polynomial: y(x) = a0 + a1*x
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 2;
      FrameTrajectory3D linear3D = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      
      for(int i = 0; i < nTests; i++)
      {
         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();
                    
         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FramePoint3D cmpFinal = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         
         linear3D.setLinear(t0, tFinal, cmp0, cmpFinal);
         
         double time = t0 + Math.random() * (tFinal - t0);
         
         DenseMatrix64F alphaCoMPrimeAutomatic = new DenseMatrix64F(3, 3 * numberOfCoefficients);
         DenseMatrix64F alphaCoMPrimeManual = new DenseMatrix64F(3, 3 * numberOfCoefficients);
      
         comToolbox.calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(omega0, time, alphaCoMPrimeAutomatic, 0, linear3D);
         calculateAlphaCoMPrime3DByHandLinear(omega0 , time, t0, tFinal, alphaCoMPrimeManual);
         
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
      
         comToolbox.calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(omega0, time, betaCoMPrimeAutomatic, 0, linear3D);
         calculateBetaCoMPrime3DByHandLinear(omega0 , time, t0, tFinal, betaCoMPrimeManual);
         
//         PrintTools.debug("B linear calc: " + betaCoMPrimeAutomatic.toString());
//         PrintTools.debug("B linear test: " + betaCoMPrimeManual.toString());
         
         for(int j = 0; j < betaCoMPrimeAutomatic.getNumCols(); j++)
         {
            for(int k = 0; k < betaCoMPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(betaCoMPrimeAutomatic.get(k, j), betaCoMPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F gammaCoMPrimeManual = new DenseMatrix64F(1, 1);
      
         double gammaCoMPrimeAutomatic = comToolbox.calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(omega0, time, 0, linear3D);
         calculateGammaCoMPrime3DByHandLinear(omega0 , time, t0, tFinal, gammaCoMPrimeManual);
         
//         PrintTools.debug("C linear calc: " + gammaCoMPrimeAutomatic.toString());
//         PrintTools.debug("C linear test: " + gammaCoMPrimeManual.toString());
         
         assertEquals(gammaCoMPrimeAutomatic, gammaCoMPrimeManual.get(0), EPSILON);
         
         double deltaCoMPrimeAutomatic = comToolbox.calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(omega0, time, 0, linear3D);
         double deltaCoMPrimeManual = calculateDeltaCoMPrime3DByHandLinear(omega0 , time, t0, tFinal);

         assertEquals(deltaCoMPrimeAutomatic, deltaCoMPrimeManual, EPSILON);
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDMatricesCoMPrime3DLinear()
   {
      // Linear polynomial: y(x) = a0 + a1*x
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 2;
      FrameTrajectory3D linear3D = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      
      for(int i = 0; i < nTests; i++)
      {
         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();
                    
         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FramePoint3D cmpFinal = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         
         linear3D.setLinear(t0, tFinal, cmp0, cmpFinal);
         
         double time = t0 + Math.random() * (tFinal - t0);
         
         DenseMatrix64F dAlphaCoMPrimeAutomatic = new DenseMatrix64F(3, 3 * numberOfCoefficients);
         DenseMatrix64F dAlphaCoMPrimeManual = new DenseMatrix64F(3, 3 * numberOfCoefficients);
      
         comToolbox.calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(omega0, time, dAlphaCoMPrimeAutomatic, 1, linear3D);
         calculateDAlphaCoMPrime3DByHandLinear(omega0 , time, t0, tFinal, dAlphaCoMPrimeManual);
         
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
      
         comToolbox.calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(omega0, time, dBetaCoMPrimeAutomatic, 1, linear3D);
         calculateDBetaCoMPrime3DByHandLinear(omega0 , time, t0, tFinal, dBetaCoMPrimeManual);
         
//         PrintTools.debug("dB linear calc: " + dBetaCoMPrimeAutomatic.toString());
//         PrintTools.debug("dB linear test: " + dBetaCoMPrimeManual.toString());
         
         for(int j = 0; j < dBetaCoMPrimeAutomatic.getNumCols(); j++)
         {
            for(int k = 0; k < dBetaCoMPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(dBetaCoMPrimeAutomatic.get(k, j), dBetaCoMPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F dGammaCoMPrimeManual = new DenseMatrix64F(1, 1);
      
         double dGammaCoMPrimeAutomatic = comToolbox.calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(omega0, time, 1, linear3D);
         calculateDGammaCoMPrime3DByHandLinear(omega0 , time, t0, tFinal, dGammaCoMPrimeManual);
         
//         PrintTools.debug("dC linear calc: " + dGammaCoMPrimeAutomatic.toString());
//         PrintTools.debug("dC linear test: " + dGammaCoMPrimeManual.toString());
         
         assertEquals(dGammaCoMPrimeAutomatic, dGammaCoMPrimeManual.get(0), EPSILON);

         double dDeltaCoMPrimeAutomatic = comToolbox.calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(omega0, time, 1, linear3D);
         double dDeltaCoMPrimeManual = calculateDDeltaCoMPrime3DByHandLinear(omega0 , time, t0, tFinal);
         
//         PrintTools.debug("dD linear calc: " + dDeltaCoMPrimeAutomatic.toString());
//         PrintTools.debug("dD linear test: " + dDeltaCoMPrimeManual.toString());
         
         assertEquals(dDeltaCoMPrimeAutomatic, dDeltaCoMPrimeManual, EPSILON);
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCalculateCoMPositionAndVelocityOnSegment3DLinear()
   {
   // Linear polynomial: y(x) = a0 + a1*x
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 2;
      FrameTrajectory3D linear3D = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      
      for(int i = 0; i < nTests; i++)
      {
//         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + Math.random(); //scaleTFinal * Math.random();
                    
         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 1.0));
         FramePoint3D cmpFinal = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 1.0));
         
         linear3D.setLinear(t0, tFinal, cmp0, cmpFinal);
         
         double time = t0 + Math.random() * (tFinal - t0);                
         
         FramePoint3D icpPositionDesiredFinal = new FramePoint3D(worldFrame, cmpFinal);
         FramePoint3D comPositionDesiredInitial = new FramePoint3D(worldFrame, cmp0);
         
         FramePoint3D icpPositionDesiredInitial = new FramePoint3D(worldFrame);
         icpToolbox.computeDesiredCapturePointPosition3D(omega0, t0, icpPositionDesiredFinal, linear3D, icpPositionDesiredInitial);

         // Position
         FramePoint3D comPositionDesiredCurrent = new FramePoint3D(worldFrame);
         FramePoint3D comPositionDesiredFinal = new FramePoint3D(worldFrame);
         FramePoint3D comPositionDesiredCurrentByHand = new FramePoint3D(worldFrame);
         FramePoint3D comPositionDesiredFinalByHand = new FramePoint3D(worldFrame);
         
         comToolbox.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, linear3D, icpPositionDesiredFinal, comPositionDesiredInitial, comPositionDesiredCurrent);
         comToolbox.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, tFinal, 0, linear3D, icpPositionDesiredFinal, comPositionDesiredInitial, comPositionDesiredFinal);
         calculateCoMPositionByHand3DLinear(omega0, time, linear3D, icpPositionDesiredFinal, comPositionDesiredInitial, comPositionDesiredCurrentByHand);
         calculateCoMPositionByHand3DLinear(omega0, tFinal, linear3D, icpPositionDesiredFinal, comPositionDesiredInitial, comPositionDesiredFinalByHand);
         
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

         EuclidCoreTestTools.assertTuple3DEquals("", comPositionDesiredCurrent, comPositionDesiredCurrentByHand, EPSILON);
         
         // Velocity
         FrameVector3D comVelocityDesiredCurrent = new FrameVector3D(worldFrame);
         FrameVector3D comVelocityDesiredCurrentByHand = new FrameVector3D(worldFrame);
         
         comToolbox.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 1, linear3D, icpPositionDesiredFinal, comPositionDesiredInitial, comVelocityDesiredCurrent);
         calculateCoMVelocityByHand3DLinear(omega0, time, linear3D, icpPositionDesiredFinal, comPositionDesiredInitial, comVelocityDesiredCurrentByHand);
         
//         PrintTools.debug("CoM vel calc: " + comVelocityDesiredCurrent.toString());
//         PrintTools.debug("CoM vel hand: " + comVelocityDesiredCurrentByHand.toString());
//         PrintTools.debug("");
         
         EuclidCoreTestTools.assertTuple3DEquals("", comVelocityDesiredCurrent, comVelocityDesiredCurrentByHand, EPSILON);
         
         // Dynamics
         linear3D.compute(time);
         FramePoint3D icpPositionDesiredCurrent = new FramePoint3D(worldFrame);
         icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, linear3D, icpPositionDesiredFinal, icpPositionDesiredCurrent);
         
         FrameVector3D comVelocityDesiredCurrentDynamics = new FrameVector3D(worldFrame);
         comVelocityDesiredCurrentDynamics.sub(icpPositionDesiredCurrent, comPositionDesiredCurrent);
         comVelocityDesiredCurrentDynamics.scale(omega0);
         
         EuclidCoreTestTools.assertTuple3DEquals("", comVelocityDesiredCurrent, comVelocityDesiredCurrentDynamics, EPSILON);
         
         FrameVector3D comVelocityDesiredCurrentDynamicsByHand = new FrameVector3D(worldFrame);
         comVelocityDesiredCurrentDynamicsByHand.sub(icpPositionDesiredCurrent, comPositionDesiredCurrentByHand);
         comVelocityDesiredCurrentDynamicsByHand.scale(omega0);
         
         EuclidCoreTestTools.assertTuple3DEquals("", comVelocityDesiredCurrentByHand, comVelocityDesiredCurrentDynamicsByHand, EPSILON);
      }
   }
   
   public static void calculateCoMPositionByHand3DLinear(double omega0, double time, FrameTrajectory3D linear3D, FramePoint3D icpPositionDesiredFinal, FramePoint3D comPositionDesiredInitial, FramePoint3D comPositionDesiredCurrent)
   {      
      linear3D.compute(linear3D.getInitialTime());
      FramePoint3D cmpRefInit = new FramePoint3D(linear3D.getFramePosition());
      
      linear3D.compute(linear3D.getFinalTime());
      FramePoint3D cmpRefFinal = new FramePoint3D(linear3D.getFramePosition());
      
      double timeInitial = linear3D.getInitialTime();
      double timeFinal = linear3D.getFinalTime();
      
      double at = 1 - Math.exp(omega0 * (timeInitial - time));
      double bt = time - timeInitial * Math.exp(omega0 * (timeInitial- time)) - 1.0/omega0 + 1.0/omega0 * Math.exp(omega0 * (timeInitial - time));
      double ct = 0.5 * Math.exp(omega0 * (timeInitial - timeFinal)) * (Math.exp(omega0 * (time - timeInitial)) - Math.exp(omega0 * (timeInitial - time)));
      
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
         comPositionDesiredCurrent.setElement(i, Math.exp(omega0 * (timeInitial - time)) * comPositionDesiredInitial.getElement(i) + (a0*at + b0*bt + c0*ct));
      }
   }
   
   public static void calculateCoMVelocityByHand3DLinear(double omega0, double time, FrameTrajectory3D linear3D, FramePoint3D icpPositionDesiredFinal, FramePoint3D comPositionDesiredFinal, FrameVector3D comVelocityDesiredCurrent)
   {      
      linear3D.compute(linear3D.getInitialTime());
      FramePoint3D cmpRefInit = new FramePoint3D(linear3D.getFramePosition());
      
      linear3D.compute(linear3D.getFinalTime());
      FramePoint3D cmpRefFinal = new FramePoint3D(linear3D.getFramePosition());
      
      double timeInitial = linear3D.getInitialTime();
      double timeFinal = linear3D.getFinalTime();
      
      double dat = omega0 * Math.exp(omega0 * (timeInitial - time));
      double dbt = 1 + omega0 * timeInitial * Math.exp(omega0 * (timeInitial- time)) - Math.exp(omega0 * (timeInitial - time));
      double dct = 0.5 * Math.exp(omega0 * (timeInitial - timeFinal)) * (omega0 * Math.exp(omega0 * (time - timeInitial)) + omega0 * Math.exp(omega0 * (timeInitial - time)));
      
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
         comVelocityDesiredCurrent.setElement(i, -omega0 * Math.exp(omega0 * (timeInitial - time)) * comPositionDesiredFinal.getElement(i) + (a0*dat + b0*dbt + c0*dct));
      }
   }
      
   public static void calculateAlphaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal, DenseMatrix64F alphaCoMPrimeLinear)
   {
      alphaCoMPrimeLinear.set(0, 0, 1);
      alphaCoMPrimeLinear.set(0, 1, time);
      
      alphaCoMPrimeLinear.set(1, 2, 1);
      alphaCoMPrimeLinear.set(1, 3, time);
      
      alphaCoMPrimeLinear.set(2, 4, 1);
      alphaCoMPrimeLinear.set(2, 5, time);
   }
   
   public static void calculateDAlphaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal, DenseMatrix64F alphaCoMPrimeLinear)
   {
      alphaCoMPrimeLinear.set(0, 0, 0);
      alphaCoMPrimeLinear.set(0, 1, 1);
      
      alphaCoMPrimeLinear.set(1, 2, 0);
      alphaCoMPrimeLinear.set(1, 3, 1);
      
      alphaCoMPrimeLinear.set(2, 4, 0);
      alphaCoMPrimeLinear.set(2, 5, 1);
   }
   
   public static void calculateBetaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal, DenseMatrix64F betaCoMPrimeLinear)
   {
      betaCoMPrimeLinear.set(0, 0, Math.exp(omega0 * (timeInitial - time)) * 1);
      betaCoMPrimeLinear.set(0, 1, Math.exp(omega0 * (timeInitial - time)) * timeInitial);
      
      betaCoMPrimeLinear.set(1, 2, Math.exp(omega0 * (timeInitial - time)) * 1);
      betaCoMPrimeLinear.set(1, 3, Math.exp(omega0 * (timeInitial - time)) * timeInitial);
      
      betaCoMPrimeLinear.set(2, 4, Math.exp(omega0 * (timeInitial - time)) * 1);
      betaCoMPrimeLinear.set(2, 5, Math.exp(omega0 * (timeInitial - time)) * timeInitial);
   }
   
   public static void calculateDBetaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal, DenseMatrix64F betaCoMPrimeLinear)
   {
      betaCoMPrimeLinear.set(0, 0, -omega0 * Math.exp(omega0 * (timeInitial - time)) * 1);
      betaCoMPrimeLinear.set(0, 1, -omega0 * Math.exp(omega0 * (timeInitial - time)) * timeInitial);
      
      betaCoMPrimeLinear.set(1, 2, -omega0 * Math.exp(omega0 * (timeInitial - time)) * 1);
      betaCoMPrimeLinear.set(1, 3, -omega0 * Math.exp(omega0 * (timeInitial - time)) * timeInitial);
      
      betaCoMPrimeLinear.set(2, 4, -omega0 * Math.exp(omega0 * (timeInitial - time)) * 1);
      betaCoMPrimeLinear.set(2, 5, -omega0 * Math.exp(omega0 * (timeInitial - time)) * timeInitial);
   }
   
   public static void calculateGammaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal, DenseMatrix64F gammaCoMPrimeLinear)
   {
      gammaCoMPrimeLinear.set(0, 0, Math.exp(omega0 * (timeInitial - time)));
   }
   
   public static void calculateDGammaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal, DenseMatrix64F gammaCoMPrimeLinear)
   {
      gammaCoMPrimeLinear.set(0, 0, -omega0 * Math.exp(omega0 * (timeInitial - time)));
   }
   
   public static double calculateDeltaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal)
   {
      return 0.5 * Math.exp(omega0 * (timeInitial - timeTotal)) * (Math.exp(omega0 * (time - timeInitial)) - Math.exp(omega0 * (timeInitial - time)));
   }
   
   public static double calculateDDeltaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal)
   {
      return 0.5 * Math.exp(omega0 * (timeInitial - timeTotal)) * (omega0 * Math.exp(omega0 * (time - timeInitial)) + omega0 * Math.exp(omega0 * (timeInitial - time)));
   }
}
