package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SmoothCapturePointToolboxTest
{
   private static final int nTests = 20;
   private static final double omega0 = 3.4;
   private static final double EPSILON = 10e-6;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private Random random = new Random();
   
   YoVariableRegistry registry = new YoVariableRegistry("");
   String namePrefix = "SmoothCapturePointToolboxTest";
   
   private final SmoothCapturePointToolbox icpToolbox = new SmoothCapturePointToolbox();
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeDesiredCornerPointsFromCMPPolynomials()
   {
      double omega0 = 3.4;
      int numberOfCoPWaypoints = 8;
      int numberOfCoefficients = 2;
      String namePrefix = "cmpPolynomial";
      
      double t0 = 0.0;
      double tFinal = 0.8;    

      ArrayList<FramePoint3D> copsToPack = new ArrayList<FramePoint3D>();
      List<FrameTrajectory3D> cmpPolynomials3D = new ArrayList<FrameTrajectory3D>();
      
      ArrayList<FramePoint3D> entryCornerPointsToPack = new ArrayList<FramePoint3D>();
      ArrayList<FramePoint3D> exitCornerPointsToPack = new ArrayList<FramePoint3D>();
      
      ArrayList<FramePoint3D> entryCornerPointsToPackDecoupled = new ArrayList<FramePoint3D>();
      ArrayList<FramePoint3D> exitCornerPointsToPackDecoupled = new ArrayList<FramePoint3D>();
      
      ArrayList<FramePoint3D> entryCornerPointsByHandToPack = new ArrayList<FramePoint3D>();
      ArrayList<FramePoint3D> exitCornerPointsByHandToPack = new ArrayList<FramePoint3D>();

      for (int i = 0; i < numberOfCoPWaypoints; i++)
      {
         copsToPack.add(new FramePoint3D(ReferenceFrame.getWorldFrame()));
      }
      for (int i = 0; i < numberOfCoPWaypoints-1; i++)
      {
         entryCornerPointsToPack.add(new FramePoint3D(ReferenceFrame.getWorldFrame()));
         exitCornerPointsToPack.add(new FramePoint3D(ReferenceFrame.getWorldFrame()));
         
         entryCornerPointsToPackDecoupled.add(new FramePoint3D(ReferenceFrame.getWorldFrame()));
         exitCornerPointsToPackDecoupled.add(new FramePoint3D(ReferenceFrame.getWorldFrame()));
         
         entryCornerPointsByHandToPack.add(new FramePoint3D(ReferenceFrame.getWorldFrame()));
         exitCornerPointsByHandToPack.add(new FramePoint3D(ReferenceFrame.getWorldFrame()));
         
         cmpPolynomials3D.add(new FrameTrajectory3D(numberOfCoefficients, ReferenceFrame.getWorldFrame()));
      }

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < numberOfCoPWaypoints; i++)
         {
            FramePoint3D cop = new FramePoint3D(ReferenceFrame.getWorldFrame(), new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
            copsToPack.set(i, cop);
         }
         for (int i = 0; i < numberOfCoPWaypoints-1; i++)
         {
            FrameTrajectory3D cmpPolynomial3D = new FrameTrajectory3D(numberOfCoefficients, ReferenceFrame.getWorldFrame());
            cmpPolynomial3D.setLinear(t0, tFinal, copsToPack.get(i), copsToPack.get(i+1));
            cmpPolynomial3D.compute(t0);
            EuclidCoreTestTools.assertTuple3DEquals(copsToPack.get(i), cmpPolynomial3D.getPosition(), EPSILON);
            cmpPolynomial3D.compute(tFinal);
            EuclidCoreTestTools.assertTuple3DEquals(copsToPack.get(i+1), cmpPolynomial3D.getPosition(), EPSILON);
            assertEquals(t0, cmpPolynomial3D.getInitialTime(), EPSILON);
            assertEquals(tFinal, cmpPolynomial3D.getFinalTime(), EPSILON);
            cmpPolynomials3D.set(i, cmpPolynomial3D);
            cmpPolynomials3D.get(i).compute(t0);
            EuclidCoreTestTools.assertTuple3DEquals(copsToPack.get(i), cmpPolynomials3D.get(i).getPosition(), EPSILON);
            cmpPolynomials3D.get(i).compute(tFinal);
            EuclidCoreTestTools.assertTuple3DEquals(copsToPack.get(i+1), cmpPolynomials3D.get(i).getPosition(), EPSILON);
            assertEquals(t0, cmpPolynomials3D.get(i).getInitialTime(), EPSILON);
            assertEquals(tFinal, cmpPolynomials3D.get(i).getFinalTime(), EPSILON);
         }
         icpToolbox.computeDesiredCornerPoints3D(entryCornerPointsToPack, exitCornerPointsToPack, cmpPolynomials3D, omega0);
         icpToolbox.computeDesiredCornerPoints(entryCornerPointsToPackDecoupled, exitCornerPointsToPackDecoupled, cmpPolynomials3D, omega0);
         
         exitCornerPointsByHandToPack.set(exitCornerPointsByHandToPack.size()-1, copsToPack.get(copsToPack.size()-1));
         for(int i = numberOfCoPWaypoints-2; i >= 0; i--)
         {
            double time = 0.0;
            FramePoint3D newEntryICP = new FramePoint3D(ReferenceFrame.getWorldFrame());
            SmoothCapturePointToolboxTest.calculateICPPositionByHand3DLinear(omega0, time, cmpPolynomials3D.get(i), exitCornerPointsByHandToPack.get(i), newEntryICP);
            entryCornerPointsByHandToPack.set(i, newEntryICP);
            if(i > 0)
            {
               exitCornerPointsByHandToPack.set(i-1, newEntryICP);
            }
         }
//         PrintTools.debug("Test " + j + ":");
//         PrintTools.debug("Entry Calc: " + entryCornerPointsToPack.toString());
//         PrintTools.debug("Entry Hand: " + entryCornerPointsByHandToPack.toString());
//         PrintTools.debug("Exit Calc: " + exitCornerPointsToPack.toString());
//         PrintTools.debug("Exit Hand: " + exitCornerPointsByHandToPack.toString());
//         PrintTools.debug("");
         for (int i = 0; i < numberOfCoPWaypoints-1; i++)
         {
            cmpPolynomials3D.get(i).compute(t0);
            assertEquals(t0, cmpPolynomials3D.get(i).getInitialTime(), EPSILON);
            assertEquals(tFinal, cmpPolynomials3D.get(i).getFinalTime(), EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals(copsToPack.get(i), cmpPolynomials3D.get(i).getPosition(), EPSILON);
            cmpPolynomials3D.get(i).compute(tFinal);
            EuclidCoreTestTools.assertTuple3DEquals(copsToPack.get(i+1), cmpPolynomials3D.get(i).getPosition(), EPSILON);
         }
         for(int i = 0; i < numberOfCoefficients-1; i++)
         {
            EuclidCoreTestTools.assertTuple3DEquals("", entryCornerPointsByHandToPack.get(i), entryCornerPointsToPack.get(i), EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("", exitCornerPointsByHandToPack.get(i), exitCornerPointsToPack.get(i), EPSILON);
            
            EuclidCoreTestTools.assertTuple3DEquals("", entryCornerPointsToPackDecoupled.get(i), entryCornerPointsToPack.get(i), EPSILON);
            EuclidCoreTestTools.assertTuple3DEquals("", exitCornerPointsToPackDecoupled.get(i), exitCornerPointsToPack.get(i), EPSILON);
         }
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMatricesPrime3DLinear()
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
         
         DenseMatrix64F alphaPrimeAutomatic = new DenseMatrix64F(3, 3 * numberOfCoefficients);
         DenseMatrix64F alphaPrimeManual = new DenseMatrix64F(3, 3 * numberOfCoefficients);
      
         icpToolbox.calculateGeneralizedAlphaPrimeOnCMPSegment3D(omega0, time, alphaPrimeAutomatic, 0, linear3D);
         calculateAlphaPrime3DByHandLinear(omega0 , time, tFinal, alphaPrimeManual);
         
//         PrintTools.debug("A linear calc: " + alphaPrimeAutomatic.toString());
//         PrintTools.debug("A linear test: " + alphaPrimeManual.toString());

         for(int j = 0; j < alphaPrimeAutomatic.getNumCols(); j++)
         {
            for(int k = 0; k < alphaPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(alphaPrimeAutomatic.get(k, j), alphaPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F betaPrimeAutomatic = new DenseMatrix64F(3, 3 * numberOfCoefficients);
         DenseMatrix64F betaPrimeManual = new DenseMatrix64F(3, 3 * numberOfCoefficients);
      
         icpToolbox.calculateGeneralizedBetaPrimeOnCMPSegment3D(omega0, time, betaPrimeAutomatic, 0, linear3D);
         calculateBetaPrime3DByHandLinear(omega0 , time, tFinal, betaPrimeManual);
         
//         PrintTools.debug("B linear calc: " + betaPrimeAutomatic.toString());
//         PrintTools.debug("B linear test: " + betaPrimeManual.toString());
         
         for(int j = 0; j < betaPrimeAutomatic.getNumCols(); j++)
         {
            for(int k = 0; k < betaPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(betaPrimeAutomatic.get(k, j), betaPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F gammaPrimeManual = new DenseMatrix64F(1, 1);
      
         double gammaPrimeAutomatic = icpToolbox.calculateGeneralizedGammaPrimeOnCMPSegment3D(omega0, time, 0, linear3D);
         calculateGammaPrime3DByHandLinear(omega0 , time, tFinal, gammaPrimeManual);
         
//         PrintTools.debug("C linear calc: " + gammaPrimeAutomatic.toString());
//         PrintTools.debug("C linear test: " + gammaPrimeManual.toString());
         
         assertEquals(gammaPrimeAutomatic, gammaPrimeManual.get(0), EPSILON);
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMatricesPrime3DCubic()
   {
      // Cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 4;
      FrameTrajectory3D cubic3D = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      
      for(int i = 0; i < nTests; i++)
      {
         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();
                    
         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FramePoint3D cmpFinal = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         
         FrameVector3D cmpD0 = new FrameVector3D(worldFrame, new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FrameVector3D cmpDFinal = new FrameVector3D(worldFrame, new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         
         cubic3D.setCubic(t0, tFinal, cmp0, cmpD0, cmpFinal,cmpDFinal);
         
         double time = t0 + Math.random() * (tFinal - t0);
         
         DenseMatrix64F alphaPrimeAutomatic = new DenseMatrix64F(3, 3 * numberOfCoefficients);
         DenseMatrix64F alphaPrimeManual = new DenseMatrix64F(3, 3 * numberOfCoefficients);
      
         icpToolbox.calculateGeneralizedAlphaPrimeOnCMPSegment3D(omega0, time, alphaPrimeAutomatic, 0, cubic3D);
         calculateAlphaPrime3DByHandCubic(omega0 , time, tFinal, alphaPrimeManual);
         
//         PrintTools.debug("A cubic calc: " + alphaPrimeAutomatic.toString());
//         PrintTools.debug("A cubic test: " + alphaPrimeManual.toString());

         for(int j = 0; j < alphaPrimeAutomatic.getNumCols(); j++)
         {
            for(int k = 0; k < alphaPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(alphaPrimeAutomatic.get(k, j), alphaPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F betaPrimeAutomatic = new DenseMatrix64F(3, 3 * numberOfCoefficients);
         DenseMatrix64F betaPrimeManual = new DenseMatrix64F(3, 3 * numberOfCoefficients);
      
         icpToolbox.calculateGeneralizedBetaPrimeOnCMPSegment3D(omega0, time, betaPrimeAutomatic, 0, cubic3D);
         calculateBetaPrime3DByHandCubic(omega0 , time, tFinal, betaPrimeManual);
         
//         PrintTools.debug("B cubic calc: " + betaPrimeAutomatic.toString());
//         PrintTools.debug("B cubic test: " + betaPrimeManual.toString());
         
         for(int j = 0; j < betaPrimeAutomatic.getNumCols(); j++)
         {
            for(int k = 0; k < betaPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(betaPrimeAutomatic.get(k, j), betaPrimeManual.get(k, j), EPSILON);
            }
         }
         
         DenseMatrix64F gammaPrimeManual = new DenseMatrix64F(1, 1);
      
         double gammaPrimeAutomatic = icpToolbox.calculateGeneralizedGammaPrimeOnCMPSegment3D(omega0, time, 0, cubic3D);
         calculateGammaPrime3DByHandCubic(omega0 , time, tFinal, gammaPrimeManual);
         
//         PrintTools.debug("C cubic calc: " + gammaPrimeAutomatic.toString());
//         PrintTools.debug("C cubic test: " + gammaPrimeManual.toString());
         
         assertEquals(gammaPrimeAutomatic, gammaPrimeManual.get(0), EPSILON);
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCalculateICPPositionAndVelocityOnSegment3DLinear()
   {
   // Linear polynomial: y(x) = a0 + a1*x
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 2;
      FrameTrajectory3D linear3D = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      
      for(int i = 0; i < nTests; i++)
      {
         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();
                    
         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 1.0));
         FramePoint3D cmpFinal = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), 1.0));
         
         linear3D.setLinear(t0, tFinal, cmp0, cmpFinal);
         
         double time = t0 + Math.random() * (tFinal - t0);
                  
         FramePoint3D icpPositionDesiredFinal = new FramePoint3D(worldFrame, cmpFinal.getPoint());
         
         // Position
         FramePoint3D icpPositionDesiredCurrent = new FramePoint3D(worldFrame);
         FramePoint3D icpPositionDesiredCurrentByHand = new FramePoint3D(worldFrame);
         
         icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, linear3D, icpPositionDesiredFinal, icpPositionDesiredCurrent);
         calculateICPPositionByHand3DLinear(omega0, time, linear3D, icpPositionDesiredFinal, icpPositionDesiredCurrentByHand);
         
//         PrintTools.debug("ICP pos calc: " + icpPositionDesiredCurrent.toString());
//         PrintTools.debug("ICP pos hand: " + icpPositionDesiredCurrentByHand.toString());

         EuclidCoreTestTools.assertTuple3DEquals("", icpPositionDesiredCurrent.getPoint(), icpPositionDesiredCurrentByHand.getPoint(), EPSILON);
         
         //Velocity
         FrameVector3D icpVelocityDesiredCurrent = new FrameVector3D(worldFrame);
         FrameVector3D icpVelocityDesiredCurrentByHand = new FrameVector3D(worldFrame);
         
         icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 1, linear3D, icpPositionDesiredFinal, icpVelocityDesiredCurrent);
         calculateICPVelocityByHand3DLinear(omega0, time, linear3D, icpPositionDesiredFinal, icpVelocityDesiredCurrentByHand);
         
//         PrintTools.debug("ICP vel calc: " + icpVelocityDesiredCurrent.toString());
//         PrintTools.debug("ICP vel hand: " + icpVelocityDesiredCurrentByHand.toString());
         
         EuclidCoreTestTools.assertTuple3DEquals("", icpVelocityDesiredCurrent, icpVelocityDesiredCurrentByHand, EPSILON);
         
         // Dynamics
         linear3D.compute(time);
         FramePoint3D cmpPositionDesiredCurrent = new FramePoint3D(worldFrame, linear3D.getPosition());
         
         FrameVector3D icpVelocityDesiredCurrentDynamics = new FrameVector3D(worldFrame);
         icpVelocityDesiredCurrentDynamics.sub(icpPositionDesiredCurrent, cmpPositionDesiredCurrent);
         icpVelocityDesiredCurrentDynamics.scale(omega0);
         
         EuclidCoreTestTools.assertTuple3DEquals("", icpVelocityDesiredCurrent, icpVelocityDesiredCurrentDynamics, EPSILON);
      }
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCalculateICPPositionAndVelocityOnSegment3DCubic()
   {
      // Cubic polynomial: y(x) = a0 + a1*x + a2*x^2 + a3*x^3
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      int numberOfCoefficients = 4;
      FrameTrajectory3D cubic3D = new FrameTrajectory3D(numberOfCoefficients, worldFrame);
      
      for(int i = 0; i < nTests; i++)
      {
         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();
                    
         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FramePoint3D cmpFinal = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         
         // !!! TESTING WITH 0 VELOCITIES !!!
         FrameVector3D cmpD0 = new FrameVector3D(worldFrame);
         FrameVector3D cmpDFinal = new FrameVector3D(worldFrame);
         
         cubic3D.setCubic(t0, tFinal, cmp0, cmpD0, cmpFinal, cmpDFinal);
         
         double time = t0 + Math.random() * (tFinal - t0);
                  
         FramePoint3D icpPositionDesiredFinal = new FramePoint3D(worldFrame, cmpFinal.getPoint());
         
         // Position
         FramePoint3D icpPositionDesiredCurrent = new FramePoint3D(worldFrame);
         FramePoint3D icpPositionDesiredCurrentByHand = new FramePoint3D(worldFrame);
         
         icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, cubic3D, icpPositionDesiredFinal, icpPositionDesiredCurrent);
         calculateICPPositionByHand3DCubic(omega0, time, cubic3D, icpPositionDesiredFinal, icpPositionDesiredCurrentByHand);
         
//         PrintTools.debug("ICP pos calc: " + icpPositionDesiredCurrent.toString());
//         PrintTools.debug("ICP pos hand: " + icpPositionDesiredCurrentByHand.toString());

         EuclidCoreTestTools.assertTuple3DEquals("", icpPositionDesiredCurrent.getPoint(), icpPositionDesiredCurrentByHand.getPoint(), EPSILON);
         
         // Velocity
         FrameVector3D icpVelocityDesiredCurrent = new FrameVector3D(worldFrame);
         FrameVector3D icpVelocityDesiredCurrentByHand = new FrameVector3D(worldFrame);
         
         icpToolbox.calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 1, cubic3D, icpPositionDesiredFinal, icpVelocityDesiredCurrent);
         calculateICPVelocityByHand3DCubic(omega0, time, cubic3D, icpPositionDesiredFinal, icpVelocityDesiredCurrentByHand);
         
//         PrintTools.debug("ICP vel calc: " + icpVelocityDesiredCurrent.toString());
//         PrintTools.debug("ICP vel hand: " + icpVelocityDesiredCurrentByHand.toString());
         
         EuclidCoreTestTools.assertTuple3DEquals("", icpVelocityDesiredCurrent, icpVelocityDesiredCurrentByHand, EPSILON);
         
         // Dynamics
         cubic3D.compute(time);
         FramePoint3D cmpPositionDesiredCurrent = new FramePoint3D(worldFrame, cubic3D.getPosition());
         
         FrameVector3D icpVelocityDesiredCurrentDynamics = new FrameVector3D(worldFrame);
         icpVelocityDesiredCurrentDynamics.sub(icpPositionDesiredCurrent, cmpPositionDesiredCurrent);
         icpVelocityDesiredCurrentDynamics.scale(omega0);
         
         EuclidCoreTestTools.assertTuple3DEquals("", icpVelocityDesiredCurrent, icpVelocityDesiredCurrentDynamics, EPSILON);
      }
   }
   
   public static void calculateICPPositionByHand3DLinear(double omega0, double time, FrameTrajectory3D linear3D, FramePoint3D icpPositionDesiredFinal, FramePoint3D icpPositionDesiredCurrent)
   {      
      linear3D.compute(linear3D.getInitialTime());
      FramePoint3D cmpRefInit = new FramePoint3D(linear3D.getFramePosition());
      
      linear3D.compute(linear3D.getFinalTime());
      FramePoint3D cmpRefFinal = new FramePoint3D(linear3D.getFramePosition());
      
      double timeFinal = linear3D.getFinalTime();
      
      double sigmat = calculateSigmaLinear(time, timeFinal, omega0);
      double sigmaT = calculateSigmaLinear(timeFinal, timeFinal, omega0);
      
      double alpha = (1.0 - sigmat - Math.exp(omega0*(time-timeFinal)) * (1.0 - sigmaT));
      double beta = (sigmat - Math.exp(omega0*(time-timeFinal)) * sigmaT);
      double gamma =  Math.exp(omega0*(time-timeFinal));
      
      icpPositionDesiredCurrent.setToZero();
      icpPositionDesiredCurrent.scaleAdd(alpha, cmpRefInit             , icpPositionDesiredCurrent);
      icpPositionDesiredCurrent.scaleAdd(beta, cmpRefFinal             , icpPositionDesiredCurrent);
      icpPositionDesiredCurrent.scaleAdd(gamma, icpPositionDesiredFinal, icpPositionDesiredCurrent);
   }
   
   public static void calculateICPPositionByHand3DCubic(double omega0, double time, FrameTrajectory3D cubic3D, FramePoint3D icpPositionDesiredFinal, FramePoint3D icpPositionDesiredCurrent)
   {      
      cubic3D.compute(cubic3D.getInitialTime());
      FramePoint3D cmpRefInit = new FramePoint3D(cubic3D.getFramePosition());
      
      cubic3D.compute(cubic3D.getFinalTime());
      FramePoint3D cmpRefFinal = new FramePoint3D(cubic3D.getFramePosition());
      
      double timeFinal = cubic3D.getFinalTime();
      
      double sigmat = calculateSigmaCubic(time, timeFinal, omega0);
      double sigmaT = calculateSigmaCubic(timeFinal, timeFinal, omega0);
      
      double alpha = (1.0 - sigmat - Math.exp(omega0*(time-timeFinal)) * (1.0 - sigmaT));
      double beta = (sigmat - Math.exp(omega0*(time-timeFinal)) * sigmaT);
      double gamma =  Math.exp(omega0*(time-timeFinal));
      
      icpPositionDesiredCurrent.setToZero();
      icpPositionDesiredCurrent.scaleAdd(alpha, cmpRefInit             , icpPositionDesiredCurrent);
      icpPositionDesiredCurrent.scaleAdd(beta, cmpRefFinal             , icpPositionDesiredCurrent);
      icpPositionDesiredCurrent.scaleAdd(gamma, icpPositionDesiredFinal, icpPositionDesiredCurrent);
   }
   
   public static void calculateICPVelocityByHand3DLinear(double omega0, double time, FrameTrajectory3D linear3D, FramePoint3D icpPositionDesiredFinal, FrameVector3D icpVelocityDesiredCurrent)
   {      
      linear3D.compute(linear3D.getInitialTime());
      FramePoint3D cmpRefInit = new FramePoint3D(linear3D.getFramePosition());
      
      linear3D.compute(linear3D.getFinalTime());
      FramePoint3D cmpRefFinal = new FramePoint3D(linear3D.getFramePosition());
      
      double timeFinal = linear3D.getFinalTime();
      
      double dSigmat = calculateSigmaDotLinear(time, timeFinal, omega0);
      double sigmaT = calculateSigmaLinear(timeFinal, timeFinal, omega0);
      
      double dAlpha = (-dSigmat - omega0 * Math.exp(omega0*(time-timeFinal)) * (1.0 - sigmaT));
      double dBeta = (dSigmat - omega0 * Math.exp(omega0*(time-timeFinal)) * sigmaT);
      double dGamma = omega0 * Math.exp(omega0*(time-timeFinal));
      
      icpVelocityDesiredCurrent.setToZero();
      icpVelocityDesiredCurrent.scaleAdd(dAlpha, cmpRefInit             , icpVelocityDesiredCurrent);
      icpVelocityDesiredCurrent.scaleAdd(dBeta, cmpRefFinal             , icpVelocityDesiredCurrent);
      icpVelocityDesiredCurrent.scaleAdd(dGamma, icpPositionDesiredFinal, icpVelocityDesiredCurrent);
   }
   
   public static void calculateICPVelocityByHand3DCubic(double omega0, double time, FrameTrajectory3D cubic3D, FramePoint3D icpPositionDesiredFinal, FrameVector3D icpVelocityDesiredCurrent)
   {      
      cubic3D.compute(cubic3D.getInitialTime());
      FramePoint3D cmpRefInit = new FramePoint3D(cubic3D.getFramePosition());
      
      cubic3D.compute(cubic3D.getFinalTime());
      FramePoint3D cmpRefFinal = new FramePoint3D(cubic3D.getFramePosition());
      
      double timeFinal = cubic3D.getFinalTime();
      
      double dSigmat = calculateSigmaDotCubic(time, timeFinal, omega0);
      double sigmaT = calculateSigmaCubic(timeFinal, timeFinal, omega0);
      
      double dAlpha = (-dSigmat - omega0 * Math.exp(omega0*(time-timeFinal)) * (1.0 - sigmaT));
      double dBeta = (dSigmat - omega0 * Math.exp(omega0*(time-timeFinal)) * sigmaT);
      double dGamma = omega0 * Math.exp(omega0*(time-timeFinal));
      
      icpVelocityDesiredCurrent.setToZero();
      icpVelocityDesiredCurrent.scaleAdd(dAlpha, cmpRefInit.getPoint()             , icpVelocityDesiredCurrent);
      icpVelocityDesiredCurrent.scaleAdd(dBeta, cmpRefFinal.getPoint()             , icpVelocityDesiredCurrent);
      icpVelocityDesiredCurrent.scaleAdd(dGamma, icpPositionDesiredFinal.getPoint(), icpVelocityDesiredCurrent);
   }
   
   public static double calculateSigmaLinear(double t, double T, double omega0)
   {
      double sigmaLinear = t/T + 1.0/omega0 * 1/T;
      return sigmaLinear;
   }
   
   public static double calculateSigmaCubic(double t, double T, double omega0)
   {
      double f = 3.0 * Math.pow(t, 2) / Math.pow(T, 2) - 2.0 * Math.pow(t, 3) / Math.pow(T, 3);
      double df = 6.0 * Math.pow(t, 1) / Math.pow(T, 2) - 6.0 * Math.pow(t, 2) / Math.pow(T, 3);
      double ddf = 6.0 * 1.0 / Math.pow(T, 2) - 12.0 * Math.pow(t, 1) / Math.pow(T, 3);
      double dddf = -12.0 / Math.pow(T, 3);
      
      double sigmaCubic = f + Math.pow(1.0/omega0, 1) * df + Math.pow(1.0/omega0, 2) * ddf + Math.pow(1.0/omega0, 3) * dddf;
      return sigmaCubic;
   }
   
   public static double calculateSigmaDotLinear(double t, double T, double omega0)
   {
      double dSigmaLinear = 1/T;
      return dSigmaLinear;
   }
   
   public static double calculateSigmaDotCubic(double t, double T, double omega0)
   {
      double df = 6.0 * Math.pow(t, 1) / Math.pow(T, 2) - 6.0 * Math.pow(t, 2) / Math.pow(T, 3);
      double ddf = 6.0 * 1.0 / Math.pow(T, 2) - 12.0 * Math.pow(t, 1) / Math.pow(T, 3);
      double dddf = -12.0 / Math.pow(T, 3);
      
      double dSigmaCubic = df + Math.pow(1.0/omega0, 1) * ddf + Math.pow(1.0/omega0, 2) * dddf;
      return dSigmaCubic;
   }
      
      
   public static void calculateAlphaPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F alphaPrimeLinear)
   {
      alphaPrimeLinear.set(0, 0, 1);
      alphaPrimeLinear.set(0, 1, time + 1.0/omega0);
      
      alphaPrimeLinear.set(1, 2, 1);
      alphaPrimeLinear.set(1, 3, time + 1.0/omega0);
      
      alphaPrimeLinear.set(2, 4, 1);
      alphaPrimeLinear.set(2, 5, time + 1.0/omega0);
   }
   
   public static void calculateBetaPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F betaPrimeLinear)
   {
      betaPrimeLinear.set(0, 0, Math.exp(omega0 * (time - timeTotal))*1);
      betaPrimeLinear.set(0, 1, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
      
      betaPrimeLinear.set(1, 2, Math.exp(omega0 * (time - timeTotal))*1);
      betaPrimeLinear.set(1, 3, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
      
      betaPrimeLinear.set(2, 4, Math.exp(omega0 * (time - timeTotal))*1);
      betaPrimeLinear.set(2, 5, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
   }
   
   public static void calculateGammaPrime3DByHandLinear(double omega0, double time, double timeTotal, DenseMatrix64F gammaPrimeLinear)
   {
      gammaPrimeLinear.set(0, 0, Math.exp(omega0 * (time - timeTotal)));
   }
   
   public static void calculateAlphaPrime3DByHandCubic(double omega0, double time, double timeTotal, DenseMatrix64F alphaPrimeLinear)
   {
      alphaPrimeLinear.set(0, 0, 1);
      alphaPrimeLinear.set(0, 1, time + 1.0/omega0);
      alphaPrimeLinear.set(0, 2, Math.pow(time, 2) + 2.0 * time/omega0 + 2.0/Math.pow(omega0, 2));
      alphaPrimeLinear.set(0, 3, Math.pow(time, 3) + 3.0 * Math.pow(time, 2)/omega0 + 6.0 * time/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3));
      
      alphaPrimeLinear.set(1, 4, 1);
      alphaPrimeLinear.set(1, 5, time + 1.0/omega0);
      alphaPrimeLinear.set(1, 6, Math.pow(time, 2) + 2.0 * time/omega0 + 2.0/Math.pow(omega0, 2));
      alphaPrimeLinear.set(1, 7, Math.pow(time, 3) + 3.0 * Math.pow(time, 2)/omega0 + 6.0 * time/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3));
      
      alphaPrimeLinear.set(2, 8, 1);
      alphaPrimeLinear.set(2, 9, time + 1.0/omega0);
      alphaPrimeLinear.set(2, 10, Math.pow(time, 2) + 2.0 * time/omega0 + 2.0/Math.pow(omega0, 2));
      alphaPrimeLinear.set(2, 11, Math.pow(time, 3) + 3.0 * Math.pow(time, 2)/omega0 + 6.0 * time/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3));
   }
   
   public static void calculateBetaPrime3DByHandCubic(double omega0, double time, double timeTotal, DenseMatrix64F betaPrimeLinear)
   {
      betaPrimeLinear.set(0, 0, Math.exp(omega0 * (time - timeTotal))*1);
      betaPrimeLinear.set(0, 1, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
      betaPrimeLinear.set(0, 2, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 2) + 2.0 * timeTotal/omega0 + 2.0/Math.pow(omega0, 2)));
      betaPrimeLinear.set(0, 3, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 3) + 3.0 * Math.pow(timeTotal, 2)/omega0 + 6.0 * timeTotal/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3)));
      
      betaPrimeLinear.set(1, 4, Math.exp(omega0 * (time - timeTotal))*1);
      betaPrimeLinear.set(1, 5, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
      betaPrimeLinear.set(1, 6, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 2) + 2.0 * timeTotal/omega0 + 2.0/Math.pow(omega0, 2)));
      betaPrimeLinear.set(1, 7, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 3) + 3.0 * Math.pow(timeTotal, 2)/omega0 + 6.0 * timeTotal/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3)));
      
      betaPrimeLinear.set(2, 8, Math.exp(omega0 * (time - timeTotal))*1);
      betaPrimeLinear.set(2, 9, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
      betaPrimeLinear.set(2, 10, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 2) + 2.0 * timeTotal/omega0 + 2.0/Math.pow(omega0, 2)));
      betaPrimeLinear.set(2, 11, Math.exp(omega0 * (time - timeTotal))*(Math.pow(timeTotal, 3) + 3.0 * Math.pow(timeTotal, 2)/omega0 + 6.0 * timeTotal/Math.pow(omega0, 2) + 6.0/Math.pow(omega0, 3)));
   }
   
   public static void calculateGammaPrime3DByHandCubic(double omega0, double time, double timeTotal, DenseMatrix64F gammaPrimeLinear)
   {
      gammaPrimeLinear.set(0, 0, Math.exp(omega0 * (time - timeTotal)));
   }
}
