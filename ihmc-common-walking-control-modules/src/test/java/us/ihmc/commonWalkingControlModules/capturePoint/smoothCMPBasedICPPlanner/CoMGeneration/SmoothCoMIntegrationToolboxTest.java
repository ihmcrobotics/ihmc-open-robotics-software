package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoMGeneration;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.DenseMatrixVector3D;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.SmoothCapturePointToolbox;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory;

public class SmoothCoMIntegrationToolboxTest
{
   private static final int nTests = 20;
   private static final double omega0 = 3.4;
   private static final double EPSILON = 10e-6;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private Random random = new Random();

   String namePrefix = "SmoothCoMIntegrationToolboxTest";

   private final SmoothCapturePointToolbox icpToolbox = new SmoothCapturePointToolbox();
   private final SmoothCoMIntegrationToolbox comToolbox = new SmoothCoMIntegrationToolbox();

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testMatricesCoMPrime3DLinear()
   {
      // Linear polynomial: y(x) = a0 + a1*x
      int numberOfCoefficients = 2;
      FrameTrajectory3D linear3D = new FrameTrajectory3D(numberOfCoefficients, worldFrame);

      for (int i = 0; i < nTests; i++)
      {
         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();

         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FramePoint3D cmpFinal = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));

         linear3D.setLinear(t0, tFinal, cmp0, cmpFinal);

         double time = t0 + Math.random() * (tFinal - t0);

         DMatrixRMaj alphaCoMPrimeAutomatic = new DMatrixRMaj(3, 3 * numberOfCoefficients);
         DMatrixRMaj alphaCoMPrimeManual = new DMatrixRMaj(3, 3 * numberOfCoefficients);

         SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(omega0, time, alphaCoMPrimeAutomatic, 0, linear3D);
         calculateAlphaCoMPrime3DByHandLinear(omega0, time, t0, tFinal, alphaCoMPrimeManual);

         //         PrintTools.debug("A linear calc: " + alphaCoMPrimeAutomatic.toString());
         //         PrintTools.debug("A linear test: " + alphaCoMPrimeManual.toString());

         for (int j = 0; j < alphaCoMPrimeAutomatic.getNumCols(); j++)
         {
            for (int k = 0; k < alphaCoMPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(alphaCoMPrimeAutomatic.get(k, j), alphaCoMPrimeManual.get(k, j), EPSILON);
            }
         }

         DMatrixRMaj betaCoMPrimeAutomatic = new DMatrixRMaj(3, 3 * numberOfCoefficients);
         DMatrixRMaj betaCoMPrimeManual = new DMatrixRMaj(3, 3 * numberOfCoefficients);

         SmoothCoMIntegrationToolbox.calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(omega0, time, betaCoMPrimeAutomatic, 0, linear3D);
         calculateBetaCoMPrime3DByHandLinear(omega0, time, t0, tFinal, betaCoMPrimeManual);

         //         PrintTools.debug("B linear calc: " + betaCoMPrimeAutomatic.toString());
         //         PrintTools.debug("B linear test: " + betaCoMPrimeManual.toString());

         for (int j = 0; j < betaCoMPrimeAutomatic.getNumCols(); j++)
         {
            for (int k = 0; k < betaCoMPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(betaCoMPrimeAutomatic.get(k, j), betaCoMPrimeManual.get(k, j), EPSILON);
            }
         }

         DMatrixRMaj gammaCoMPrimeManual = new DMatrixRMaj(1, 1);

         double gammaCoMPrimeAutomatic = SmoothCoMIntegrationToolbox.calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(omega0, time, 0, linear3D);
         calculateGammaCoMPrime3DByHandLinear(omega0, time, t0, tFinal, gammaCoMPrimeManual);

         //         PrintTools.debug("C linear calc: " + gammaCoMPrimeAutomatic.toString());
         //         PrintTools.debug("C linear test: " + gammaCoMPrimeManual.toString());

         assertEquals(gammaCoMPrimeAutomatic, gammaCoMPrimeManual.get(0), EPSILON);

         double deltaCoMPrimeAutomatic = SmoothCoMIntegrationToolbox.calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(omega0, time, 0, linear3D);
         double deltaCoMPrimeManual = calculateDeltaCoMPrime3DByHandLinear(omega0, time, t0, tFinal);

         assertEquals(deltaCoMPrimeAutomatic, deltaCoMPrimeManual, EPSILON);
      }
   }

   @Test
   public void testDMatricesCoMPrime3DLinear()
   {
      // Linear polynomial: y(x) = a0 + a1*x
      int numberOfCoefficients = 2;
      FrameTrajectory3D linear3D = new FrameTrajectory3D(numberOfCoefficients, worldFrame);

      for (int i = 0; i < nTests; i++)
      {
         double scaleTFinal = 1.0 / Math.random();
         double t0 = 0.0, tFinal = t0 + scaleTFinal * Math.random();

         FramePoint3D cmp0 = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));
         FramePoint3D cmpFinal = new FramePoint3D(worldFrame, new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()));

         linear3D.setLinear(t0, tFinal, cmp0, cmpFinal);

         double time = t0 + Math.random() * (tFinal - t0);

         DMatrixRMaj dAlphaCoMPrimeAutomatic = new DMatrixRMaj(3, 3 * numberOfCoefficients);
         DMatrixRMaj dAlphaCoMPrimeManual = new DMatrixRMaj(3, 3 * numberOfCoefficients);

         SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(omega0, time, dAlphaCoMPrimeAutomatic, 1, linear3D);
         calculateDAlphaCoMPrime3DByHandLinear(omega0, time, t0, tFinal, dAlphaCoMPrimeManual);

         //         PrintTools.debug("dA linear calc: " + dAlphaCoMPrimeAutomatic.toString());
         //         PrintTools.debug("dA linear test: " + dAlphaCoMPrimeManual.toString());

         for (int j = 0; j < dAlphaCoMPrimeAutomatic.getNumCols(); j++)
         {
            for (int k = 0; k < dAlphaCoMPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(dAlphaCoMPrimeAutomatic.get(k, j), dAlphaCoMPrimeManual.get(k, j), EPSILON);
            }
         }

         DMatrixRMaj dBetaCoMPrimeAutomatic = new DMatrixRMaj(3, 3 * numberOfCoefficients);
         DMatrixRMaj dBetaCoMPrimeManual = new DMatrixRMaj(3, 3 * numberOfCoefficients);

         SmoothCoMIntegrationToolbox.calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(omega0, time, dBetaCoMPrimeAutomatic, 1, linear3D);
         calculateDBetaCoMPrime3DByHandLinear(omega0, time, t0, tFinal, dBetaCoMPrimeManual);

         //         PrintTools.debug("dB linear calc: " + dBetaCoMPrimeAutomatic.toString());
         //         PrintTools.debug("dB linear test: " + dBetaCoMPrimeManual.toString());

         for (int j = 0; j < dBetaCoMPrimeAutomatic.getNumCols(); j++)
         {
            for (int k = 0; k < dBetaCoMPrimeAutomatic.getNumRows(); k++)
            {
               assertEquals(dBetaCoMPrimeAutomatic.get(k, j), dBetaCoMPrimeManual.get(k, j), EPSILON);
            }
         }

         DMatrixRMaj dGammaCoMPrimeManual = new DMatrixRMaj(1, 1);

         double dGammaCoMPrimeAutomatic = SmoothCoMIntegrationToolbox.calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(omega0, time, 1, linear3D);
         calculateDGammaCoMPrime3DByHandLinear(omega0, time, t0, tFinal, dGammaCoMPrimeManual);

         //         PrintTools.debug("dC linear calc: " + dGammaCoMPrimeAutomatic.toString());
         //         PrintTools.debug("dC linear test: " + dGammaCoMPrimeManual.toString());

         assertEquals(dGammaCoMPrimeAutomatic, dGammaCoMPrimeManual.get(0), EPSILON);

         double dDeltaCoMPrimeAutomatic = SmoothCoMIntegrationToolbox.calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(omega0, time, 1, linear3D);
         double dDeltaCoMPrimeManual = calculateDDeltaCoMPrime3DByHandLinear(omega0, time, t0, tFinal);

         //         PrintTools.debug("dD linear calc: " + dDeltaCoMPrimeAutomatic.toString());
         //         PrintTools.debug("dD linear test: " + dDeltaCoMPrimeManual.toString());

         assertEquals(dDeltaCoMPrimeAutomatic, dDeltaCoMPrimeManual, EPSILON);
      }
   }

   @Test
   public void testCalculateCoMPositionAndVelocityOnSegment3DLinear()
   {
      // Linear polynomial: y(x) = a0 + a1*x
      int numberOfCoefficients = 2;
      FrameTrajectory3D linear3D = new FrameTrajectory3D(numberOfCoefficients, worldFrame);

      for (int i = 0; i < nTests; i++)
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

         comToolbox.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, linear3D, icpPositionDesiredFinal, comPositionDesiredInitial,
                                                                         comPositionDesiredCurrent);
         comToolbox.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, tFinal, 0, linear3D, icpPositionDesiredFinal, comPositionDesiredInitial,
                                                                         comPositionDesiredFinal);
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

         comToolbox.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 1, linear3D, icpPositionDesiredFinal, comPositionDesiredInitial,
                                                                         comVelocityDesiredCurrent);
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

   @Test
   public void testCalculateCoMQuantity3DWithDenseMatrixVector3D() throws Exception
   {
      Random random = new Random(2432);
      SmoothCoMIntegrationToolbox toolbox = new SmoothCoMIntegrationToolbox();

      for (int i = 0; i < nTests; i++)
      {
         int nPolynomialCoeffs = random.nextInt(9) + 1;
         DMatrixRMaj alphaBetaCoMPrime = new DMatrixRMaj(3, 3 * nPolynomialCoeffs);
         DenseMatrixVector3D alphaBetaCoMPrimeVector = new DenseMatrixVector3D(1, nPolynomialCoeffs);
         DMatrixRMaj alphaPrimeTerminal = new DMatrixRMaj(3, 3 * nPolynomialCoeffs);
         DenseMatrixVector3D alphaPrimeTerminalVector = new DenseMatrixVector3D(1, nPolynomialCoeffs);

         for (int row = 0; row < 3; row++)
         {
            int colOffset = row * nPolynomialCoeffs;

            for (int col = 0; col < nPolynomialCoeffs; col++)
            {
               double randomCoeff = RandomNumbers.nextDouble(random, 1.0);
               alphaBetaCoMPrime.set(row, colOffset + col, randomCoeff);
               alphaBetaCoMPrimeVector.getMatrix(row).set(col, randomCoeff);

               randomCoeff = RandomNumbers.nextDouble(random, 1.0);
               alphaPrimeTerminal.set(row, colOffset + col, randomCoeff);
               alphaPrimeTerminalVector.getMatrix(row).set(col, randomCoeff);
            }
         }

         DMatrixRMaj polynomialCoeffs = new DMatrixRMaj(3 * nPolynomialCoeffs, 1);
         DenseMatrixVector3D polynomialCoeffsVector = new DenseMatrixVector3D(nPolynomialCoeffs, 1);

         for (Axis3D axis : Axis3D.values)
         {
            int rowOffset = nPolynomialCoeffs * axis.ordinal();

            for (int row = 0; row < nPolynomialCoeffs; row++)
            {
               double randomCoeff = RandomNumbers.nextDouble(random, 1.0);
               polynomialCoeffs.set(row + rowOffset, 0, randomCoeff);
               polynomialCoeffsVector.getMatrix(axis).set(row, randomCoeff);
            }
         }

         double gammaCoMPrime = RandomNumbers.nextDouble(random, 1.0);
         double deltaCoMPrime = RandomNumbers.nextDouble(random, 1.0);
         FrameTuple3DReadOnly icpPositionDesiredFinal = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame);
         FrameTuple3DReadOnly comPositionDesiredInitial = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame);

         FramePoint3D expectedCoMQuantityDesired = new FramePoint3D(worldFrame);
         FramePoint3D actualCoMQuantityDesired = new FramePoint3D(worldFrame);

         toolbox.calculateCoMQuantity3D(alphaBetaCoMPrime, gammaCoMPrime, deltaCoMPrime, alphaPrimeTerminal, polynomialCoeffs, icpPositionDesiredFinal,
                                        comPositionDesiredInitial, expectedCoMQuantityDesired);
         toolbox.calculateCoMQuantity3D(alphaBetaCoMPrimeVector, gammaCoMPrime, deltaCoMPrime, alphaPrimeTerminalVector, polynomialCoeffsVector,
                                        icpPositionDesiredFinal, comPositionDesiredInitial, actualCoMQuantityDesired);

         EuclidFrameTestTools.assertFrameTuple3DEquals(expectedCoMQuantityDesired, actualCoMQuantityDesired, EPSILON);
      }
   }

   @Test
   public void testCalculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D() throws Exception
   {
      Random random = new Random(545645);

      for (int i = 0; i < nTests; i++)
      {
         double t0 = random.nextDouble();
         double tf = t0 + RandomNumbers.nextDouble(random, 0.25, 1.0);
         double time = RandomNumbers.nextDouble(random, t0, tf);
         int alphaBetaCoMDerivativeOrder = random.nextInt(3);

         FrameTrajectory3D cmpPolynomial3D = new FrameTrajectory3D(10, worldFrame);
         cmpPolynomial3D.setCubic(t0, tf, EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame),
                                  EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame));

         DMatrixRMaj generalizedAlphaCoMPrime = new DMatrixRMaj(3, 3 * cmpPolynomial3D.getNumberOfCoefficients());
         DMatrixRMaj generalizedBetaCoMPrime = new DMatrixRMaj(3, 3 * cmpPolynomial3D.getNumberOfCoefficients());
         DMatrixRMaj expectedGeneralizedAlphaBetaCoMPrime = new DMatrixRMaj(3, 3 * cmpPolynomial3D.getNumberOfCoefficients());

         SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(omega0, time, generalizedAlphaCoMPrime, alphaBetaCoMDerivativeOrder,
                                                                                     cmpPolynomial3D);
         SmoothCoMIntegrationToolbox.calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(omega0, time, generalizedBetaCoMPrime, alphaBetaCoMDerivativeOrder,
                                                                                    cmpPolynomial3D);
         CommonOps_DDRM.subtract(generalizedAlphaCoMPrime, generalizedBetaCoMPrime, expectedGeneralizedAlphaBetaCoMPrime);

         DMatrixRMaj actualGeneralizedAlphaBetaCoMPrime = new DMatrixRMaj(3, 3 * cmpPolynomial3D.getNumberOfCoefficients());
         SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(omega0, time, actualGeneralizedAlphaBetaCoMPrime,
                                                                                         alphaBetaCoMDerivativeOrder, cmpPolynomial3D);

         DMatrixRMaj error = new DMatrixRMaj(3, 3 * cmpPolynomial3D.getNumberOfCoefficients());
         CommonOps_DDRM.subtract(expectedGeneralizedAlphaBetaCoMPrime, actualGeneralizedAlphaBetaCoMPrime, error);
         for (int j = 0; j < error.getNumElements(); j++)
            error.set(j, Math.abs(error.get(j)));
         assertTrue("Expected: " + expectedGeneralizedAlphaBetaCoMPrime + "\nwas: " + actualGeneralizedAlphaBetaCoMPrime + "\nerror: " + error,
                    MatrixFeatures_DDRM.isEquals(expectedGeneralizedAlphaBetaCoMPrime, actualGeneralizedAlphaBetaCoMPrime, 1.0e-12));
      }
      for (int i = 0; i < nTests; i++)
      { // Test calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D with DenseMatrixVector3D
         double t0 = random.nextDouble();
         double tf = t0 + RandomNumbers.nextDouble(random, 0.25, 1.0);
         double time = RandomNumbers.nextDouble(random, t0, tf);
         int alphaBetaCoMDerivativeOrder = random.nextInt(3);

         FrameTrajectory3D cmpPolynomial3D = new FrameTrajectory3D(10, worldFrame);
         cmpPolynomial3D.setCubic(t0, tf, EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame),
                                  EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame));

         DMatrixRMaj expectedGeneralizedAlphaBetaCoMPrime = new DMatrixRMaj(3, 3 * cmpPolynomial3D.getNumberOfCoefficients());

         SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(omega0, time, expectedGeneralizedAlphaBetaCoMPrime,
                                                                                         alphaBetaCoMDerivativeOrder, cmpPolynomial3D);

         DenseMatrixVector3D actualGeneralizedAlphaBetaCoMPrime = new DenseMatrixVector3D(1, 3 * cmpPolynomial3D.getNumberOfCoefficients());

         SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(omega0, time, actualGeneralizedAlphaBetaCoMPrime,
                                                                                         alphaBetaCoMDerivativeOrder, cmpPolynomial3D);

         DMatrixRMaj error = new DMatrixRMaj(3, 3 * cmpPolynomial3D.getNumberOfCoefficients());

         for (int k = 0; k < actualGeneralizedAlphaBetaCoMPrime.getMatrix(Axis3D.X).numCols; k++)
            error.set(0, k, expectedGeneralizedAlphaBetaCoMPrime.get(0, k) - actualGeneralizedAlphaBetaCoMPrime.getMatrix(Axis3D.X).get(0, k));

         int colOffset = actualGeneralizedAlphaBetaCoMPrime.getMatrix(Axis3D.X).numCols;
         for (int k = 0; k < actualGeneralizedAlphaBetaCoMPrime.getMatrix(Axis3D.Y).numCols; k++)
            error.set(1, colOffset + k,
                      expectedGeneralizedAlphaBetaCoMPrime.get(1, colOffset + k) - actualGeneralizedAlphaBetaCoMPrime.getMatrix(Axis3D.Y).get(0, k));

         colOffset += actualGeneralizedAlphaBetaCoMPrime.getMatrix(Axis3D.Y).numCols;
         for (int k = 0; k < actualGeneralizedAlphaBetaCoMPrime.getMatrix(Axis3D.Z).numCols; k++)
            error.set(2, colOffset + k,
                      expectedGeneralizedAlphaBetaCoMPrime.get(2, colOffset + k) - actualGeneralizedAlphaBetaCoMPrime.getMatrix(Axis3D.Z).get(0, k));

         for (int j = 0; j < error.getNumElements(); j++)
            error.set(j, Math.abs(error.get(j)));
         assertTrue("Expected: " + expectedGeneralizedAlphaBetaCoMPrime + "\nwas: " + actualGeneralizedAlphaBetaCoMPrime + "\nerror: " + error,
                    MatrixFeatures_DDRM.isZeros(error, 1.0e-12));
      }

      for (int i = 0; i < nTests; i++)
      { // Test calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D with for 0-th, 1-st, and 3-rd derivatives
         double t0 = random.nextDouble();
         double tf = t0 + RandomNumbers.nextDouble(random, 0.25, 1.0);
         double time = RandomNumbers.nextDouble(random, t0, tf);

         FrameTrajectory3D cmpPolynomial3D = new FrameTrajectory3D(10, worldFrame);
         cmpPolynomial3D.setCubic(t0, tf, EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame),
                                  EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame));

         DenseMatrixVector3D expectedAlphaBetaCoMPrime = new DenseMatrixVector3D(1, cmpPolynomial3D.getNumberOfCoefficients());
         DenseMatrixVector3D expectedAlphaBetaCoMSecond = new DenseMatrixVector3D(1, cmpPolynomial3D.getNumberOfCoefficients());
         DenseMatrixVector3D expectedAlphaBetaCoMThird = new DenseMatrixVector3D(1, cmpPolynomial3D.getNumberOfCoefficients());

         SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(omega0, time, expectedAlphaBetaCoMPrime, 0, cmpPolynomial3D);
         SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(omega0, time, expectedAlphaBetaCoMSecond, 1, cmpPolynomial3D);
         SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(omega0, time, expectedAlphaBetaCoMThird, 2, cmpPolynomial3D);

         DenseMatrixVector3D actualAlphaBetaCoMPrime = new DenseMatrixVector3D(1, cmpPolynomial3D.getNumberOfCoefficients());
         DenseMatrixVector3D actualAlphaBetaCoMSecond = new DenseMatrixVector3D(1, cmpPolynomial3D.getNumberOfCoefficients());
         DenseMatrixVector3D actualAlphaBetaCoMThird = new DenseMatrixVector3D(1, cmpPolynomial3D.getNumberOfCoefficients());
         SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(omega0, time, actualAlphaBetaCoMPrime, actualAlphaBetaCoMSecond,
                                                                                         actualAlphaBetaCoMThird, cmpPolynomial3D);

         DenseMatrixVector3D errorAlphaBetaCoMPrime = new DenseMatrixVector3D(1, cmpPolynomial3D.getNumberOfCoefficients());
         DenseMatrixVector3D errorAlphaBetaCoMSecond = new DenseMatrixVector3D(1, cmpPolynomial3D.getNumberOfCoefficients());
         DenseMatrixVector3D errorAlphaBetaCoMThird = new DenseMatrixVector3D(1, cmpPolynomial3D.getNumberOfCoefficients());

         errorAlphaBetaCoMPrime.sub(expectedAlphaBetaCoMPrime, actualAlphaBetaCoMPrime);
         errorAlphaBetaCoMSecond.sub(expectedAlphaBetaCoMSecond, actualAlphaBetaCoMSecond);
         errorAlphaBetaCoMThird.sub(expectedAlphaBetaCoMThird, actualAlphaBetaCoMThird);

         assertTrue("Expected: " + expectedAlphaBetaCoMPrime + "\nwas: " + actualAlphaBetaCoMPrime + "\nerror: " + errorAlphaBetaCoMPrime,
                    expectedAlphaBetaCoMPrime.epsilonEquals(actualAlphaBetaCoMPrime, EPSILON));
         assertTrue("Expected: " + expectedAlphaBetaCoMSecond + "\nwas: " + actualAlphaBetaCoMSecond + "\nerror: " + errorAlphaBetaCoMSecond,
                    expectedAlphaBetaCoMSecond.epsilonEquals(actualAlphaBetaCoMSecond, EPSILON));
         assertTrue("Expected: " + expectedAlphaBetaCoMThird + "\nwas: " + actualAlphaBetaCoMThird + "\nerror: " + errorAlphaBetaCoMThird,
                    expectedAlphaBetaCoMThird.epsilonEquals(actualAlphaBetaCoMThird, EPSILON));
      }
   }

   @Test
   public void testComputeDesiredCenterOfMassPositionVelocityAcceleration() throws Exception
   {
      Random random = new Random(453453);
      SmoothCoMIntegrationToolbox toolbox = new SmoothCoMIntegrationToolbox();

      for (int i = 0; i < nTests; i++)
      {
         double t0 = random.nextDouble();
         double tf = t0 + RandomNumbers.nextDouble(random, 0.25, 1.0);
         double time = RandomNumbers.nextDouble(random, t0, tf);
         FrameTrajectory3D cmpPolynomial3D = new FrameTrajectory3D(16, worldFrame);
         cmpPolynomial3D.setCubic(t0, tf, EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame),
                                  EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame));

         FramePoint3DReadOnly finalCapturePoint = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame);
         FramePoint3DReadOnly initialCenterOfMass = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame);

         FramePoint3D expectedCenterOfMassPosition = new FramePoint3D(worldFrame);
         FrameVector3D expectedCenterOfMassVelocity = new FrameVector3D(worldFrame);
         FrameVector3D expectedCenterOfMassAcceleration = new FrameVector3D(worldFrame);
         toolbox.computeDesiredCenterOfMassPosition(omega0, time, finalCapturePoint, initialCenterOfMass, cmpPolynomial3D, expectedCenterOfMassPosition);
         toolbox.computeDesiredCenterOfMassVelocity(omega0, time, finalCapturePoint, initialCenterOfMass, cmpPolynomial3D, expectedCenterOfMassVelocity);
         toolbox.computeDesiredCenterOfMassAcceleration(omega0, time, finalCapturePoint, initialCenterOfMass, cmpPolynomial3D, expectedCenterOfMassAcceleration);

         FramePoint3D actualCenterOfMassPosition = new FramePoint3D(worldFrame);
         FrameVector3D actualCenterOfMassVelocity = new FrameVector3D(worldFrame);
         FrameVector3D actualCenterOfMassAcceleration = new FrameVector3D(worldFrame);
         toolbox.computeDesiredCenterOfMassPositionVelocityAcceleration(omega0, time, finalCapturePoint, initialCenterOfMass, cmpPolynomial3D,
                                                                        actualCenterOfMassPosition, actualCenterOfMassVelocity, actualCenterOfMassAcceleration);

         EuclidFrameTestTools.assertFrameTuple3DEquals(expectedCenterOfMassPosition, actualCenterOfMassPosition, EPSILON);
         EuclidFrameTestTools.assertFrameTuple3DEquals(expectedCenterOfMassVelocity, actualCenterOfMassVelocity, EPSILON);
         EuclidFrameTestTools.assertFrameTuple3DEquals(expectedCenterOfMassAcceleration, actualCenterOfMassAcceleration, EPSILON);
      }
   }

   @Test
   public void testPower() throws Exception
   {
      Random random = new Random(34549037);

      for (int i = 0; i < 5000; i++)
      {
         double scalar = RandomNumbers.nextDouble(random, 10.0);
         int exponent = random.nextInt(10);
         double expected = Math.pow(scalar, exponent);
         double actual = SmoothCoMIntegrationToolbox.power(scalar, exponent);
         double epsilon = Math.min(Math.abs(expected) * 1.0e-15, 1.0e-6);
         assertEquals("Difference: " + Math.abs(expected - actual), expected, actual, epsilon);
      }
   }

   @Test
   public void testBugWithDataset()
   {
      double omega0 = 3.0;
      double time = 0.012;
      Trajectory xCMPTrajectory = new Trajectory(0.0, 0.2, new double[] {1.000030572996156, 0.3152786856832297, -0.5121259090354553, 2.115440848323875,
            7.120997871247123E-16, 1.837333165325549E-14, -1.1985177263371334E-13, 1.4665174726420681E-13, 1.5074587857295702E-16});//, Double.NaN});
      Trajectory yCMPTrajectory = new Trajectory(0.0, 0.2, new double[] {-13.488553923816662, 0.034554264879721235, -0.056128546318945906, 0.23185044448052147,
            6.674785404512585E-17, 2.0742558091635797E-15, -1.3485956037477206E-14, 1.6516519718330272E-14, 3.4121533325635544E-17}); //, Double.NaN});
      Trajectory zCMPTrajectory = new Trajectory(0.0, 0.2, new double[] {0.001261447302717669, 6.757841918973695E-5}); //, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN});
      FrameTrajectory3D cmpTrajectory = new FrameTrajectory3D(xCMPTrajectory, yCMPTrajectory, zCMPTrajectory, worldFrame);
      FramePoint3D icpFinal = new FramePoint3D(worldFrame, 0.977, -13.491, 0.001);
      FramePoint3D comInitial = new FramePoint3D(worldFrame, 1.000, -13.489, 0.001);


      DenseMatrixVector3D expectedAlphaBetaCoMPrime = new DenseMatrixVector3D(10, 10);
      DenseMatrixVector3D expectedAlphaBetaCoMSecond = new DenseMatrixVector3D(10, 10);
      DenseMatrixVector3D expectedAlphaBetaCoMThird = new DenseMatrixVector3D(10, 10);
      SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(omega0, time, expectedAlphaBetaCoMPrime, 0, cmpTrajectory);
      SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(omega0, time, expectedAlphaBetaCoMSecond, 1, cmpTrajectory);
      SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(omega0, time, expectedAlphaBetaCoMThird, 2, cmpTrajectory);

      DenseMatrixVector3D actualAlphaBetaCoMPrime = new DenseMatrixVector3D(10, 10);
      DenseMatrixVector3D actualAlphaBetaCoMSecond = new DenseMatrixVector3D(10, 10);
      DenseMatrixVector3D actualAlphaBetaCoMThird = new DenseMatrixVector3D(10, 10);
      SmoothCoMIntegrationToolbox.calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(omega0, time, actualAlphaBetaCoMPrime, actualAlphaBetaCoMSecond, actualAlphaBetaCoMThird, cmpTrajectory);

      assertTrue(expectedAlphaBetaCoMPrime.epsilonEquals(actualAlphaBetaCoMPrime, EPSILON));
      assertTrue(expectedAlphaBetaCoMSecond.epsilonEquals(actualAlphaBetaCoMSecond, EPSILON));
      assertTrue(expectedAlphaBetaCoMThird.epsilonEquals(actualAlphaBetaCoMThird, EPSILON));


      SmoothCoMIntegrationToolbox toolbox = new SmoothCoMIntegrationToolbox();

      FramePoint3D expectedCoMPosition = new FramePoint3D(worldFrame);
      FrameVector3D expectedCoMVelocity = new FrameVector3D(worldFrame);
      FrameVector3D expectedCoMAcceleration = new FrameVector3D(worldFrame);

      FramePoint3D actualCoMPosition = new FramePoint3D(worldFrame);
      FrameVector3D actualCoMVelocity = new FrameVector3D(worldFrame);
      FrameVector3D actualCoMAcceleration = new FrameVector3D(worldFrame);

      toolbox.computeDesiredCenterOfMassPosition(omega0, time, icpFinal, comInitial, cmpTrajectory, expectedCoMPosition);
      toolbox.computeDesiredCenterOfMassVelocity(omega0, time, icpFinal, comInitial, cmpTrajectory, expectedCoMVelocity);
      toolbox.computeDesiredCenterOfMassAcceleration(omega0, time, icpFinal, comInitial, cmpTrajectory, expectedCoMAcceleration);
      toolbox.computeDesiredCenterOfMassPositionVelocityAcceleration(omega0, time, icpFinal, comInitial, cmpTrajectory, actualCoMPosition, actualCoMVelocity,
                                                                     actualCoMAcceleration);

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(expectedCoMPosition, actualCoMPosition, EPSILON);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(expectedCoMVelocity, actualCoMVelocity, EPSILON);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(expectedCoMAcceleration, actualCoMAcceleration, EPSILON);
   }

   public static void calculateCoMPositionByHand3DLinear(double omega0, double time, FrameTrajectory3D linear3D, FramePoint3D icpPositionDesiredFinal,
                                                         FramePoint3D comPositionDesiredInitial, FramePoint3D comPositionDesiredCurrent)
   {
      linear3D.compute(linear3D.getInitialTime());
      FramePoint3D cmpRefInit = new FramePoint3D(linear3D.getFramePosition());

      linear3D.compute(linear3D.getFinalTime());
      FramePoint3D cmpRefFinal = new FramePoint3D(linear3D.getFramePosition());

      double timeInitial = linear3D.getInitialTime();
      double timeFinal = linear3D.getFinalTime();

      double at = 1 - Math.exp(omega0 * (timeInitial - time));
      double bt = time - timeInitial * Math.exp(omega0 * (timeInitial - time)) - 1.0 / omega0 + 1.0 / omega0 * Math.exp(omega0 * (timeInitial - time));
      double ct = 0.5 * Math.exp(omega0 * (timeInitial - timeFinal)) * (Math.exp(omega0 * (time - timeInitial)) - Math.exp(omega0 * (timeInitial - time)));

      //      PrintTools.debug("at = " + at);
      //      PrintTools.debug("bt = " + bt);
      //      PrintTools.debug("ct = " + ct);

      comPositionDesiredCurrent.setToZero();
      for (int i = 0; i < 3; i++)
      {
         double a0 = (1.0 - 1.0 / (omega0 * timeFinal)) * cmpRefInit.getElement(i) + 1.0 / (omega0 * timeFinal) * cmpRefFinal.getElement(i);
         double b0 = 1.0 / timeFinal * (cmpRefFinal.getElement(i) - cmpRefInit.getElement(i));
         double c0 = 1.0 / (omega0 * timeFinal) * cmpRefInit.getElement(i) - (1.0 + 1.0 / (omega0 * timeFinal)) * cmpRefFinal.getElement(i)
               + icpPositionDesiredFinal.getElement(i);

         //         PrintTools.debug("a0 = " + a0);
         //         PrintTools.debug("b0 = " + b0);
         //         PrintTools.debug("c0 = " + c0);
         comPositionDesiredCurrent.setElement(i, Math.exp(omega0 * (timeInitial - time)) * comPositionDesiredInitial.getElement(i)
               + (a0 * at + b0 * bt + c0 * ct));
      }
   }

   public static void calculateCoMVelocityByHand3DLinear(double omega0, double time, FrameTrajectory3D linear3D, FramePoint3D icpPositionDesiredFinal,
                                                         FramePoint3D comPositionDesiredFinal, FrameVector3D comVelocityDesiredCurrent)
   {
      linear3D.compute(linear3D.getInitialTime());
      FramePoint3D cmpRefInit = new FramePoint3D(linear3D.getFramePosition());

      linear3D.compute(linear3D.getFinalTime());
      FramePoint3D cmpRefFinal = new FramePoint3D(linear3D.getFramePosition());

      double timeInitial = linear3D.getInitialTime();
      double timeFinal = linear3D.getFinalTime();

      double dat = omega0 * Math.exp(omega0 * (timeInitial - time));
      double dbt = 1 + omega0 * timeInitial * Math.exp(omega0 * (timeInitial - time)) - Math.exp(omega0 * (timeInitial - time));
      double dct = 0.5 * Math.exp(omega0 * (timeInitial - timeFinal))
            * (omega0 * Math.exp(omega0 * (time - timeInitial)) + omega0 * Math.exp(omega0 * (timeInitial - time)));

      //      PrintTools.debug("dat = " + dat);
      //      PrintTools.debug("dbt = " + dbt);
      //      PrintTools.debug("dct = " + dct);

      comVelocityDesiredCurrent.setToZero();
      for (int i = 0; i < 3; i++)
      {
         double a0 = (1.0 - 1.0 / (omega0 * timeFinal)) * cmpRefInit.getElement(i) + 1.0 / (omega0 * timeFinal) * cmpRefFinal.getElement(i);
         double b0 = 1.0 / timeFinal * (cmpRefFinal.getElement(i) - cmpRefInit.getElement(i));
         double c0 = 1.0 / (omega0 * timeFinal) * cmpRefInit.getElement(i) - (1.0 + 1.0 / (omega0 * timeFinal)) * cmpRefFinal.getElement(i)
               + icpPositionDesiredFinal.getElement(i);

         //         PrintTools.debug("a0 = " + a0);
         //         PrintTools.debug("b0 = " + b0);
         //         PrintTools.debug("c0 = " + c0);
         comVelocityDesiredCurrent.setElement(i, -omega0 * Math.exp(omega0 * (timeInitial - time)) * comPositionDesiredFinal.getElement(i)
               + (a0 * dat + b0 * dbt + c0 * dct));
      }
   }

   public static void calculateAlphaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal, DMatrixRMaj alphaCoMPrimeLinear)
   {
      alphaCoMPrimeLinear.set(0, 0, 1);
      alphaCoMPrimeLinear.set(0, 1, time);

      alphaCoMPrimeLinear.set(1, 2, 1);
      alphaCoMPrimeLinear.set(1, 3, time);

      alphaCoMPrimeLinear.set(2, 4, 1);
      alphaCoMPrimeLinear.set(2, 5, time);
   }

   public static void calculateDAlphaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal,
                                                            DMatrixRMaj alphaCoMPrimeLinear)
   {
      alphaCoMPrimeLinear.set(0, 0, 0);
      alphaCoMPrimeLinear.set(0, 1, 1);

      alphaCoMPrimeLinear.set(1, 2, 0);
      alphaCoMPrimeLinear.set(1, 3, 1);

      alphaCoMPrimeLinear.set(2, 4, 0);
      alphaCoMPrimeLinear.set(2, 5, 1);
   }

   public static void calculateBetaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal, DMatrixRMaj betaCoMPrimeLinear)
   {
      betaCoMPrimeLinear.set(0, 0, Math.exp(omega0 * (timeInitial - time)) * 1);
      betaCoMPrimeLinear.set(0, 1, Math.exp(omega0 * (timeInitial - time)) * timeInitial);

      betaCoMPrimeLinear.set(1, 2, Math.exp(omega0 * (timeInitial - time)) * 1);
      betaCoMPrimeLinear.set(1, 3, Math.exp(omega0 * (timeInitial - time)) * timeInitial);

      betaCoMPrimeLinear.set(2, 4, Math.exp(omega0 * (timeInitial - time)) * 1);
      betaCoMPrimeLinear.set(2, 5, Math.exp(omega0 * (timeInitial - time)) * timeInitial);
   }

   public static void calculateDBetaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal, DMatrixRMaj betaCoMPrimeLinear)
   {
      betaCoMPrimeLinear.set(0, 0, -omega0 * Math.exp(omega0 * (timeInitial - time)) * 1);
      betaCoMPrimeLinear.set(0, 1, -omega0 * Math.exp(omega0 * (timeInitial - time)) * timeInitial);

      betaCoMPrimeLinear.set(1, 2, -omega0 * Math.exp(omega0 * (timeInitial - time)) * 1);
      betaCoMPrimeLinear.set(1, 3, -omega0 * Math.exp(omega0 * (timeInitial - time)) * timeInitial);

      betaCoMPrimeLinear.set(2, 4, -omega0 * Math.exp(omega0 * (timeInitial - time)) * 1);
      betaCoMPrimeLinear.set(2, 5, -omega0 * Math.exp(omega0 * (timeInitial - time)) * timeInitial);
   }

   public static void calculateGammaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal, DMatrixRMaj gammaCoMPrimeLinear)
   {
      gammaCoMPrimeLinear.set(0, 0, Math.exp(omega0 * (timeInitial - time)));
   }

   public static void calculateDGammaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal,
                                                            DMatrixRMaj gammaCoMPrimeLinear)
   {
      gammaCoMPrimeLinear.set(0, 0, -omega0 * Math.exp(omega0 * (timeInitial - time)));
   }

   public static double calculateDeltaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal)
   {
      return 0.5 * Math.exp(omega0 * (timeInitial - timeTotal)) * (Math.exp(omega0 * (time - timeInitial)) - Math.exp(omega0 * (timeInitial - time)));
   }

   public static double calculateDDeltaCoMPrime3DByHandLinear(double omega0, double time, double timeInitial, double timeTotal)
   {
      return 0.5 * Math.exp(omega0 * (timeInitial - timeTotal))
            * (omega0 * Math.exp(omega0 * (time - timeInitial)) + omega0 * Math.exp(omega0 * (timeInitial - time)));
   }
}
