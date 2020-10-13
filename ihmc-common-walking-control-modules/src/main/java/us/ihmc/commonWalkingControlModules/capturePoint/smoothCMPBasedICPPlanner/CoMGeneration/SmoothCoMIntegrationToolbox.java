package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoMGeneration;

import java.util.List;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.DenseMatrixVector3D;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.SmoothCapturePointToolbox;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory;
import us.ihmc.robotics.math.trajectories.Trajectory3D;

public class SmoothCoMIntegrationToolbox
{
   private static final int defaultSize = 100;

   private final DenseMatrixVector3D polynomialCoefficients = new DenseMatrixVector3D(1, 16);
   private final DenseMatrixVector3D generalizedAlphaPrimeTerminal = new DenseMatrixVector3D(1, 16);
   private final DenseMatrixVector3D generalizedAlphaBetaCoMPrime = new DenseMatrixVector3D(1, 16);

   private final DenseMatrixVector3D alphaBetaCoMPrime = new DenseMatrixVector3D(1, 16);
   private final DenseMatrixVector3D alphaBetaCoMSecond = new DenseMatrixVector3D(1, 16);
   private final DenseMatrixVector3D alphaBetaCoMThird = new DenseMatrixVector3D(1, 16);

   private final DMatrixRMaj polynomialCoefficientCombinedVector = new DMatrixRMaj(3, defaultSize);

   private final DMatrixRMaj generalizedAlphaCoMPrimeMatrix = new DMatrixRMaj(3, defaultSize);
   private final DMatrixRMaj generalizedBetaCoMPrimeMatrix = new DMatrixRMaj(3, defaultSize);
   private final DMatrixRMaj generalizedAlphaPrimeTerminalMatrix = new DMatrixRMaj(3, defaultSize);
   private final DMatrixRMaj generalizedAlphaBetaCoMPrimeMatrix = new DMatrixRMaj(3, defaultSize);

   private final DMatrixRMaj M1 = new DMatrixRMaj(3, defaultSize);
   private final DMatrixRMaj M2 = new DMatrixRMaj(3, defaultSize);
   private final DMatrixRMaj M3 = new DMatrixRMaj(3, defaultSize);

   public void computeDesiredCenterOfMassCornerData(List<? extends FramePoint3DReadOnly> exitICPCornerPoints,
                                                    List<? extends FixedFramePoint3DBasics> entryCoMCornerPointsToPack,
                                                    List<? extends FixedFramePoint3DBasics> exitCoMCornerPointsToPack,
                                                    List<? extends FixedFrameVector3DBasics> entryCoMCornerVelocitiesToPack,
                                                    List<? extends FixedFrameVector3DBasics> exitCoMCornerVelocitiesToPack,
                                                    List<? extends FixedFrameVector3DBasics> entryCoMCornerAccelerationsToPack,
                                                    List<? extends FixedFrameVector3DBasics> exitCoMCornerAccelerationsToPack,
                                                    List<FrameTrajectory3D> cmpPolynomials3D, FramePoint3DReadOnly initialCenterOfMassPosition,
                                                    FrameVector3DReadOnly initialCenterOfMassVelocity, FrameVector3DReadOnly initialCenterOfMassAcceleration,
                                                    double omega0)
   {
      FrameTrajectory3D cmpPolynomial3D;
      FramePoint3DReadOnly previousExitCoMCornerPoint = initialCenterOfMassPosition;
      FrameVector3DReadOnly previousExitCoMCornerVelocity = initialCenterOfMassVelocity;
      FrameVector3DReadOnly previousExitCoMCornerAcceleration = initialCenterOfMassAcceleration;

      for (int i = 0; i < cmpPolynomials3D.size(); i++)
      {
         cmpPolynomial3D = cmpPolynomials3D.get(i);

         FramePoint3DReadOnly exitICPCornerPoint = exitICPCornerPoints.get(i);

         FixedFramePoint3DBasics entryCoMCornerPoint = entryCoMCornerPointsToPack.get(i);
         FixedFramePoint3DBasics exitCoMCornerPoint = exitCoMCornerPointsToPack.get(i);
         FixedFrameVector3DBasics entryCoMCornerVelocity = entryCoMCornerVelocitiesToPack.get(i);
         FixedFrameVector3DBasics exitCoMCornerVelocity = exitCoMCornerVelocitiesToPack.get(i);
         FixedFrameVector3DBasics entryCoMCornerAcceleration = entryCoMCornerAccelerationsToPack.get(i);
         FixedFrameVector3DBasics exitCoMCornerAcceleration = exitCoMCornerAccelerationsToPack.get(i);

         entryCoMCornerPoint.set(previousExitCoMCornerPoint);
         entryCoMCornerVelocity.set(previousExitCoMCornerVelocity);
         entryCoMCornerAcceleration.set(previousExitCoMCornerAcceleration);

         computeDesiredCenterOfMassPositionVelocityAcceleration(omega0, cmpPolynomial3D.getFinalTime(), exitICPCornerPoint, entryCoMCornerPoint,
                                                                cmpPolynomial3D, exitCoMCornerPoint, exitCoMCornerVelocity, exitCoMCornerAcceleration);

         previousExitCoMCornerPoint = exitCoMCornerPoint;
         previousExitCoMCornerVelocity = exitCoMCornerVelocity;
         previousExitCoMCornerAcceleration = exitCoMCornerAcceleration;
      }
   }

   //TODO: implement validity checks
   public void computeDesiredCenterOfMassCornerPoints(List<? extends FramePoint3DReadOnly> exitICPCornerPoints,
                                                      List<? extends FixedFramePoint3DBasics> entryCoMCornerPointsToPack,
                                                      List<? extends FixedFramePoint3DBasics> exitCoMCornerPointsToPack,
                                                      List<FrameTrajectory3D> cmpPolynomials3D, FramePoint3DReadOnly initialCenterOfMass, double omega0)
   {
      FrameTrajectory3D cmpPolynomial3D;
      FramePoint3DReadOnly previousExitCoMCornerPoint = initialCenterOfMass;

      for (int i = 0; i < cmpPolynomials3D.size(); i++)
      {
         cmpPolynomial3D = cmpPolynomials3D.get(i);

         FramePoint3DReadOnly exitCornerPoint = exitICPCornerPoints.get(i);

         FixedFramePoint3DBasics entryCoMCornerPoint = entryCoMCornerPointsToPack.get(i);
         FixedFramePoint3DBasics exitCoMCornerPoint = exitCoMCornerPointsToPack.get(i);

         entryCoMCornerPoint.set(previousExitCoMCornerPoint);

         computeDesiredCenterOfMassPosition(omega0, cmpPolynomial3D.getFinalTime(), exitCornerPoint, entryCoMCornerPoint, cmpPolynomial3D, exitCoMCornerPoint);
         previousExitCoMCornerPoint = exitCoMCornerPoint;
      }
   }

   public void computeDesiredCenterOfMassCornerVelocities(List<? extends FramePoint3DReadOnly> exitICPCornerPoints,
                                                          List<? extends FramePoint3DReadOnly> entryCoMCornerPoints,
                                                          List<? extends FixedFrameVector3DBasics> entryCoMCornerVelocitiesToPack,
                                                          List<? extends FixedFrameVector3DBasics> exitCoMCornerVelocitiesToPack,
                                                          List<FrameTrajectory3D> cmpPolynomials3D, FrameVector3DReadOnly initialCenterOfMassVelocity,
                                                          double omega0)
   {
      FrameTrajectory3D cmpPolynomial3D;
      FrameVector3DReadOnly previousExitCoMCornerVelocity = initialCenterOfMassVelocity;

      for (int i = 0; i < cmpPolynomials3D.size(); i++)
      {
         cmpPolynomial3D = cmpPolynomials3D.get(i);

         FramePoint3DReadOnly exitICPCornerPoint = exitICPCornerPoints.get(i);
         FramePoint3DReadOnly entryCoMCornerPoint = entryCoMCornerPoints.get(i);

         FixedFrameVector3DBasics entryCoMCornerVelocity = entryCoMCornerVelocitiesToPack.get(i);
         FixedFrameVector3DBasics exitCoMCornerVelocity = exitCoMCornerVelocitiesToPack.get(i);
         entryCoMCornerVelocity.set(previousExitCoMCornerVelocity);

         computeDesiredCenterOfMassVelocity(omega0, cmpPolynomial3D.getFinalTime(), exitICPCornerPoint, entryCoMCornerPoint, cmpPolynomial3D,
                                            exitCoMCornerVelocity);
         previousExitCoMCornerVelocity = exitCoMCornerVelocity;
      }
   }

   public void computeDesiredCenterOfMassCornerAccelerations(List<? extends FramePoint3DReadOnly> exitICPCornerPoints,
                                                             List<? extends FramePoint3DReadOnly> entryCoMCornerPoints,
                                                             List<? extends FixedFrameVector3DBasics> entryCoMCornerAccelerationsToPack,
                                                             List<? extends FixedFrameVector3DBasics> exitCoMCornerAccelerationsToPack,
                                                             List<FrameTrajectory3D> cmpPolynomials3D, FrameVector3DReadOnly initialCenterOfMassAcceleration,
                                                             double omega0)
   {
      FrameTrajectory3D cmpPolynomial3D;
      FrameVector3DReadOnly previousExitCoMCornerAcceleration = initialCenterOfMassAcceleration;

      for (int i = 0; i < cmpPolynomials3D.size(); i++)
      {
         cmpPolynomial3D = cmpPolynomials3D.get(i);

         FramePoint3DReadOnly exitICPCornerPoint = exitICPCornerPoints.get(i);
         FramePoint3DReadOnly entryCoMCornerPoint = entryCoMCornerPoints.get(i);

         FixedFrameVector3DBasics entryCoMCornerAcceleration = entryCoMCornerAccelerationsToPack.get(i);
         FixedFrameVector3DBasics exitCoMCornerAcceleration = exitCoMCornerAccelerationsToPack.get(i);
         entryCoMCornerAcceleration.set(previousExitCoMCornerAcceleration);

         computeDesiredCenterOfMassAcceleration(omega0, cmpPolynomial3D.getFinalTime(), exitICPCornerPoint, entryCoMCornerPoint, cmpPolynomial3D,
                                                exitCoMCornerAcceleration);
         previousExitCoMCornerAcceleration = exitCoMCornerAcceleration;
      }
   }

   public void computeDesiredCenterOfMassPosition(double omega0, double time, FramePoint3DReadOnly finalCapturePoint, FramePoint3DReadOnly initialCenterOfMass,
                                                  FrameTrajectory3D cmpPolynomial3D, FixedFramePoint3DBasics desiredCenterOfMassPositionToPack)
   {
      calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, cmpPolynomial3D, finalCapturePoint, initialCenterOfMass,
                                                           desiredCenterOfMassPositionToPack);
   }

   public void computeDesiredCenterOfMassVelocity(double omega0, double time, FramePoint3DReadOnly finalCapturePoint, FramePoint3DReadOnly initialCenterOfMass,
                                                  FrameTrajectory3D cmpPolynomial3D, FixedFrameVector3DBasics desiredCenterOfMassVelocityToPack)
   {
      calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 1, cmpPolynomial3D, finalCapturePoint, initialCenterOfMass,
                                                           desiredCenterOfMassVelocityToPack);
   }

   public void computeDesiredCenterOfMassAcceleration(double omega0, double time, FramePoint3DReadOnly finalCapturePoint,
                                                      FramePoint3DReadOnly initialCenterOfMass, FrameTrajectory3D cmpPolynomial3D,
                                                      FixedFrameVector3DBasics desiredCenterOfMassAccelerationToPack)
   {
      calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 2, cmpPolynomial3D, finalCapturePoint, initialCenterOfMass,
                                                           desiredCenterOfMassAccelerationToPack);
   }

   public void computeDesiredCenterOfMassPositionVelocityAcceleration(double omega0, double time, FramePoint3DReadOnly icpPositionDesiredFinal,
                                                                      FramePoint3DReadOnly comPositionDesiredInitial, FrameTrajectory3D cmpPolynomial3D,
                                                                      FixedFramePoint3DBasics desiredCenterOfMassPositionToPack,
                                                                      FixedFrameVector3DBasics desiredCenterOfMassVelocityToPack,
                                                                      FixedFrameVector3DBasics desiredCenterOfMassAccelerationToPack)
   {
      icpPositionDesiredFinal.checkReferenceFrameMatch(comPositionDesiredInitial);
      icpPositionDesiredFinal.checkReferenceFrameMatch(cmpPolynomial3D);
      icpPositionDesiredFinal.checkReferenceFrameMatch(desiredCenterOfMassPositionToPack);
      icpPositionDesiredFinal.checkReferenceFrameMatch(desiredCenterOfMassVelocityToPack);
      icpPositionDesiredFinal.checkReferenceFrameMatch(desiredCenterOfMassAccelerationToPack);

      int numberOfCoefficients = cmpPolynomial3D.getNumberOfCoefficients();
      if (numberOfCoefficients < 0)
      {
         desiredCenterOfMassPositionToPack.setToNaN();
         desiredCenterOfMassVelocityToPack.setToNaN();
         desiredCenterOfMassAccelerationToPack.setToNaN();
         return;
      }

      setPolynomialCoefficientVector3D(polynomialCoefficients, cmpPolynomial3D);

      calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(omega0, time, alphaBetaCoMPrime, alphaBetaCoMSecond, alphaBetaCoMThird, cmpPolynomial3D);

      double expOmega0Time = Math.exp(omega0 * (cmpPolynomial3D.getInitialTime() - time));
      // Optimized implementation of calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(omega0, time, gammaCoMDerivativeOrder, cmpPolynomial3D) with gammaCoMDerivativeOrder = 0, 1, and 2
      double gammaCoMPrime = expOmega0Time;
      double gammaCoMSecond = -omega0 * expOmega0Time;
      double gammaCoMThird = -omega0 * gammaCoMSecond;

      double expOmega0Duration = Math.exp(omega0 * (cmpPolynomial3D.getInitialTime() - cmpPolynomial3D.getFinalTime()));
      // Optimized implementation of calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(expOmega0Duration, expOmega0Time, deltaCoMDerivativeOrder, cmpPolynomial3D) with deltaCoMDerivativeOrder = 0, 1, and 2
      double deltaCoMPrime = 0.5 * expOmega0Duration * (1.0 / expOmega0Time - expOmega0Time);
      double deltaCoMSecond = 0.5 * expOmega0Duration * omega0 * (1.0 / expOmega0Time + expOmega0Time);
      double deltaCoMThird = deltaCoMPrime * omega0 * omega0; // == 0.5 * expOmega0Duration * omega0 * omega0 * (1.0 / expOmega0Time - expOmega0Time);

      double timeSegmentTotal = cmpPolynomial3D.getFinalTime();
      SmoothCapturePointToolbox.calculateGeneralizedAlphaPrimeOnCMPSegment3D(omega0, timeSegmentTotal, generalizedAlphaPrimeTerminal, 0, cmpPolynomial3D);

      calculateCoMQuantity3D(alphaBetaCoMPrime, gammaCoMPrime, deltaCoMPrime, generalizedAlphaPrimeTerminal, polynomialCoefficients, icpPositionDesiredFinal,
                             comPositionDesiredInitial, desiredCenterOfMassPositionToPack);
      calculateCoMQuantity3D(alphaBetaCoMSecond, gammaCoMSecond, deltaCoMSecond, generalizedAlphaPrimeTerminal, polynomialCoefficients, icpPositionDesiredFinal,
                             comPositionDesiredInitial, desiredCenterOfMassVelocityToPack);
      calculateCoMQuantity3D(alphaBetaCoMThird, gammaCoMThird, deltaCoMThird, generalizedAlphaPrimeTerminal, polynomialCoefficients, icpPositionDesiredFinal,
                             comPositionDesiredInitial, desiredCenterOfMassAccelerationToPack);
   }

   public void calculateCoMQuantityFromCorrespondingCMPPolynomial3D(double omega0, double time, int comDerivativeOrder, FrameTrajectory3D cmpPolynomial3D,
                                                                    FrameTuple3DReadOnly icpPositionDesiredFinal,
                                                                    FrameTuple3DReadOnly comPositionDesiredInitial, FixedFrameTuple3DBasics comQuantityDesired)
   {
      int numberOfCoefficients = cmpPolynomial3D.getNumberOfCoefficients();
      if (numberOfCoefficients < 0)
      {
         comQuantityDesired.setToNaN();
         return;
      }

      initializeMatrices3D(numberOfCoefficients);
      setPolynomialCoefficientVector3D(polynomialCoefficients, cmpPolynomial3D);

      calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(omega0, time, generalizedAlphaBetaCoMPrime, comDerivativeOrder, cmpPolynomial3D);
      double generalizedGammaCoMPrime = calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(omega0, time, comDerivativeOrder, cmpPolynomial3D);
      double generalizedDeltaCoMPrime = calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(omega0, time, comDerivativeOrder, cmpPolynomial3D);

      double timeSegmentTotal = cmpPolynomial3D.getFinalTime();
      SmoothCapturePointToolbox.calculateGeneralizedAlphaPrimeOnCMPSegment3D(omega0, timeSegmentTotal, generalizedAlphaPrimeTerminal, 0, cmpPolynomial3D);

      calculateCoMQuantity3D(generalizedAlphaBetaCoMPrime, generalizedGammaCoMPrime, generalizedDeltaCoMPrime, generalizedAlphaPrimeTerminal,
                             polynomialCoefficients, icpPositionDesiredFinal, comPositionDesiredInitial, comQuantityDesired);
   }

   /**
    * Compute the i-th derivative of x<sub>ref,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * x<sup>(i)</sup><sub>ref,&phi;</sub>(t<sub>&phi;</sub>) =
    * (&alpha;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) -
    * &beta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)) * p<sub>&phi;</sub> +
    * &gamma;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) *
    * x<sub>ref,&phi;</sub>(t<sub>0,&phi;</sub>) +
    * &delta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) *
    * (&xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>) -
    * &alpha;<sup>(0)</sup><sub>ICP,&phi;</sub>(T<sub>&phi;</sub>) * p<sub>&phi;</sub>)
    * 
    * @param generalizedAlphaBetaCoMPrimeMatrix =
    *        &alpha;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) -
    *           &beta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)
    * @param generalizedGammaCoMPrime = &gamma;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)
    * @param generalizedDeltaCoMPrime = &delta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)
    * @param generalizedAlphaPrimeTerminalMatrix =
    *        &alpha;<sup>(0)</sup><sub>ICP,&phi;</sub>(T<sub>&phi;</sub>)
    * @param polynomialCoefficientCombinedVector = p<sub>&phi;</sub>
    * @param icpPositionDesiredFinal = &xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>)
    * @param comPositionDesiredInitial = x<sub>ref,&phi;</sub>(t<sub>0,&phi;</sub>)
    * @param comQuantityDesiredToPack = x<sup>(i)</sup><sub>ref,&phi;</sub>(t<sub>&phi;</sub>)
    */
   // FIXME this can probably be more efficient
   public void calculateCoMQuantity3D(DMatrixRMaj generalizedAlphaBetaCoMPrimeMatrix, double generalizedGammaCoMPrime, double generalizedDeltaCoMPrime,
                                      DMatrixRMaj generalizedAlphaPrimeTerminalMatrix, DMatrixRMaj polynomialCoefficientCombinedVector,
                                      FrameTuple3DReadOnly icpPositionDesiredFinal, FrameTuple3DReadOnly comPositionDesiredInitial,
                                      FixedFrameTuple3DBasics comQuantityDesiredToPack)
   {
      int numRows = generalizedAlphaBetaCoMPrimeMatrix.getNumRows();
      M1.reshape(numRows, 1);
      M2.reshape(numRows, 1);
      M3.reshape(numRows, 1);

      CommonOps_DDRM.mult(generalizedAlphaBetaCoMPrimeMatrix, polynomialCoefficientCombinedVector, M1);

      M2.set(0, generalizedGammaCoMPrime * comPositionDesiredInitial.getX());
      M2.set(1, generalizedGammaCoMPrime * comPositionDesiredInitial.getY());
      M2.set(2, generalizedGammaCoMPrime * comPositionDesiredInitial.getZ());

      CommonOps_DDRM.mult(generalizedAlphaPrimeTerminalMatrix, polynomialCoefficientCombinedVector, M3);

      M3.set(0, icpPositionDesiredFinal.getX() - M3.get(0));
      M3.set(1, icpPositionDesiredFinal.getY() - M3.get(1));
      M3.set(2, icpPositionDesiredFinal.getZ() - M3.get(2));

      CommonOps_DDRM.addEquals(M1, M2);
      CommonOps_DDRM.addEquals(M1, generalizedDeltaCoMPrime, M3);

      comQuantityDesiredToPack.set(M1.get(0), M1.get(1), M1.get(2));
   }

   /**
    * Compute the i-th derivative of x<sub>ref,&phi;</sub> at time t<sub>&phi;</sub>:
    * 
    * <pre>
    * x<sup>(i)</sup><sub>ref,&phi;</sub>(t<sub>&phi;</sub>) = 
    *       (&alpha;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) - &beta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)) * p<sub>&phi;</sub>
    *     + &gamma;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) * x<sub>ref,&phi;</sub>(t<sub>0,&phi;</sub>)
    *     + &delta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) * (&xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>) - &alpha;<sup>(0)</sup><sub>ICP,&phi;</sub>(T<sub>&phi;</sub>) * p<sub>&phi;</sub>)
    * </pre>
    * 
    * @param generalizedAlphaBetaCoMPrime =
    *        &alpha;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) -
    *           &beta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)
    * @param generalizedGammaCoMPrime = &gamma;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)
    * @param generalizedDeltaCoMPrime = &delta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)
    * @param generalizedAlphaPrimeTerminal =
    *        &alpha;<sup>(0)</sup><sub>ICP,&phi;</sub>(T<sub>&phi;</sub>)
    * @param polynomialCoefficientCombined = p<sub>&phi;</sub>
    * @param icpPositionDesiredFinal = &xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>)
    * @param comPositionDesiredInitial = x<sub>ref,&phi;</sub>(t<sub>0,&phi;</sub>)
    * @param comQuantityDesiredToPack = x<sup>(i)</sup><sub>ref,&phi;</sub>(t<sub>&phi;</sub>)
    */
   public void calculateCoMQuantity3D(DenseMatrixVector3D generalizedAlphaBetaCoMPrime, double generalizedGammaCoMPrime, double generalizedDeltaCoMPrime,
                                      DenseMatrixVector3D generalizedAlphaPrimeTerminal, DenseMatrixVector3D polynomialCoefficientCombined,
                                      Tuple3DReadOnly icpPositionDesiredFinal, Tuple3DReadOnly comPositionDesiredInitial,
                                      Tuple3DBasics comQuantityDesiredToPack)
   {
      for (Axis3D axis : Axis3D.values)
      {
         DMatrixRMaj alphaBetaCoM = generalizedAlphaBetaCoMPrime.getMatrix(axis);
         DMatrixRMaj alphaICP = generalizedAlphaPrimeTerminal.getMatrix(axis);
         DMatrixRMaj p = polynomialCoefficientCombined.getMatrix(axis);

         double comQuantityDesired = 0.0;

         // (alpha^(i)_(CoM,phi) (t_phi) - beta^(i)_(CoM,phi) (t_phi)) * p_phi
         for (int i = 0; i < alphaBetaCoM.numCols; i++)
            comQuantityDesired += alphaBetaCoM.get(i) * p.get(i);

         // (alpha^(i)_(CoM,phi) (t_phi) - beta^(i)_(CoM,phi) (t_phi)) * p_phi
         //    + gamma^(i)_(CoM,phi) (t_phi) * x_(ref,phi) (t_phi)
         comQuantityDesired += generalizedGammaCoMPrime * comPositionDesiredInitial.getElement(axis.ordinal());

         double alphaICP_p = 0.0;

         // alpha^(0)_(ICP,phi) (T_phi) * p_phi
         for (int i = 0; i < alphaICP.numCols; i++)
            alphaICP_p += alphaICP.get(i) * p.get(i);

         // (alpha^(i)_(CoM,phi) (t_phi) - beta^(i)_(CoM,phi) (t_phi)) * p_phi
         //    + gamma^(i)_(CoM,phi) (t_phi) * x_(ref,phi) (t_phi)
         //    + delta^(i)_(CoM,phi) (t_phi) * (xi_(CoM,phi) (T_phi) - alpha^(0)_(ICP,phi) (T_phi) * p_phi)
         comQuantityDesired += generalizedDeltaCoMPrime * (icpPositionDesiredFinal.getElement(axis.ordinal()) - alphaICP_p);

         comQuantityDesiredToPack.setElement(axis.ordinal(), comQuantityDesired);
      }
   }

   // FIXME this can probably be more efficient
   public double calculateCoMQuantity1D(DMatrixRMaj generalizedAlphaBetaCoMPrimeMatrix, double generalizedGammaCoMPrime, double generalizedDeltaCoMPrime,
                                        DMatrixRMaj generalizedAlphaPrimeTerminalMatrix, DMatrixRMaj polynomialCoefficientVector,
                                        double icpPositionDesiredFinal, double comPositionDesiredInitial)
   {
      int numRows = generalizedAlphaBetaCoMPrimeMatrix.numRows;
      M1.reshape(numRows, 1);
      M2.reshape(numRows, 1);
      M3.reshape(numRows, 1);

      CommonOps_DDRM.mult(generalizedAlphaBetaCoMPrimeMatrix, polynomialCoefficientVector, M1);

      M2.set(0, generalizedGammaCoMPrime * comPositionDesiredInitial);
      CommonOps_DDRM.mult(generalizedAlphaPrimeTerminalMatrix, polynomialCoefficientVector, M3);

      M3.set(0, icpPositionDesiredFinal - M3.get(0));

      CommonOps_DDRM.addEquals(M1, M2);
      CommonOps_DDRM.addEquals(M1, generalizedDeltaCoMPrime, M3);

      return M1.get(0);
   }

   /**
    * Compute the i-th derivative of (&alpha;<sub>CoM,&phi;</sub> - &beta;<sub>CoM,&phi;</sub>) at
    * time t<sub>&phi;</sub>.
    * 
    * @param omega0 the natural frequency of the walking system.
    * @param time the time t<sub>&phi;</sub>
    * @param generalizedAlphaBetaCoMPrime matrix used to store the result. Modified.
    * @param alphaBetaCoMDerivativeOrder the order of the derivative.
    * @param cmpPolynomial3D the polynomial to compute &alpha;<sub>CoM,&phi;</sub> and
    *           &beta;<sub>CoM,&phi;</sub> from. Not modified.
    */
   public static void calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(double omega0, double time, DMatrixRMaj generalizedAlphaBetaCoMPrime,
                                                                          int alphaBetaCoMDerivativeOrder, FrameTrajectory3D cmpPolynomial3D)
   {
      int numberOfCoefficients = cmpPolynomial3D.getNumberOfCoefficients();
      double timeSegmentInitial = cmpPolynomial3D.getInitialTime();

      generalizedAlphaBetaCoMPrime.zero();

      double signAlpha = 1.0;
      double signBeta = alphaBetaCoMDerivativeOrder % 2 == 0 ? 1.0 : -1.0;
      double omega0Inv = 1.0 / omega0;
      double omega0PowerInitialAlpha = 1.0;
      double omega0PowerBeta = power(omega0, alphaBetaCoMDerivativeOrder);
      double expOmega0Time = Math.exp(omega0 * (timeSegmentInitial - time));

      for (int i = 0; i < numberOfCoefficients; i++)
      {
         double omega0PowerAlpha = omega0PowerInitialAlpha;

         for (int j = i; j < numberOfCoefficients; j++)
         {
            double scalarAlpha = signAlpha * omega0PowerAlpha; // sign * omega0^(-j)
            double scalarBeta = signBeta * omega0PowerBeta * omega0PowerAlpha * expOmega0Time; // == sign * omega^(betaCoMDerivativeOrder - j) * exp(omega0 * (timeSegmentInitial - time))

            for (Axis3D dir : Axis3D.values)
            {
               Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);
               DMatrixRMaj geometricSequenceDerivative = cmpPolynomial.evaluateGeometricSequenceDerivative(j + alphaBetaCoMDerivativeOrder, time);

               MatrixTools.addMatrixBlock(generalizedAlphaBetaCoMPrime, dir.ordinal(), dir.ordinal() * geometricSequenceDerivative.numCols,
                                          geometricSequenceDerivative, 0, 0, geometricSequenceDerivative.numRows, geometricSequenceDerivative.numCols,
                                          scalarAlpha);

               geometricSequenceDerivative = cmpPolynomial.evaluateGeometricSequenceDerivative(j, timeSegmentInitial);

               MatrixTools.addMatrixBlock(generalizedAlphaBetaCoMPrime, dir.ordinal(), dir.ordinal() * geometricSequenceDerivative.numCols,
                                          geometricSequenceDerivative, 0, 0, geometricSequenceDerivative.numRows, geometricSequenceDerivative.numCols,
                                          -scalarBeta);
            }

            omega0PowerAlpha *= omega0Inv;
         }

         signAlpha = -signAlpha;
         signBeta = -signBeta;
         omega0PowerInitialAlpha *= omega0Inv;
      }
   }

   /**
    * Compute the i-th derivative of (&alpha;<sub>CoM,&phi;</sub> - &beta;<sub>CoM,&phi;</sub>) at
    * time t<sub>&phi;</sub>:
    * 
    * <pre>
    * &alpha;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub> k=0</sub><sup>n</sup> &Sigma;<sub>j=k</sub><sup>n</sup> (-1)<sup>k</sup> * &omega;<sub>0</sub><sup>-j</sup> * t<sup>(j+i)<sup>T</sup></sup> (t<sub>&phi;</sub>)
    * 
    * &beta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>k=0</sub><sup>n</sup> &Sigma;<sub>j=k</sub><sup>n</sup> (-1)<sup>k+i</sup> * &omega;<sub>0</sub><sup>-j+i</sup> * t<sup>(j)<sup>T</sup></sup> (T<sub>&phi;</sub>) * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    * </pre>
    * 
    * @param omega0 the natural frequency of the walking system.
    * @param time the time t<sub>&phi;</sub>
    * @param generalizedAlphaBetaCoMPrime matrix used to store the result. Modified.
    * @param alphaBetaCoMDerivativeOrder the order of the derivative.
    * @param cmpPolynomial3D the polynomial to compute &alpha;<sub>CoM,&phi;</sub> and
    *           &beta;<sub>CoM,&phi;</sub> from. Not modified.
    */
   public static void calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(double omega0, double time, DenseMatrixVector3D generalizedAlphaBetaCoMPrime,
                                                                          int alphaBetaCoMDerivativeOrder, FrameTrajectory3D cmpPolynomial3D)
   {
      int numberOfCoefficients = cmpPolynomial3D.getNumberOfCoefficients();
      double timeSegmentInitial = cmpPolynomial3D.getInitialTime();

      for (Axis3D dir : Axis3D.values)
      {
         generalizedAlphaBetaCoMPrime.getMatrix(dir).reshape(1, cmpPolynomial3D.getNumberOfCoefficients(dir));
      }

      generalizedAlphaBetaCoMPrime.zero();

      double signAlpha = 1.0;
      double signBeta = alphaBetaCoMDerivativeOrder % 2 == 0 ? 1.0 : -1.0;
      double omega0Inv = 1.0 / omega0;
      double omega0PowerInitialAlpha = 1.0;
      double omega0PowerBeta = power(omega0, alphaBetaCoMDerivativeOrder);
      double expOmega0Time = Math.exp(omega0 * (timeSegmentInitial - time));

      for (int i = 0; i < numberOfCoefficients; i++)
      {
         double omega0PowerAlpha = omega0PowerInitialAlpha;

         for (int j = i; j < numberOfCoefficients; j++)
         {
            double scalarAlpha = signAlpha * omega0PowerAlpha; // sign * omega0^(-j)
            double scalarBeta = signBeta * omega0PowerBeta * omega0PowerAlpha * expOmega0Time; // == sign * omega^(betaCoMDerivativeOrder - j) * exp(omega0 * (timeSegmentInitial - time))

            for (Axis3D dir : Axis3D.values)
            {
               Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);
               DMatrixRMaj matrix = generalizedAlphaBetaCoMPrime.getMatrix(dir);

               DMatrixRMaj geometricSequenceDerivative = cmpPolynomial.evaluateGeometricSequenceDerivative(j + alphaBetaCoMDerivativeOrder, time);
               CommonOps_DDRM.addEquals(matrix, scalarAlpha, geometricSequenceDerivative);
               geometricSequenceDerivative = cmpPolynomial.evaluateGeometricSequenceDerivative(j, timeSegmentInitial);
               CommonOps_DDRM.addEquals(matrix, -scalarBeta, geometricSequenceDerivative);
            }

            omega0PowerAlpha *= omega0Inv;
         }

         signAlpha = -signAlpha;
         signBeta = -signBeta;
         omega0PowerInitialAlpha *= omega0Inv;
      }
   }

   /**
    * Compute the 0-th, 1-st, and 2-nd derivatives of (&alpha;<sub>CoM,&phi;</sub> -
    * &beta;<sub>CoM,&phi;</sub>) at time t<sub>&phi;</sub>:
    * 
    * <pre>
    * &alpha;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub> k=0</sub><sup>n</sup> &Sigma;<sub>j=k</sub><sup>n</sup> (-1)<sup>k</sup> * &omega;<sub>0</sub><sup>-j</sup> * t<sup>(j+i)<sup>T</sup></sup>(t<sub>&phi;</sub>)
    * 
    * &beta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>k=0</sub><sup>n</sup> &Sigma;<sub>j=k</sub><sup>n</sup> (-1)<sup>k+i</sup> * &omega;<sub>0</sub><sup>-j+i</sup> * t<sup>(j)<sup>T</sup></sup>(T<sub>&phi;</sub>) * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    * </pre>
    * 
    * It is simplified to the following:
    * 
    * <pre>
    * (&alpha;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) - &beta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)) = 
    *       &Sigma;<sub> k=0</sub><sup>n</sup> &Sigma;<sub>j=k</sub><sup>n</sup> (-1)<sup>k</sup> * &omega;<sub>0</sub><sup>-j</sup> <b>[</b> t<sup>(j+i)<sup>T</sup></sup>(t<sub>&phi;</sub>) - (-1)<sup>i</sup> * &omega;<sub>0</sub><sup>i</sup>  * t<sup>(j)<sup>T</sup></sup>(T<sub>&phi;</sub>) * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup> <b>]</b>
    * </pre>
    * 
    * @param omega0 the natural frequency of the walking system.
    * @param time the time t<sub>&phi;</sub>
    * @param alphaBetaCoMPrime matrix used to store the result. Modified.
    * @param alphaBetaCoMDerivativeOrder the order of the derivative.
    * @param cmpPolynomial3D the polynomial to compute &alpha;<sub>CoM,&phi;</sub> and
    *           &beta;<sub>CoM,&phi;</sub> from. Not modified.
    */
   public static void calculateGeneralizedAlphaBetaCoMPrimeOnCMPSegment3D(double omega0, double time, DenseMatrixVector3D alphaBetaCoMPrime,
                                                                          DenseMatrixVector3D alphaBetaCoMSecond, DenseMatrixVector3D alphaBetaCoMThird,
                                                                          Trajectory3D cmpPolynomial3D)
   {
      int numberOfCoefficients = cmpPolynomial3D.getNumberOfCoefficients();
      double timeSegmentInitial = cmpPolynomial3D.getInitialTime();

      for (Axis3D dir : Axis3D.values)
      {
         int numCols = cmpPolynomial3D.getNumberOfCoefficients(dir);
         alphaBetaCoMPrime.getMatrix(dir).reshape(1, numCols);
         alphaBetaCoMSecond.getMatrix(dir).reshape(1, numCols);
         alphaBetaCoMThird.getMatrix(dir).reshape(1, numCols);
      }

      alphaBetaCoMPrime.zero();
      alphaBetaCoMSecond.zero();
      alphaBetaCoMThird.zero();

      double minusOneToK = 1.0; // (-1)^k
      double omega0Inverse = 1.0 / omega0;
      double omega0ToMinusK = 1.0; // omega0^(-k)
      double omega0ToMinusJ = 1.0; // omega0^(-j)

      double expOmega0Time = Math.exp(omega0 * (timeSegmentInitial - time));
      double scaleBetaPrime = -expOmega0Time; // - (-1)^i * omega0^i * exp(t0 - t) for the 0-th derivative
      double scaleBetaSecond = omega0 * expOmega0Time; // - (-1)^i * omega0^i * exp(t0 - t) for the 1-st derivative
      double scaleBetaThird = -omega0 * scaleBetaSecond; // - (-1)^i * omega0^i * exp(t0 - t) for the 2-nd derivative

      DMatrixRMaj geometricSequenceDerivative;

      for (int k = 0; k < numberOfCoefficients; k++)
      {
         omega0ToMinusJ = omega0ToMinusK;

         for (int j = k; j < numberOfCoefficients; j++)
         {
            for (Axis3D dir : Axis3D.values)
            {
               Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);

               DMatrixRMaj alphaBetaCoMPrimeComponent = alphaBetaCoMPrime.getMatrix(dir);
               DMatrixRMaj alphaBetaCoMSecondComponent = alphaBetaCoMSecond.getMatrix(dir);
               DMatrixRMaj alphaBetaCoMThirdComponent = alphaBetaCoMThird.getMatrix(dir);

               geometricSequenceDerivative = cmpPolynomial.evaluateGeometricSequenceDerivative(j, timeSegmentInitial);
               double minusOneToK_times_omega0ToMinusJ = minusOneToK * omega0ToMinusJ;
               CommonOps_DDRM.addEquals(alphaBetaCoMPrimeComponent, scaleBetaPrime * minusOneToK_times_omega0ToMinusJ, geometricSequenceDerivative);
               CommonOps_DDRM.addEquals(alphaBetaCoMSecondComponent, scaleBetaSecond * minusOneToK_times_omega0ToMinusJ, geometricSequenceDerivative);
               CommonOps_DDRM.addEquals(alphaBetaCoMThirdComponent, scaleBetaThird * minusOneToK_times_omega0ToMinusJ, geometricSequenceDerivative);
               CommonOps_DDRM.addEquals(alphaBetaCoMPrimeComponent, minusOneToK_times_omega0ToMinusJ,
                                   cmpPolynomial.evaluateGeometricSequenceDerivative(j + 0, time));
               CommonOps_DDRM.addEquals(alphaBetaCoMSecondComponent, minusOneToK_times_omega0ToMinusJ,
                                   cmpPolynomial.evaluateGeometricSequenceDerivative(j + 1, time));
               CommonOps_DDRM.addEquals(alphaBetaCoMThirdComponent, minusOneToK_times_omega0ToMinusJ,
                                   cmpPolynomial.evaluateGeometricSequenceDerivative(j + 2, time));
            }

            omega0ToMinusJ *= omega0Inverse;
         }

         minusOneToK = -minusOneToK;
         omega0ToMinusK *= omega0Inverse;
      }
   }

   /**
    * Compute the i-th derivative of &alpha;<sub>CoM,&phi;</sub> at time t<sub>&phi;</sub>:
    * 
    * <pre>
    * &alpha;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>k=0</sub><sup>n</sup> &Sigma;<sub>j=k</sub><sup>n</sup> (-1)<sup>k</sup> * &omega;<sub>0</sub><sup>-j</sup> * t<sup>(j+i)<sup>T</sup></sup> (t<sub>&phi;</sub>)
    * </pre>
    */
   public static void calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(double omega0, double time, DMatrixRMaj generalizedAlphaCoMPrime,
                                                                      int alphaCoMDerivativeOrder, FrameTrajectory3D cmpPolynomial3D)
   {
      int numberOfCoefficients = cmpPolynomial3D.getNumberOfCoefficients();
      generalizedAlphaCoMPrime.zero();

      double sign = 1.0;
      double omega0Inv = 1.0 / omega0;
      double omega0PowerInitial = 1.0;

      for (int i = 0; i < numberOfCoefficients; i++)
      {
         double omega0Power = omega0PowerInitial;

         for (int j = i; j < numberOfCoefficients; j++)
         {
            double scalar = sign * omega0Power; // sign * omega0^(-j)

            for (Axis3D dir : Axis3D.values)
            {
               Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);
               DMatrixRMaj geometricSequenceDerivative = cmpPolynomial.evaluateGeometricSequenceDerivative(j + alphaCoMDerivativeOrder, time);

               MatrixTools.addMatrixBlock(generalizedAlphaCoMPrime, dir.ordinal(), dir.ordinal() * geometricSequenceDerivative.numCols,
                                          geometricSequenceDerivative, 0, 0, geometricSequenceDerivative.numRows, geometricSequenceDerivative.numCols, scalar);
            }

            omega0Power *= omega0Inv;
         }

         sign = -sign;
         omega0PowerInitial *= omega0Inv;
      }
   }

   /**
    * Compute the i-th derivative of &alpha;<sub>CoM,&phi;</sub> at time t<sub>&phi;</sub>:
    * 
    * <pre>
    * &alpha;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>k=0</sub><sup>n</sup> &Sigma;<sub>j=k</sub><sup>n</sup> (-1)<sup>k</sup> * &omega;<sub>0</sub><sup>-j</sup> * t<sup>(j+i)<sup>T</sup></sup> (t<sub>&phi;</sub>)
    * </pre>
    */
   public static void calculateGeneralizedAlphaCoMPrimeOnCMPSegment1D(double omega0, double time, DMatrixRMaj generalizedAlphaCoMPrimeRow,
                                                                      int alphaCoMDerivativeOrder, Trajectory cmpPolynomial)
   {
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();

      generalizedAlphaCoMPrimeRow.reshape(1, numberOfCoefficients);
      generalizedAlphaCoMPrimeRow.zero();

      double sign = 1.0;
      double omega0Inv = 1.0 / omega0;
      double omega0PowerInitial = 1.0;

      for (int i = 0; i < numberOfCoefficients; i++)
      {
         double omega0Power = omega0PowerInitial;

         for (int j = i; j < numberOfCoefficients; j++)
         {
            double scalar = sign * omega0Power; // sign * omega0^(-j)
            CommonOps_DDRM.addEquals(generalizedAlphaCoMPrimeRow, scalar, cmpPolynomial.evaluateGeometricSequenceDerivative(j + alphaCoMDerivativeOrder, time));
            omega0Power *= omega0Inv;
         }

         sign = -sign;
         omega0PowerInitial *= omega0Inv;
      }
   }

   /**
    * Compute the i-th derivative of &beta;<sub>CoM,&phi;</sub> at time t<sub>&phi;</sub>:
    * 
    * <pre>
    * &beta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>k=0</sub><sup>n</sup> &Sigma;<sub>j=k</sub><sup>n</sup> (-1)<sup>k+i</sup> * &omega;<sub>0</sub><sup>-j+i</sup> * t<sup>(j)<sup>T</sup></sup> (T<sub>&phi;</sub>) * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    * </pre>
    */
   public static void calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(double omega0, double time, DMatrixRMaj generalizedBetaCoMPrime,
                                                                     int betaCoMDerivativeOrder, FrameTrajectory3D cmpPolynomial3D)
   {
      int numberOfCoefficients = cmpPolynomial3D.getNumberOfCoefficients();
      double timeSegmentInitial = cmpPolynomial3D.getInitialTime();

      generalizedBetaCoMPrime.zero();

      double sign = betaCoMDerivativeOrder % 2 == 0 ? 1.0 : -1.0;
      double omega0Inv = 1.0 / omega0;
      double omega0PowInitial = power(omega0, betaCoMDerivativeOrder);
      double expOmega0Time = Math.exp(omega0 * (timeSegmentInitial - time));

      for (int i = 0; i < numberOfCoefficients; i++)
      {
         double omega0Pow = omega0PowInitial;

         for (int j = i; j < numberOfCoefficients; j++)
         {
            double scalar = sign * omega0Pow * expOmega0Time; // == sign * omega^(betaCoMDerivativeOrder - j) * exp(omega0 * (timeSegmentInitial - time))

            for (Axis3D dir : Axis3D.values)
            {
               Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);

               DMatrixRMaj geometricSequenceDerivative = cmpPolynomial.evaluateGeometricSequenceDerivative(j, timeSegmentInitial);
               MatrixTools.addMatrixBlock(generalizedBetaCoMPrime, dir.ordinal(), dir.ordinal() * geometricSequenceDerivative.numCols,
                                          geometricSequenceDerivative, 0, 0, geometricSequenceDerivative.numRows, geometricSequenceDerivative.numCols, scalar);
            }
            omega0Pow *= omega0Inv;
         }

         sign = -sign;
         omega0PowInitial *= omega0Inv;
      }
   }

   public static void calculateGeneralizedBetaCoMPrimeOnCMPSegment1D(double omega0, double time, DMatrixRMaj generalizedBetaCoMPrimeRow,
                                                                     int betaCoMDerivativeOrder, Trajectory cmpPolynomial)
   {
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      double timeSegmentInitial = cmpPolynomial.getInitialTime();

      generalizedBetaCoMPrimeRow.reshape(1, numberOfCoefficients);
      generalizedBetaCoMPrimeRow.zero();

      double sign = betaCoMDerivativeOrder % 2 == 0 ? 1.0 : -1.0;
      double omega0Inv = 1.0 / omega0;
      double omega0PowInitial = power(omega0, betaCoMDerivativeOrder);
      double expOmega0Time = Math.exp(omega0 * (timeSegmentInitial - time));

      for (int i = 0; i < numberOfCoefficients; i++)
      {
         double omega0Pow = omega0PowInitial;

         for (int j = i; j < numberOfCoefficients; j++)
         {
            double scalar = sign * omega0Pow * expOmega0Time; // == sign * omega^(betaCoMDerivativeOrder - j) * exp(omega0 * (timeSegmentInitial - time))
            DMatrixRMaj xPowersDerivativeVector = cmpPolynomial.evaluateGeometricSequenceDerivative(j, timeSegmentInitial);
            CommonOps_DDRM.addEquals(generalizedBetaCoMPrimeRow, scalar, xPowersDerivativeVector);
            omega0Pow *= omega0Inv;
         }

         sign = -sign;
         omega0PowInitial *= omega0Inv;
      }
   }

   /**
    * Raise the scalar {@code a} to the power of {@code exponent}.
    * <p>
    * Compared to {@link Math#pow(double, double)}, this algorithm is optimized for integer
    * exponents.
    * </p>
    * 
    * @param a the base.
    * @param exponent the exponent.
    * @return the value {@code a}<sup>{@code exponent}</sup>.
    */
   public static double power(double a, int exponent)
   {
      if (exponent < 0)
         return 1.0 / power(a, -exponent);
      if (exponent == 0)
         return 1.0;
      if (exponent == 1)
         return a;

      double halfPow = power(a, exponent / 2);
      if ((exponent & 1) == 0) // exponent is even
         return halfPow * halfPow;
      else
         return a * halfPow * halfPow;
   }

   /**
    * Compute the i-th derivative of &gamma;<sub>CoM,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &gamma;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = (-1)<sup>i</sup> *
    * &omega;<sub>0</sub><sup>i</sup> *
    * e<sup>&omega;<sub>0</sub>(t<sub>0,&phi;</sub>-t<sub>&phi;</sub>)</sup>
    *
    * @return generalizedGammaCoMPrime
    */
   public static double calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(double omega0, double time, int gammaCoMDerivativeOrder,
                                                                        FrameTrajectory3D cmpPolynomial3D)
   {
      double timeSegmentInitial = cmpPolynomial3D.getInitialTime();
      double sign = gammaCoMDerivativeOrder % 2 == 0 ? 1.0 : -1.0;
      return sign * power(omega0, gammaCoMDerivativeOrder) * Math.exp(omega0 * (timeSegmentInitial - time));
   }

   /**
    * Compute the i-th derivative of &delta;<sub>CoM,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &delta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = 0.5 *
    * e<sup>&omega;<sub>0</sub>(t<sub>0,&phi;</sub>-T<sub>&phi;</sub>)</sup> *
    * (&omega;<sub>0</sub><sup>i</sup> *
    * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-t<sub>0,&phi;</sub>)</sup> - (-1)<sup>i</sup> *
    * &omega;<sub>0</sub><sup>i</sup> *
    * e<sup>&omega;<sub>0</sub>(t<sub>0,&phi;</sub>-t<sub>&phi;</sub>)</sup>)
    */
   public static double calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(double omega0, double time, int deltaCoMDerivativeOrder,
                                                                        FrameTrajectory3D cmpPolynomial3D)
   {
      double timeSegmentInitial = cmpPolynomial3D.getInitialTime();
      double timeSegmentTotal = cmpPolynomial3D.getFinalTime();
      double expOmega0Time = Math.exp(omega0 * (time - timeSegmentInitial));
      double powOmega0 = power(omega0, deltaCoMDerivativeOrder);
      double sign = deltaCoMDerivativeOrder % 2 == 0 ? 1.0 : -1.0;

      return 0.5 * Math.exp(omega0 * (timeSegmentInitial - timeSegmentTotal)) * powOmega0 * (expOmega0Time - sign / expOmega0Time);
   }

   private void initializeMatrices3D(int numberOfCoefficients)
   {
      initializeMatrices(3, numberOfCoefficients);
   }

   private void initializeMatrices(int dimension, int numberOfCoefficients)
   {
      polynomialCoefficientCombinedVector.reshape(dimension * numberOfCoefficients, 1);
      polynomialCoefficientCombinedVector.zero();

      generalizedAlphaCoMPrimeMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedAlphaCoMPrimeMatrix.zero();

      generalizedBetaCoMPrimeMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedBetaCoMPrimeMatrix.zero();

      generalizedAlphaBetaCoMPrimeMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedAlphaBetaCoMPrimeMatrix.zero();

      generalizedAlphaPrimeTerminalMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedAlphaPrimeTerminalMatrix.zero();
   }

   private void setPolynomialCoefficientVector3D(DenseMatrixVector3D polynomialCoefficient, FrameTrajectory3D cmpPolynomial3D)
   {
      setPolynomialCoefficientVector1D(polynomialCoefficient.getMatrixX(), cmpPolynomial3D.getTrajectoryX());
      setPolynomialCoefficientVector1D(polynomialCoefficient.getMatrixY(), cmpPolynomial3D.getTrajectoryY());
      setPolynomialCoefficientVector1D(polynomialCoefficient.getMatrixZ(), cmpPolynomial3D.getTrajectoryZ());
   }

   private static void setPolynomialCoefficientVector1D(DMatrixRMaj polynomialCoefficientVector, Trajectory cmpPolynomial)
   {
      double[] polynomialCoefficients = cmpPolynomial.getCoefficients();

      polynomialCoefficientVector.reshape(cmpPolynomial.getNumberOfCoefficients(), 1);
      System.arraycopy(polynomialCoefficients, 0, polynomialCoefficientVector.data, 0, cmpPolynomial.getNumberOfCoefficients());
   }
}
