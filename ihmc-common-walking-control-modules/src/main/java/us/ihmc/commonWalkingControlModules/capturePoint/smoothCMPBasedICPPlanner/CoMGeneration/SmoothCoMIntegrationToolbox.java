package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoMGeneration;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.SmoothCapturePointToolbox;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory;

public class SmoothCoMIntegrationToolbox
{
   private static final int defaultSize = 100;

   private final DenseMatrix64F generalizedAlphaCoMPrimeRow = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedBetaCoMPrimeRow = new DenseMatrix64F(1, defaultSize);

   private final DenseMatrix64F polynomialCoefficientCombinedVector = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F polynomialCoefficientVector = new DenseMatrix64F(defaultSize, 1);

   private final DenseMatrix64F generalizedAlphaCoMPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F generalizedBetaCoMPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F generalizedAlphaPrimeTerminalMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F generalizedAlphaBetaCoMPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);

   private final DenseMatrix64F M1 = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F M2 = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F M3 = new DenseMatrix64F(defaultSize, defaultSize);

   private final SmoothCapturePointToolbox icpToolbox;

   public SmoothCoMIntegrationToolbox(SmoothCapturePointToolbox smoothCapturePointToolbox)
   {
      this.icpToolbox = smoothCapturePointToolbox;
   }

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
      computeDesiredCenterOfMassCornerPoints(exitICPCornerPoints, entryCoMCornerPointsToPack, exitCoMCornerPointsToPack, cmpPolynomials3D,
                                             initialCenterOfMassPosition, omega0);
      computeDesiredCenterOfMassCornerVelocities(exitICPCornerPoints, entryCoMCornerPointsToPack, entryCoMCornerVelocitiesToPack, exitCoMCornerVelocitiesToPack,
                                                 cmpPolynomials3D, initialCenterOfMassVelocity, omega0);
      computeDesiredCenterOfMassCornerAccelerations(exitICPCornerPoints, entryCoMCornerPointsToPack, entryCoMCornerAccelerationsToPack,
                                                    exitCoMCornerAccelerationsToPack, cmpPolynomials3D, initialCenterOfMassAcceleration, omega0);
   }

   //TODO: implement validity checks
   public void computeDesiredCenterOfMassCornerPoints(List<? extends FramePoint3DReadOnly> exitICPCornerPoints,
                                                      List<? extends FixedFramePoint3DBasics> entryCoMCornerPointsToPack,
                                                      List<? extends FixedFramePoint3DBasics> exitCoMCornerPointsToPack,
                                                      List<FrameTrajectory3D> cmpPolynomials3D,
                                                      FramePoint3DReadOnly initialCenterOfMass, double omega0)
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
                                                          List<FrameTrajectory3D> cmpPolynomials3D, FrameVector3DReadOnly initialCenterOfMassVelocity, double omega0)
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

   public void computeDesiredCenterOfMassAcceleration(double omega0, double time, FramePoint3DReadOnly finalCapturePoint, FramePoint3DReadOnly initialCenterOfMass,
                                                      FrameTrajectory3D cmpPolynomial3D, FixedFrameVector3DBasics desiredCenterOfMassAccelerationToPack)
   {
      calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 2, cmpPolynomial3D, finalCapturePoint, initialCenterOfMass,
                                                           desiredCenterOfMassAccelerationToPack);
   }

   public void calculateCoMQuantityFromCorrespondingCMPPolynomial3D(double omega0, double time, int comDerivativeOrder, 
                                                                           FrameTrajectory3D cmpPolynomial3D, 
                                                                           FrameTuple3DReadOnly icpPositionDesiredFinal, 
                                                                           FrameTuple3DReadOnly comPositionDesiredInitial, 
                                                                           FixedFrameTuple3DBasics comQuantityDesired)
   {        
      int numberOfCoefficients = cmpPolynomial3D.getNumberOfCoefficients();
      if (numberOfCoefficients < 0)
      {
         comQuantityDesired.setToNaN();
         return;
      }

      initializeMatrices3D(numberOfCoefficients);
      setPolynomialCoefficientVector3D(polynomialCoefficientCombinedVector, cmpPolynomial3D);

      calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(omega0, time, generalizedAlphaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial3D);
      calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(omega0, time, generalizedBetaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial3D);
      double generalizedGammaCoMPrime = calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(omega0, time, comDerivativeOrder, cmpPolynomial3D);
      double generalizedDeltaCoMPrime = calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(omega0, time, comDerivativeOrder, cmpPolynomial3D);
      CommonOps.subtract(generalizedAlphaCoMPrimeMatrix, generalizedBetaCoMPrimeMatrix, generalizedAlphaBetaCoMPrimeMatrix);

      double timeSegmentTotal = cmpPolynomial3D.getFinalTime();
      icpToolbox.calculateGeneralizedAlphaPrimeOnCMPSegment3D(omega0, timeSegmentTotal, generalizedAlphaPrimeTerminalMatrix, 0, cmpPolynomial3D);

      calculateCoMQuantity3D(generalizedAlphaBetaCoMPrimeMatrix, generalizedGammaCoMPrime, generalizedDeltaCoMPrime,
                             generalizedAlphaPrimeTerminalMatrix, polynomialCoefficientCombinedVector, icpPositionDesiredFinal, comPositionDesiredInitial,
                             comQuantityDesired);
   }

   /**
    * /**
    * Compute the i-th derivative of x<sub>ref,&phi;</sub> at time t<sub>&phi;</sub>: 
    * <P>
    * x<sup>(i)</sup><sub>ref,&phi;</sub>(t<sub>&phi;</sub>) = 
    * (&alpha;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)
    *  - &beta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)) * p<sub>&phi;</sub>
    *  + &gamma;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) * x<sub>ref,&phi;</sub>(t<sub>0,&phi;</sub>)
    *  + &delta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) * (&xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>)
    *   - &alpha;<sup>(0)</sup><sub>ICP,&phi;</sub>(T<sub>&phi;</sub>) * p<sub>&phi;</sub>)
    *   
    * @param generalizedAlphaBetaCoMPrimeMatrix = &alpha;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)
    *  - &beta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)
    * @param generalizedGammaCoMPrime = &gamma;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)
    * @param generalizedDeltaCoMPrime = &delta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)
    * @param generalizedAlphaPrimeTerminalMatrix = &alpha;<sup>(0)</sup><sub>ICP,&phi;</sub>(T<sub>&phi;</sub>)
    * @param polynomialCoefficientCombinedVector = p<sub>&phi;</sub>
    * @param icpPositionDesiredFinal = &xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>)
    * @param comPositionDesiredInitial = x<sub>ref,&phi;</sub>(t<sub>0,&phi;</sub>)
    * @param comQuantityDesiredToPack = x<sup>(i)</sup><sub>ref,&phi;</sub>(t<sub>&phi;</sub>)
    */
   // FIXME this can probably be more efficient
   public void calculateCoMQuantity3D(DenseMatrix64F generalizedAlphaBetaCoMPrimeMatrix, double generalizedGammaCoMPrime,
                                      double generalizedDeltaCoMPrime, DenseMatrix64F generalizedAlphaPrimeTerminalMatrix,
                                      DenseMatrix64F polynomialCoefficientCombinedVector, FrameTuple3DReadOnly icpPositionDesiredFinal,
                                      FrameTuple3DReadOnly comPositionDesiredInitial, FixedFrameTuple3DBasics comQuantityDesiredToPack)
   {
      int numRows = generalizedAlphaBetaCoMPrimeMatrix.getNumRows();
      M1.reshape(numRows, 1);
      M2.reshape(numRows, 1);
      M3.reshape(numRows, 1);

      CommonOps.mult(generalizedAlphaBetaCoMPrimeMatrix, polynomialCoefficientCombinedVector, M1);

      M2.set(0, generalizedGammaCoMPrime * comPositionDesiredInitial.getX());
      M2.set(1, generalizedGammaCoMPrime * comPositionDesiredInitial.getY());
      M2.set(2, generalizedGammaCoMPrime * comPositionDesiredInitial.getZ());

      CommonOps.mult(generalizedAlphaPrimeTerminalMatrix, polynomialCoefficientCombinedVector, M3);

      M3.set(0, icpPositionDesiredFinal.getX() - M3.get(0));
      M3.set(1, icpPositionDesiredFinal.getY() - M3.get(1));
      M3.set(2, icpPositionDesiredFinal.getZ() - M3.get(2));

      CommonOps.addEquals(M1, M2);
      CommonOps.addEquals(M1, generalizedDeltaCoMPrime, M3);

      comQuantityDesiredToPack.set(M1.get(0), M1.get(1), M1.get(2));
   }

   // FIXME this can probably be more efficient
   public double calculateCoMQuantity1D(DenseMatrix64F generalizedAlphaBetaCoMPrimeMatrix, double generalizedGammaCoMPrime,
                                        double generalizedDeltaCoMPrime, DenseMatrix64F generalizedAlphaPrimeTerminalMatrix,
                                        DenseMatrix64F polynomialCoefficientVector, double icpPositionDesiredFinal, double comPositionDesiredInitial)
   {
      int numRows = generalizedAlphaBetaCoMPrimeMatrix.numRows;
      M1.reshape(numRows, 1);
      M2.reshape(numRows, 1);
      M3.reshape(numRows, 1);

      CommonOps.mult(generalizedAlphaBetaCoMPrimeMatrix, polynomialCoefficientVector, M1);

      M2.set(0, generalizedGammaCoMPrime * comPositionDesiredInitial);
      CommonOps.mult(generalizedAlphaPrimeTerminalMatrix, polynomialCoefficientVector, M3);

      M3.set(0, icpPositionDesiredFinal - M3.get(0));

      CommonOps.addEquals(M1, M2);
      CommonOps.addEquals(M1, generalizedDeltaCoMPrime, M3);

      return M1.get(0);
   }

   /**
    * Compute the i-th derivative of &alpha;<sub>CoM,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &alpha;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>k=0</sub><sup>n</sup> &Sigma;<sub>j=k</sub><sup>n</sup> (-1)<sup>k</sup> * &omega;<sub>0</sub><sup>-j</sup> *
    * t<sup>(j+i)<sup>T</sup></sup> (t<sub>&phi;</sub>)
    */
   public void calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedAlphaCoMPrime, 
                                                                      int alphaCoMDerivativeOrder, FrameTrajectory3D cmpPolynomial3D)
   {
      for (Axis dir : Axis.values)
      {
         Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);
         
         calculateGeneralizedAlphaCoMPrimeOnCMPSegment1D(omega0, time, generalizedAlphaCoMPrimeRow, alphaCoMDerivativeOrder, cmpPolynomial);

         MatrixTools.setMatrixBlock(generalizedAlphaCoMPrime, dir.ordinal(), dir.ordinal() * generalizedAlphaCoMPrimeRow.numCols, generalizedAlphaCoMPrimeRow,
                                    0, 0, generalizedAlphaCoMPrimeRow.numRows, generalizedAlphaCoMPrimeRow.numCols, 1.0);
      }
   }

   public static void calculateGeneralizedAlphaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedAlphaCoMPrimeRow,
                                                                      int alphaCoMDerivativeOrder, Trajectory cmpPolynomial)
   {
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();

      generalizedAlphaCoMPrimeRow.reshape(1, numberOfCoefficients);
      generalizedAlphaCoMPrimeRow.zero();

      for (int i = 0; i < numberOfCoefficients; i++)
      {
         for (int j = i; j < numberOfCoefficients; j++)
         {
            double scalar = Math.pow(-1.0, i) * Math.pow(omega0, -j);
            CommonOps.addEquals(generalizedAlphaCoMPrimeRow, scalar, cmpPolynomial.getXPowersDerivativeVector(j + alphaCoMDerivativeOrder, time));
         }
      }
   }

   /**
    * Compute the i-th derivative of &beta;<sub>CoM,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &beta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>k=0</sub><sup>n</sup> &Sigma;<sub>j=k</sub><sup>n</sup> (-1)<sup>k+i</sup> * &omega;<sub>0</sub><sup>-j+i</sup> *
    * t<sup>(j)<sup>T</sup></sup> (T<sub>&phi;</sub>) * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    */
   public void calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedBetaCoMPrime,
                                                              int betaCoMDerivativeOrder, FrameTrajectory3D cmpPolynomial3D)
   {                  
      for(Axis dir : Axis.values)
      {
         Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);
         
         calculateGeneralizedBetaCoMPrimeOnCMPSegment1D(omega0, time, generalizedBetaCoMPrimeRow, betaCoMDerivativeOrder, cmpPolynomial);

         MatrixTools.setMatrixBlock(generalizedBetaCoMPrime, dir.ordinal(), dir.ordinal() * generalizedBetaCoMPrimeRow.numCols, generalizedBetaCoMPrimeRow,
                                    0, 0, generalizedBetaCoMPrimeRow.numRows, generalizedBetaCoMPrimeRow.numCols, 1.0);
      }
   }

   public static void calculateGeneralizedBetaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedBetaCoMPrimeRow,
                                                                     int betaCoMDerivativeOrder, Trajectory cmpPolynomial)
   {                        
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      double timeSegmentInitial = cmpPolynomial.getInitialTime();

      generalizedBetaCoMPrimeRow.reshape(1, numberOfCoefficients);
      generalizedBetaCoMPrimeRow.zero();

      for (int i = 0; i < numberOfCoefficients; i++)
      {
         for (int j = i; j < numberOfCoefficients; j++)
         {
            double scalar = Math.pow(-1.0, i + betaCoMDerivativeOrder) * Math.pow(omega0, -j + betaCoMDerivativeOrder) * Math.exp(omega0 * (timeSegmentInitial - time));
            CommonOps.addEquals(generalizedBetaCoMPrimeRow, scalar, cmpPolynomial.getXPowersDerivativeVector(j, timeSegmentInitial));
         }
      }
   }

   /**
    * Compute the i-th derivative of &gamma;<sub>CoM,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &gamma;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = (-1)<sup>i</sup> * &omega;<sub>0</sub><sup>i</sup> * 
    * e<sup>&omega;<sub>0</sub>(t<sub>0,&phi;</sub>-t<sub>&phi;</sub>)</sup>
    *
    * @return generalizedGammaCoMPrime
    */
   public double calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(double omega0, double time, int gammaCoMDerivativeOrder, FrameTrajectory3D cmpPolynomial3D)
   {
      double timeSegmentInitial = cmpPolynomial3D.getInitialTime();
      return Math.pow(-1.0, gammaCoMDerivativeOrder) * Math.pow(omega0, gammaCoMDerivativeOrder) * Math.exp(omega0 * (timeSegmentInitial - time));
   }

   /**
    * Compute the i-th derivative of &delta;<sub>CoM,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &delta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = 0.5 * e<sup>&omega;<sub>0</sub>(t<sub>0,&phi;</sub>-T<sub>&phi;</sub>)</sup> * (&omega;<sub>0</sub><sup>i</sup> * 
    * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-t<sub>0,&phi;</sub>)</sup> - (-1)<sup>i</sup> * &omega;<sub>0</sub><sup>i</sup> * e<sup>&omega;<sub>0</sub>(t<sub>0,&phi;</sub>-t<sub>&phi;</sub>)</sup>)
    */
   public double calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(double omega0, double time, int deltaCoMDerivativeOrder, FrameTrajectory3D cmpPolynomial3D)
   {                  
      double timeSegmentInitial = cmpPolynomial3D.getInitialTime();
      double timeSegmentTotal = cmpPolynomial3D.getFinalTime();
      return 0.5 * Math.exp(omega0 * (timeSegmentInitial - timeSegmentTotal)) * (Math.pow(omega0, deltaCoMDerivativeOrder) * Math.exp(omega0 * (time - timeSegmentInitial))
            - Math.pow(-1.0, deltaCoMDerivativeOrder) * Math.pow(omega0, deltaCoMDerivativeOrder) * Math.exp(omega0 * (timeSegmentInitial - time)));
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
   
   private void setPolynomialCoefficientVector3D(DenseMatrix64F polynomialCoefficientCombinedVector, FrameTrajectory3D cmpPolynomial3D)
   {
      for (Axis dir : Axis.values)
      {
         setPolynomialCoefficientVector1D(polynomialCoefficientVector, cmpPolynomial3D.getTrajectory(dir));
         
         MatrixTools.setMatrixBlock(polynomialCoefficientCombinedVector, dir.ordinal() * polynomialCoefficientVector.numRows, 0, polynomialCoefficientVector,
                                    0, 0, polynomialCoefficientVector.numRows, polynomialCoefficientVector.numCols, 1.0);
      }
   }
   
   private static void setPolynomialCoefficientVector1D(DenseMatrix64F polynomialCoefficientVector, Trajectory cmpPolynomial)
   {
      double[] polynomialCoefficients = cmpPolynomial.getCoefficients();

      polynomialCoefficientVector.setData(polynomialCoefficients);
      polynomialCoefficientVector.reshape(cmpPolynomial.getNumberOfCoefficients(), 1);
   }
}
