package us.ihmc.commonWalkingControlModules.dynamicReachability;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.YoTrajectory;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.SmoothCapturePointToolbox;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameTuple;
import us.ihmc.robotics.geometry.FrameVector;

public class SmoothCoMIntegrationToolbox
{
   private static final int defaultSize = 1000;
   
   private final DenseMatrix64F tPowersDerivativeVector = new DenseMatrix64F(defaultSize, 1);
   private final DenseMatrix64F tPowersDerivativeVectorTranspose = new DenseMatrix64F(defaultSize, 1);
   
   private final DenseMatrix64F generalizedAlphaCoMPrimeRow = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedBetaCoMPrimeRow = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedDeltaCoMPrimeRow = new DenseMatrix64F(1, defaultSize);
   
   private final DenseMatrix64F polynomialCoefficientCombinedVector= new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F polynomialCoefficientVector= new DenseMatrix64F(defaultSize, 1);
   
   private final DenseMatrix64F generalizedAlphaCoMPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F generalizedBetaCoMPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F generalizedGammaCoMPrimeMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F generalizedDeltaCoMPrimeMatrix = new DenseMatrix64F(1, 1);
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
   
   //TODO: implement validity checks
   public void computeDesiredCenterOfMassCornerPoints(List<FramePoint> entryCornerPointsToPack, List<FramePoint> exitCornerPointsToPack,
                                                             List<FramePoint> entryCoMCornerPointsToPack, List<FramePoint> exitCoMCornerPointsToPack,
                                                             List<YoFrameTrajectory3D> cmpPolynomials3D, FramePoint initialCenterOfMass, double omega0)
   {
      YoFrameTrajectory3D cmpPolynomial3D = cmpPolynomials3D.get(0);
      FramePoint previousExitCoMCornerPoint = initialCenterOfMass;
      
      for (int i = 0; i < cmpPolynomials3D.size(); i++)
      {
         cmpPolynomial3D = cmpPolynomials3D.get(i);
         
         FramePoint exitCornerPoint = exitCornerPointsToPack.get(i);
         
         FramePoint entryCoMCornerPoint = entryCoMCornerPointsToPack.get(i);
         FramePoint exitCoMCornerPoint = exitCoMCornerPointsToPack.get(i);
         
         entryCoMCornerPoint.set(previousExitCoMCornerPoint);
         
         computeDesiredCenterOfMassPosition(omega0, cmpPolynomial3D.getFinalTime(), exitCornerPoint, entryCoMCornerPoint, cmpPolynomial3D, exitCoMCornerPoint);
         
         previousExitCoMCornerPoint = exitCoMCornerPoint;
      }
   }
   
   public void computeDesiredCenterOfMassPosition(double omega0, double time, FramePoint finalCapturePoint, FramePoint initialCenterOfMass, YoFrameTrajectory3D cmpPolynomial3D, 
                                                         FramePoint desiredCenterOfMassPositionToPack)
   {         
      calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, cmpPolynomial3D, finalCapturePoint, initialCenterOfMass, desiredCenterOfMassPositionToPack);
   }
   
   public void computeDesiredCenterOfMassVelocity(double omega0, double time, FramePoint finalCapturePoint, FramePoint initialCenterOfMass, YoFrameTrajectory3D cmpPolynomial3D, 
                                                         FrameVector desiredCenterOfMassVelocityToPack)
   {         
      calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 1, cmpPolynomial3D, finalCapturePoint, initialCenterOfMass, desiredCenterOfMassVelocityToPack);
   }
   
   public void computeDesiredCenterOfMassAcceleration(double omega0, double time, FramePoint finalCapturePoint, FramePoint initialCenterOfMass, YoFrameTrajectory3D cmpPolynomial3D, 
                                                         FrameVector desiredCenterOfMassAccelerationToPack)
   {         
      calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 2, cmpPolynomial3D, finalCapturePoint, initialCenterOfMass, desiredCenterOfMassAccelerationToPack);
   }  
   
   
   public void calculateCoMQuantityFromCorrespondingCMPPolynomial3D(double omega0, double time, int comDerivativeOrder, 
                                                                           YoFrameTrajectory3D cmpPolynomial3D, 
                                                                           FrameTuple<?, ?> icpPositionDesiredFinal, 
                                                                           FrameTuple<?, ?> comPositionDesiredInitial, 
                                                                           FrameTuple<?, ?> comQuantityDesired)
   {        
      int numberOfCoefficients = cmpPolynomial3D.getNumberOfCoefficients();
      if(numberOfCoefficients < 0)
      {
         comQuantityDesired.setToNaN();
         return;
      }
      
      initializeMatrices3D(numberOfCoefficients);
      setPolynomialCoefficientVector3D(polynomialCoefficientCombinedVector, cmpPolynomial3D);

      calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(omega0, time, generalizedAlphaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial3D);
      calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(omega0, time, generalizedBetaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial3D);
      calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(omega0, time, generalizedGammaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial3D);
      calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(omega0, time, generalizedDeltaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial3D);
      CommonOps.subtract(generalizedAlphaCoMPrimeMatrix, generalizedBetaCoMPrimeMatrix, generalizedAlphaBetaCoMPrimeMatrix);
      
      double timeSegmentTotal = cmpPolynomial3D.getFinalTime();
      icpToolbox.calculateGeneralizedAlphaPrimeOnCMPSegment3D(omega0, timeSegmentTotal, generalizedAlphaPrimeTerminalMatrix, 0, cmpPolynomial3D);
      
      calculateCoMQuantity3D(generalizedAlphaBetaCoMPrimeMatrix, generalizedGammaCoMPrimeMatrix, generalizedDeltaCoMPrimeMatrix, generalizedAlphaPrimeTerminalMatrix, polynomialCoefficientCombinedVector, 
                             icpPositionDesiredFinal, comPositionDesiredInitial, comQuantityDesired);
   }
   
   public double calculateCoMQuantityFromCorrespondingCMPPolynomial1D(double omega0, double time, int comDerivativeOrder, YoTrajectory cmpPolynomial, double icpPositionDesiredFinal, double comPositionDesiredInitial)
   {      
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
   
      initializeMatrices1D(numberOfCoefficients);
      setPolynomialCoefficientVector1D(polynomialCoefficientVector, cmpPolynomial);
   
      calculateGeneralizedAlphaCoMPrimeOnCMPSegment1D(omega0, time, generalizedAlphaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial);
      calculateGeneralizedBetaCoMPrimeOnCMPSegment1D(omega0, time, generalizedBetaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial);
      calculateGeneralizedGammaCoMPrimeOnCMPSegment1D(omega0, time, generalizedGammaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial);
      calculateGeneralizedDeltaCoMPrimeOnCMPSegment1D(omega0, time, generalizedDeltaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial);
      CommonOps.subtract(generalizedAlphaCoMPrimeMatrix, generalizedBetaCoMPrimeMatrix, generalizedAlphaBetaCoMPrimeMatrix);
      
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      icpToolbox.calculateGeneralizedAlphaPrimeOnCMPSegment1D(omega0, timeSegmentTotal, generalizedAlphaPrimeTerminalMatrix, 0, cmpPolynomial);
   
      return calculateCoMQuantity1D(generalizedAlphaBetaCoMPrimeMatrix, generalizedGammaCoMPrimeMatrix, generalizedDeltaCoMPrimeMatrix, generalizedAlphaPrimeTerminalMatrix, polynomialCoefficientVector, 
                                    icpPositionDesiredFinal, comPositionDesiredInitial);
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
    * @param generalizedGammaCoMPrimeMatrix = &gamma;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)
    * @param generalizedDeltaCoMPrimeMatrix = &delta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>)
    * @param generalizedAlphaPrimeTerminalMatrix = &alpha;<sup>(0)</sup><sub>ICP,&phi;</sub>(T<sub>&phi;</sub>)
    * @param polynomialCoefficientCombinedVector = p<sub>&phi;</sub>
    * @param icpPositionDesiredFinal = &xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>)
    * @param comPositionDesiredInitial = x<sub>ref,&phi;</sub>(t<sub>0,&phi;</sub>)
    * @param comQuantityDesired = x<sup>(i)</sup><sub>ref,&phi;</sub>(t<sub>&phi;</sub>)
    */
   public void calculateCoMQuantity3D(DenseMatrix64F generalizedAlphaBetaCoMPrimeMatrix, DenseMatrix64F generalizedGammaCoMPrimeMatrix, DenseMatrix64F generalizedDeltaCoMPrimeMatrix, 
                                             DenseMatrix64F generalizedAlphaPrimeTerminalMatrix, DenseMatrix64F polynomialCoefficientCombinedVector, FrameTuple<?, ?> icpPositionDesiredFinal, 
                                             FrameTuple<?, ?> comPositionDesiredInitial, FrameTuple<?, ?> comQuantityDesired)
   {
      M1.reshape(generalizedAlphaBetaCoMPrimeMatrix.getNumRows(), polynomialCoefficientCombinedVector.getNumCols());
      M1.zero();

      CommonOps.mult(generalizedAlphaBetaCoMPrimeMatrix, polynomialCoefficientCombinedVector, M1);
      
      M2.reshape(M1.getNumRows(),  M1.getNumCols());
      M2.set(0, 0, generalizedGammaCoMPrimeMatrix.get(0, 0) * comPositionDesiredInitial.getX());
      M2.set(1, 0, generalizedGammaCoMPrimeMatrix.get(0, 0) * comPositionDesiredInitial.getY());
      M2.set(2, 0, generalizedGammaCoMPrimeMatrix.get(0, 0) * comPositionDesiredInitial.getZ());

      M3.reshape(generalizedAlphaBetaCoMPrimeMatrix.getNumRows(), polynomialCoefficientCombinedVector.getNumCols());
      M3.zero();
      
      CommonOps.mult(generalizedAlphaPrimeTerminalMatrix, polynomialCoefficientCombinedVector, M3);
      M3.set(0, 0, icpPositionDesiredFinal.getX() - M3.get(0, 0));
      M3.set(1, 0, icpPositionDesiredFinal.getY() - M3.get(1, 0));
      M3.set(2, 0, icpPositionDesiredFinal.getZ() - M3.get(2, 0));
      CommonOps.scale(generalizedDeltaCoMPrimeMatrix.get(0, 0), M3);
            
      CommonOps.addEquals(M1, M2);
      CommonOps.addEquals(M1, M3);
      
      comQuantityDesired.set(M1.get(0, 0), M1.get(1, 0), M1.get(2, 0));
   }
   
   public double calculateCoMQuantity1D(DenseMatrix64F generalizedAlphaBetaCoMPrimeMatrix, DenseMatrix64F generalizedGammaCoMPrimeMatrix, DenseMatrix64F generalizedDeltaCoMPrimeMatrix, 
                                               DenseMatrix64F generalizedAlphaPrimeTerminalMatrix, DenseMatrix64F polynomialCoefficientVector, double icpPositionDesiredFinal, double comPositionDesiredInitial)
   {
      M1.reshape(generalizedAlphaBetaCoMPrimeMatrix.getNumRows(), polynomialCoefficientVector.getNumCols());
      M1.zero();

      CommonOps.mult(generalizedAlphaBetaCoMPrimeMatrix, polynomialCoefficientVector, M1);

      M2.reshape(M1.getNumRows(),  M1.getNumCols());
      M2.set(0, 0, generalizedGammaCoMPrimeMatrix.get(0, 0) * comPositionDesiredInitial);
      
      M3.reshape(generalizedAlphaBetaCoMPrimeMatrix.getNumRows(), polynomialCoefficientVector.getNumCols());
      M3.zero();

      CommonOps.mult(generalizedAlphaPrimeTerminalMatrix, polynomialCoefficientVector, M3);
      M3.set(0, 0, icpPositionDesiredFinal - M3.get(0, 0));
      CommonOps.scale(generalizedDeltaCoMPrimeMatrix.get(0, 0), M3);
            
      CommonOps.addEquals(M1, M2);
      CommonOps.addEquals(M1, M3);
      
      return M1.get(0, 0);
   }
   
   /**
    * Compute the i-th derivative of &alpha;<sub>CoM,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &alpha;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>k=0</sub><sup>n</sup> &Sigma;<sub>j=k</sub><sup>n</sup> (-1)<sup>k</sup> * &omega;<sub>0</sub><sup>-j</sup> *
    * t<sup>(j+i)<sup>T</sup></sup> (t<sub>&phi;</sub>)
    * 
    * @param omega0
    * @param time
    * @param generalizedAlphaCoMPrime
    * @param alphaCoMDerivativeOrder
    * @param cmpPolynomial3D
    */
   public void calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedAlphaCoMPrime, 
                                                                      int alphaCoMDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);
         
         calculateGeneralizedAlphaCoMPrimeOnCMPSegment1D(omega0, time, generalizedAlphaCoMPrimeRow, alphaCoMDerivativeOrder, cmpPolynomial);
         
         CommonOps.insert(generalizedAlphaCoMPrimeRow, generalizedAlphaCoMPrime, dir.ordinal(), dir.ordinal() * generalizedAlphaCoMPrimeRow.numCols);
      }
   }
   
   public void calculateGeneralizedAlphaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedAlphaCoMPrimeRow,
                                                                      int alphaCoMDerivativeOrder, YoTrajectory cmpPolynomial)
   {
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      
      generalizedAlphaCoMPrimeRow.reshape(1, numberOfCoefficients);
      generalizedAlphaCoMPrimeRow.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         for(int j = i; j < numberOfCoefficients; j++)
         {
            tPowersDerivativeVector.zero();
            tPowersDerivativeVectorTranspose.zero();
            
            tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(j + alphaCoMDerivativeOrder, time));
            CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
            
            double scalar = Math.pow(-1.0, i) * Math.pow(omega0, -j);
            CommonOps.addEquals(generalizedAlphaCoMPrimeRow, scalar, tPowersDerivativeVectorTranspose);            
         }
      }
   }
   
   /**
    * Compute the i-th derivative of &beta;<sub>CoM,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &beta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>k=0</sub><sup>n</sup> &Sigma;<sub>j=k</sub><sup>n</sup> (-1)<sup>k+i</sup> * &omega;<sub>0</sub><sup>-j+i</sup> *
    * t<sup>(j)<sup>T</sup></sup> (T<sub>&phi;</sub>) * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    * 
    * @param omega0
    * @param time
    * @param generalizedBetaCoMPrime
    * @param betaCoMDerivativeOrder
    * @param cmpPolynomial3D
    */
   public void calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedBetaCoMPrime, 
                                                                     int betaCoMDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {                  
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);
         
         calculateGeneralizedBetaCoMPrimeOnCMPSegment1D(omega0, time, generalizedBetaCoMPrimeRow, betaCoMDerivativeOrder, cmpPolynomial);
         
         CommonOps.insert(generalizedBetaCoMPrimeRow, generalizedBetaCoMPrime, dir.ordinal(), dir.ordinal() * generalizedBetaCoMPrimeRow.numCols);
      }
   }
   
   public void calculateGeneralizedBetaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedBetaCoMPrimeRow, 
                                                                     int betaCoMDerivativeOrder, YoTrajectory cmpPolynomial)
   {                        
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      double timeSegmentInitial = cmpPolynomial.getInitialTime();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      
      generalizedBetaCoMPrimeRow.reshape(1, numberOfCoefficients);
      generalizedBetaCoMPrimeRow.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         for(int j = i; j < numberOfCoefficients; j++)
         {
            tPowersDerivativeVector.zero();
            tPowersDerivativeVectorTranspose.zero();
            
            tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(j, timeSegmentInitial));
            CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
            
            double scalar =  Math.pow(-1.0, i + betaCoMDerivativeOrder) * Math.pow(omega0,  -j + betaCoMDerivativeOrder) * Math.exp(omega0*(timeSegmentInitial-time));
            CommonOps.addEquals(generalizedBetaCoMPrimeRow, scalar, tPowersDerivativeVectorTranspose);            
         }
      }
   }
   
   /**
    * Compute the i-th derivative of &gamma;<sub>CoM,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &gamma;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = (-1)<sup>i</sup> * &omega;<sub>0</sub><sup>i</sup> * 
    * e<sup>&omega;<sub>0</sub>(t<sub>0,&phi;</sub>-t<sub>&phi;</sub>)</sup>
    * 
    * @param omega0
    * @param time
    * @param generalizedGammaCoMPrime
    * @param gammaCoMDerivativeOrder
    * @param cmpPolynomial3D
    */
   public void calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedGammaCoMPrime, 
                                                                      int gammaCoMDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {      
      double timeSegmentInitial = cmpPolynomial3D.getInitialTime();
      double ddGamaPrimeValue = Math.pow(-1.0, gammaCoMDerivativeOrder) * Math.pow(omega0, gammaCoMDerivativeOrder) * Math.exp(omega0 * (timeSegmentInitial - time));
      generalizedGammaCoMPrime.set(0, 0, ddGamaPrimeValue);
   }
   
   public void calculateGeneralizedGammaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedGammaCoMPrime, 
                                                                      int gammaCoMDerivativeOrder, YoTrajectory cmpPolynomial)
   {      
      double timeSegmentInitial = cmpPolynomial.getInitialTime();
      double ddGamaPrimeValue = Math.pow(-1.0, gammaCoMDerivativeOrder) * Math.pow(omega0, gammaCoMDerivativeOrder) * Math.exp(omega0 * (timeSegmentInitial - time));
      generalizedGammaCoMPrime.set(0, 0, ddGamaPrimeValue);
   }
   
   /**
    * Compute the i-th derivative of &delta;<sub>CoM,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &delta;<sup>(i)</sup><sub>CoM,&phi;</sub>(t<sub>&phi;</sub>) = 0.5 * e<sup>&omega;<sub>0</sub>(t<sub>0,&phi;</sub>-T<sub>&phi;</sub>)</sup> * (&omega;<sub>0</sub><sup>i</sup> * 
    * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-t<sub>0,&phi;</sub>)</sup> - (-1)<sup>i</sup> * &omega;<sub>0</sub><sup>i</sup> * e<sup>&omega;<sub>0</sub>(t<sub>0,&phi;</sub>-t<sub>&phi;</sub>)</sup>)
    * 
    * @param omega0
    * @param time
    * @param generalizedDeltaCoMPrime
    * @param deltaCoMDerivativeOrder
    * @param cmpPolynomial3D
    */
   public void calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedDeltaCoMPrime, 
                                                                      int deltaCoMDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {                  
      double timeSegmentInitial = cmpPolynomial3D.getInitialTime();
      double timeSegmentTotal = cmpPolynomial3D.getFinalTime();
      double ddDeltaPrimeValue = 0.5 * Math.exp(omega0 * (timeSegmentInitial - timeSegmentTotal)) * (Math.pow(omega0, deltaCoMDerivativeOrder) * Math.exp(omega0 * (time - timeSegmentInitial)) - Math.pow(-1.0, deltaCoMDerivativeOrder) * Math.pow(omega0, deltaCoMDerivativeOrder) * Math.exp(omega0 * (timeSegmentInitial - time)));
      generalizedDeltaCoMPrime.set(0, 0, ddDeltaPrimeValue);
   }
   
   public void calculateGeneralizedDeltaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedDeltaCoMPrime, 
                                                                      int deltaCoMDerivativeOrder, YoTrajectory cmpPolynomial)
   {                  
      double timeSegmentInitial = cmpPolynomial.getInitialTime();
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      double ddDeltaPrimeValue = 0.5 * Math.exp(omega0 * (timeSegmentInitial - timeSegmentTotal)) * (Math.pow(omega0, deltaCoMDerivativeOrder) * Math.exp(omega0 * (time - timeSegmentInitial)) - Math.pow(-1.0, deltaCoMDerivativeOrder) * Math.pow(omega0, deltaCoMDerivativeOrder) * Math.exp(omega0 * (timeSegmentInitial - time)));
      generalizedDeltaCoMPrime.set(0, 0, ddDeltaPrimeValue);
   }
   
   private void initializeMatrices3D(int numberOfCoefficients)
   {
      initializeMatrices(3, numberOfCoefficients);
   }
   
   private void initializeMatrices1D(int numberOfCoefficients)
   {
      initializeMatrices(1, numberOfCoefficients);
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
      
      generalizedGammaCoMPrimeMatrix.reshape(1, 1);
      generalizedGammaCoMPrimeMatrix.zero();
      
      generalizedDeltaCoMPrimeMatrix.reshape(1, 1);
      generalizedDeltaCoMPrimeMatrix.zero();
      
      generalizedAlphaPrimeTerminalMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedAlphaPrimeTerminalMatrix.zero();
   }
   
   private void setPolynomialCoefficientVector3D(DenseMatrix64F polynomialCoefficientCombinedVector, YoFrameTrajectory3D cmpPolynomial3D)
   {
      for(Direction dir : Direction.values())
      {
         setPolynomialCoefficientVector1D(polynomialCoefficientVector, cmpPolynomial3D.getYoTrajectory(dir));
         
         CommonOps.insert(polynomialCoefficientVector, polynomialCoefficientCombinedVector, dir.ordinal() * polynomialCoefficientVector.numRows, 0);
      }
   }
   
   private void setPolynomialCoefficientVector1D(DenseMatrix64F polynomialCoefficientVector, YoTrajectory cmpPolynomial)
   {
      double[] polynomialCoefficients = cmpPolynomial.getCoefficients();
      
      polynomialCoefficientVector.reshape(cmpPolynomial.getNumberOfCoefficients(), 1);
      polynomialCoefficientVector.zero();
      
      polynomialCoefficientVector.setData(polynomialCoefficients);
   }
}
