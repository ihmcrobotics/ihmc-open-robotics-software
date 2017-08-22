package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.YoTrajectory;

/**
 * @author Tim Seyde
 */

public class SmoothCapturePointToolbox
{
   // private static final ThreadLocal<DenseMatrix64F> dummyName = new ThreadLocal<>();
   
   private static final int defaultSize = 100;
   
   private final DenseMatrix64F tPowersDerivativeVector = new DenseMatrix64F(defaultSize, 1);
   private final DenseMatrix64F tPowersDerivativeVectorTranspose = new DenseMatrix64F(defaultSize, 1);
   
   private final DenseMatrix64F generalizedAlphaPrimeRow = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedBetaPrimeRow = new DenseMatrix64F(1, defaultSize);
   
   private final DenseMatrix64F polynomialCoefficientCombinedVector= new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F polynomialCoefficientVector= new DenseMatrix64F(defaultSize, 1);
   
   private final DenseMatrix64F generalizedAlphaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F generalizedBetaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F generalizedGammaPrimeMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F generalizedAlphaBetaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   
   private final DenseMatrix64F M1 = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F M2 = new DenseMatrix64F(defaultSize, defaultSize);
   
   public SmoothCapturePointToolbox()
   {
      
   }
   
   //TODO: implement validity checks
   /**
    * Backward iteration to determine &xi;<sub>ref,&phi;</sub>(0) and &xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>) for all segments &phi;
    */
   public void computeDesiredCornerPoints3D(List<FramePoint3D> entryCornerPointsToPack, List<FramePoint3D> exitCornerPointsToPack,
                                                 List<YoFrameTrajectory3D> cmpPolynomials3D, double omega0)
   {
      YoFrameTrajectory3D cmpPolynomial3D = cmpPolynomials3D.get(cmpPolynomials3D.size() - 1);
      
      cmpPolynomial3D.compute(cmpPolynomial3D.getFinalTime());
      FramePoint3D nextEntryCornerPoint = cmpPolynomial3D.getFramePosition();
            
      for (int i = cmpPolynomials3D.size() - 1; i >= 0; i--)
      {
         cmpPolynomial3D = cmpPolynomials3D.get(i);
         
         FramePoint3D exitCornerPoint = exitCornerPointsToPack.get(i);
         FramePoint3D entryCornerPoint = entryCornerPointsToPack.get(i);
         
         exitCornerPoint.set(nextEntryCornerPoint);
         
         computeDesiredCapturePointPosition3D(omega0, cmpPolynomial3D.getInitialTime(), exitCornerPoint, cmpPolynomial3D, entryCornerPoint);
         nextEntryCornerPoint = entryCornerPoint;
      }
   }
   
   public void computeDesiredCornerPoints(List<FramePoint3D> entryCornerPointsToPack, List<FramePoint3D> exitCornerPointsToPack,
                                                   List<YoFrameTrajectory3D> cmpPolynomials3D, double omega0)
   {
      YoFrameTrajectory3D cmpPolynomial3D = cmpPolynomials3D.get(cmpPolynomials3D.size() - 1);
      
      cmpPolynomial3D.compute(cmpPolynomial3D.getFinalTime());
      FramePoint3D nextEntryCornerPoint = cmpPolynomial3D.getFramePosition();
            
      for (int i = cmpPolynomials3D.size() - 1; i >= 0; i--)
      {
         cmpPolynomial3D = cmpPolynomials3D.get(i);
         
         FramePoint3D exitCornerPoint = exitCornerPointsToPack.get(i);
         FramePoint3D entryCornerPoint = entryCornerPointsToPack.get(i);
         
         exitCornerPoint.set(nextEntryCornerPoint);
         
         computeDesiredCapturePointPosition(omega0, cmpPolynomial3D.getInitialTime(), exitCornerPoint, cmpPolynomial3D, entryCornerPoint);
         nextEntryCornerPoint = entryCornerPoint;
      }
   }
   
   public void computeDesiredCapturePointPosition3D(double omega0, double time, FramePoint3D finalCapturePoint, YoFrameTrajectory3D cmpPolynomial3D, 
                                                         FramePoint3D desiredCapturePointToPack)
   {         
      calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, cmpPolynomial3D, finalCapturePoint, 
                                                                                   desiredCapturePointToPack);
   }
   
   public void computeDesiredCapturePointPosition(double omega0, double time, FramePoint3D finalCapturePoint, YoFrameTrajectory3D cmpPolynomial3D, 
                                                                  FramePoint3D desiredCapturePointToPack)
   {  
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);    
         double icpPositionDesired = calculateICPQuantityFromCorrespondingCMPPolynomial1D(omega0, time, 0, cmpPolynomial, finalCapturePoint.getElement(dir.getIndex()));
         
         desiredCapturePointToPack.setElement(dir.getIndex(), icpPositionDesired);
      }
   }  
   
   public void computeDesiredCapturePointVelocity3D(double omega0, double time, FramePoint3D finalCapturePoint, YoFrameTrajectory3D cmpPolynomial3D,
                                                         FrameVector3D desiredCapturePointVelocityToPack)
   {
      calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 1, cmpPolynomial3D, finalCapturePoint, 
                                                                                   desiredCapturePointVelocityToPack);
   }
   
   public void computeDesiredCapturePointVelocity(double omega0, double time, FramePoint3D finalCapturePoint, YoFrameTrajectory3D cmpPolynomial3D,
                                                                  FrameVector3D desiredCapturePointVelocityToPack)
   {
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);    
         double icpVelocityDesired = calculateICPQuantityFromCorrespondingCMPPolynomial1D(omega0, time, 1, cmpPolynomial, finalCapturePoint.getElement(dir.getIndex()));
         
         desiredCapturePointVelocityToPack.setElement(dir.getIndex(), icpVelocityDesired);
      }
   }
   
   public void computeDesiredCapturePointAcceleration3D(double omega0, double time, FramePoint3D finalCapturePoint, YoFrameTrajectory3D cmpPolynomial3D,
                                                             FrameVector3D desiredCapturePointAccelerationToPack)
   {
      calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 2, cmpPolynomial3D, finalCapturePoint, 
                                                                                   desiredCapturePointAccelerationToPack);
   }
   
   public void computeDesiredCapturePointAcceleration(double omega0, double time, FramePoint3D finalCapturePoint, YoFrameTrajectory3D cmpPolynomial3D,
                                                                      FrameVector3D desiredCapturePointAccelerationToPack)
   {
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);    
         double icpAccelerationDesired = calculateICPQuantityFromCorrespondingCMPPolynomial1D(omega0, time, 2, cmpPolynomial, finalCapturePoint.getElement(dir.getIndex()));
         
         desiredCapturePointAccelerationToPack.setElement(dir.getIndex(), icpAccelerationDesired);
      }
   }  
   
   
   /**
    * Variation of J. Englsberger's "Smooth trajectory generation and push-recovery based on DCM"
    * <br>
    * The approach for calculating DCMs is based on CMP polynomials instead of discrete waypoints
    * 
    * @param icpDerivativeOrder
    * @param cmpPolynomial
    * @param icpPositionDesiredFinal
    * @param time
    * @return
    */
   public void calculateICPQuantityFromCorrespondingCMPPolynomial3D(double omega0, double time, int icpDerivativeOrder, 
                                                                           YoFrameTrajectory3D cmpPolynomial3D, 
                                                                           FrameTuple3D<?, ?> icpPositionDesiredFinal, 
                                                                           FrameTuple3D<?, ?> icpQuantityDesired)
   {        
      int numberOfCoefficients = cmpPolynomial3D.getNumberOfCoefficients();
      if(numberOfCoefficients == -1)
      {
         icpQuantityDesired.setToNaN();
         return;
      }
            
      initializeMatrices3D(numberOfCoefficients);
      setPolynomialCoefficientVector3D(polynomialCoefficientCombinedVector, cmpPolynomial3D);

      calculateGeneralizedAlphaPrimeOnCMPSegment3D(omega0, time, generalizedAlphaPrimeMatrix, icpDerivativeOrder, cmpPolynomial3D);
      calculateGeneralizedBetaPrimeOnCMPSegment3D(omega0, time, generalizedBetaPrimeMatrix, icpDerivativeOrder, cmpPolynomial3D);
      calculateGeneralizedGammaPrimeOnCMPSegment3D(omega0, time, generalizedGammaPrimeMatrix, icpDerivativeOrder, cmpPolynomial3D);
      CommonOps.subtract(generalizedAlphaPrimeMatrix, generalizedBetaPrimeMatrix, generalizedAlphaBetaPrimeMatrix);

      calculateICPQuantity3D(generalizedAlphaBetaPrimeMatrix, generalizedGammaPrimeMatrix, polynomialCoefficientCombinedVector, 
                             icpPositionDesiredFinal, icpQuantityDesired);
   }
   
   public double calculateICPQuantityFromCorrespondingCMPPolynomial1D(double omega0, double time, int icpDerivativeOrder, YoTrajectory cmpPolynomial, double icpPositionDesiredFinal)
   {      
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
   
      initializeMatrices1D(numberOfCoefficients);
      setPolynomialCoefficientVector1D(polynomialCoefficientVector, cmpPolynomial);
   
      calculateGeneralizedAlphaPrimeOnCMPSegment1D(omega0, time, generalizedAlphaPrimeMatrix, icpDerivativeOrder, cmpPolynomial);
      calculateGeneralizedBetaPrimeOnCMPSegment1D(omega0, time, generalizedBetaPrimeMatrix, icpDerivativeOrder, cmpPolynomial);
      calculateGeneralizedGammaPrimeOnCMPSegment1D(omega0, time, generalizedGammaPrimeMatrix, icpDerivativeOrder, cmpPolynomial);
      CommonOps.subtract(generalizedAlphaPrimeMatrix, generalizedBetaPrimeMatrix, generalizedAlphaBetaPrimeMatrix);
   
      return calculateICPQuantity1D(generalizedAlphaBetaPrimeMatrix, generalizedGammaPrimeMatrix, polynomialCoefficientVector, icpPositionDesiredFinal);
   }
   
   /**
    * Compute the i-th derivative of &xi;<sub>ref,&phi;</sub> at time t<sub>&phi;</sub>: 
    * <P>
    * &xi;<sup>(i)</sup><sub>ref,&phi;</sub>(t<sub>&phi;</sub>) = 
    * (&alpha;<sup>(i)</sup><sub>ICP,&phi;</sub>(t<sub>&phi;</sub>)
    *  - &beta;<sup>(i)</sup><sub>ICP,&phi;</sub>(t<sub>&phi;</sub>)) * p<sub>&phi;</sub>
    *  + &gamma;<sup>(i)</sup><sub>ICP,&phi;</sub>(t<sub>&phi;</sub>) * &xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>)
    * 
    * @param generalizedAlphaBetaPrimeMatrix
    * @param generalizedGammaPrimeMatrix
    * @param polynomialCoefficientVector
    * @param icpPositionDesiredFinal
    * @return
    */
   public void calculateICPQuantity3D(DenseMatrix64F generalizedAlphaBetaPrimeMatrix, DenseMatrix64F generalizedGammaPrimeMatrix,
                                             DenseMatrix64F polynomialCoefficientCombinedVector, FrameTuple3D<?, ?> icpPositionDesiredFinal,
                                             FrameTuple3D<?, ?> icpQuantityDesired)
   {
      M1.reshape(generalizedAlphaBetaPrimeMatrix.getNumRows(), polynomialCoefficientCombinedVector.getNumCols());
      M1.zero();

      CommonOps.mult(generalizedAlphaBetaPrimeMatrix, polynomialCoefficientCombinedVector, M1);
      
      M2.reshape(M1.getNumRows(),  M1.getNumCols());
      M2.set(0, 0, generalizedGammaPrimeMatrix.get(0, 0) * icpPositionDesiredFinal.getX());
      M2.set(1, 0, generalizedGammaPrimeMatrix.get(0, 0) * icpPositionDesiredFinal.getY());
      M2.set(2, 0, generalizedGammaPrimeMatrix.get(0, 0) * icpPositionDesiredFinal.getZ());
            
      CommonOps.addEquals(M1, M2);
      
      icpQuantityDesired.set(M1.get(0, 0), M1.get(1, 0), M1.get(2, 0));
   }
   
   public double calculateICPQuantity1D(DenseMatrix64F generalizedAlphaBetaPrimeMatrix, DenseMatrix64F generalizedGammaPrimeMatrix,
                                               DenseMatrix64F polynomialCoefficientVector, double icpPositionDesiredFinal)
   {
      M1.reshape(generalizedAlphaBetaPrimeMatrix.getNumRows(), polynomialCoefficientVector.getNumCols());
      M1.zero();

      CommonOps.mult(generalizedAlphaBetaPrimeMatrix, polynomialCoefficientVector, M1);

      M2.reshape(M1.getNumRows(),  M1.getNumCols());
      M2.set(0, 0, generalizedGammaPrimeMatrix.get(0, 0) * icpPositionDesiredFinal);
            
      CommonOps.addEquals(M1, M2);
      
      return M1.get(0, 0);
   }
   
   /**
    * Compute the i-th derivative of &alpha;<sub>ICP,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &alpha;<sup>(i)</sup><sub>ICP,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>j=0</sub><sup>n</sup> &omega;<sub>0</sub><sup>-j</sup> *
    * t<sup>(j+i)<sup>T</sup></sup> (t<sub>&phi;</sub>)
    * 
    * @param generalizedAlphaPrime
    * @param alphaDerivativeOrder
    * @param cmpPolynomial
    * @param time
    */
   public void calculateGeneralizedAlphaPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedAlphaPrime, 
                                                                   int alphaDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);
         
         calculateGeneralizedAlphaPrimeOnCMPSegment1D(omega0, time, generalizedAlphaPrimeRow, alphaDerivativeOrder, cmpPolynomial);
         
         CommonOps.insert(generalizedAlphaPrimeRow, generalizedAlphaPrime, dir.ordinal(), dir.ordinal() * generalizedAlphaPrimeRow.numCols);
      }
   }
   
   public void calculateGeneralizedAlphaPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedAlphaPrimeRow,
                                                                   int alphaDerivativeOrder, YoTrajectory cmpPolynomial)
   {
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      
      generalizedAlphaPrimeRow.reshape(1, numberOfCoefficients);
      generalizedAlphaPrimeRow.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.zero();
         tPowersDerivativeVectorTranspose.zero();
         
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i + alphaDerivativeOrder, time));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
                  
         double scalar = Math.pow(omega0, -i);
         CommonOps.addEquals(generalizedAlphaPrimeRow, scalar, tPowersDerivativeVectorTranspose);
      }
   }
   
   /**
    * Compute the i-th derivative of &beta;<sub>ICP,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &beta;<sup>(i)</sup><sub>ICP,&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>j=0</sub><sup>n</sup> &omega;<sub>0</sub><sup>-(j-i)</sup> *
    * t<sup>(j)<sup>T</sup></sup> (T<sub>&phi;</sub>) * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    * 
    * @param generalizedBetaPrime
    * @param betaDerivativeOrder
    * @param cmpPolynomial
    * @param time
    */
   public void calculateGeneralizedBetaPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedBetaPrime, 
                                                                  int betaDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {                  
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);
         
         calculateGeneralizedBetaPrimeOnCMPSegment1D(omega0, time, generalizedBetaPrimeRow, betaDerivativeOrder, cmpPolynomial);
         
         CommonOps.insert(generalizedBetaPrimeRow, generalizedBetaPrime, dir.ordinal(), dir.ordinal() * generalizedBetaPrimeRow.numCols);
      }
   }
   
   public void calculateGeneralizedBetaPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedBetaPrimeRow, 
                                                                  int betaDerivativeOrder, YoTrajectory cmpPolynomial)
   {                  
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      
      generalizedBetaPrimeRow.reshape(1, numberOfCoefficients);
      generalizedBetaPrimeRow.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.zero();
         tPowersDerivativeVectorTranspose.zero();
         
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i, timeSegmentTotal));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
                  
         double scalar = Math.pow(omega0, betaDerivativeOrder-i) * Math.exp(omega0*(time-timeSegmentTotal));
         CommonOps.addEquals(generalizedBetaPrimeRow, scalar, tPowersDerivativeVectorTranspose);
      }
   }

   /**
    * Compute the i-th derivative of &gamma;<sub>ICP,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &gamma;<sup>(i)</sup><sub>ICP,&phi;</sub>(t<sub>&phi;</sub>) = &omega;<sub>0</sub><sup>i</sup> * 
    * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    * 
    * @param generalizedGammaPrime
    * @param gammaDerivativeOrder
    * @param cmpPolynomial3D
    * @param time
    */
   public void calculateGeneralizedGammaPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedGammaPrime, 
                                                                   int gammaDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {      
      double timeSegmentTotal = cmpPolynomial3D.getFinalTime();
      double ddGamaPrimeValue = Math.pow(omega0, gammaDerivativeOrder)*Math.exp(omega0 * (time - timeSegmentTotal));
      generalizedGammaPrime.set(0, 0, ddGamaPrimeValue);
   }
   
   public void calculateGeneralizedGammaPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedGammaPrime, 
                                                                   int gammaDerivativeOrder, YoTrajectory cmpPolynomial)
   {      
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      double ddGamaPrimeValue = Math.pow(omega0, gammaDerivativeOrder)*Math.exp(omega0 * (time - timeSegmentTotal));
      generalizedGammaPrime.set(0, 0, ddGamaPrimeValue);
   }
   
   public void calculateGeneralizedMatricesPrimeOnCMPSegment3D(double omega0, double time, int derivativeOrder, YoFrameTrajectory3D cmpPolynomial, 
                                                               DenseMatrix64F generalizedAlphaPrime, DenseMatrix64F generalizedBetaPrime,
                                                               DenseMatrix64F generalizedGammaPrime, DenseMatrix64F generalizedAlphaBetaPrime)
   {
      calculateGeneralizedAlphaPrimeOnCMPSegment3D(omega0, time, generalizedAlphaPrime, derivativeOrder, cmpPolynomial);
      calculateGeneralizedBetaPrimeOnCMPSegment3D(omega0, time, generalizedBetaPrime, derivativeOrder, cmpPolynomial);
      calculateGeneralizedGammaPrimeOnCMPSegment3D(omega0, time, generalizedGammaPrime, derivativeOrder, cmpPolynomial);
      CommonOps.subtract(generalizedAlphaPrime, generalizedBetaPrime, generalizedAlphaBetaPrime);
   }
   
   public void calculateGeneralizedMatricesPrimeOnCMPSegment1D(double omega0, double time, int derivativeOrder, YoTrajectory cmpPolynomial,
                                                               DenseMatrix64F generalizedAlphaPrime, DenseMatrix64F generalizedBetaPrime,
                                                               DenseMatrix64F generalizedGammaPrime, DenseMatrix64F generalizedAlphaBetaPrime)
   {
      calculateGeneralizedAlphaPrimeOnCMPSegment1D(omega0, time, generalizedAlphaPrime, derivativeOrder, cmpPolynomial);
      calculateGeneralizedBetaPrimeOnCMPSegment1D(omega0, time, generalizedBetaPrime, derivativeOrder, cmpPolynomial);
      calculateGeneralizedGammaPrimeOnCMPSegment1D(omega0, time, generalizedGammaPrime, derivativeOrder, cmpPolynomial);
      CommonOps.subtract(generalizedAlphaPrime, generalizedBetaPrime, generalizedAlphaBetaPrime);
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
      
      generalizedAlphaPrimeMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedAlphaPrimeMatrix.zero();
      
      generalizedBetaPrimeMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedBetaPrimeMatrix.zero();
      
      generalizedAlphaBetaPrimeMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedAlphaBetaPrimeMatrix.zero();
      
      generalizedGammaPrimeMatrix.reshape(1, 1);
      generalizedGammaPrimeMatrix.zero();
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
