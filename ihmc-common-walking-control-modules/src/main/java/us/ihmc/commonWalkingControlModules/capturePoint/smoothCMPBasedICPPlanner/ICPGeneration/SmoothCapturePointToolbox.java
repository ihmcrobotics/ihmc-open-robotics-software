package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.Trajectory;

/**
 * @author Tim Seyde
 */

public class SmoothCapturePointToolbox
{
   private static final int defaultSize = 100;
   
   private final DenseMatrix64F generalizedAlphaPrimeRow = new DenseMatrix64F(1, defaultSize);
   private final DenseMatrix64F generalizedBetaPrimeRow = new DenseMatrix64F(1, defaultSize);
   
   private final DenseMatrix64F polynomialCoefficientCombinedVector= new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F polynomialCoefficientVector= new DenseMatrix64F(defaultSize, 1);
   
   private final DenseMatrix64F generalizedAlphaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F generalizedBetaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private final DenseMatrix64F generalizedAlphaBetaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   
   private final DenseMatrix64F M1 = new DenseMatrix64F(defaultSize, defaultSize);

   //TODO: implement validity checks
   /**
    * Backward iteration to determine &xi;<sub>ref,&phi;</sub>(0) and &xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>) for all segments &phi;
    */
   public void computeDesiredCornerPoints3D(List<FramePoint3D> entryCornerPointsToPack, List<FramePoint3D> exitCornerPointsToPack,
                                                 List<FrameTrajectory3D> cmpPolynomials3D, double omega0)
   {
      FrameTrajectory3D cmpPolynomial3D = cmpPolynomials3D.get(cmpPolynomials3D.size() - 1);
      
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
                                                   List<FrameTrajectory3D> cmpPolynomials3D, double omega0)
   {
      FrameTrajectory3D cmpPolynomial3D = cmpPolynomials3D.get(cmpPolynomials3D.size() - 1);
      
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
   
   public void computeDesiredCapturePointPosition3D(double omega0, double time, FramePoint3D finalCapturePoint, FrameTrajectory3D cmpPolynomial3D, 
                                                         FramePoint3D desiredCapturePointToPack)
   {         
      calculateICPQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, cmpPolynomial3D, finalCapturePoint, desiredCapturePointToPack);
   }
   
   public void computeDesiredCapturePointPosition(double omega0, double time, FramePoint3D finalCapturePoint, FrameTrajectory3D cmpPolynomial3D,
                                                                  FramePoint3D desiredCapturePointToPack)
   {  
      for(Axis dir : Axis.values)
      {
         Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);    
         double icpPositionDesired = calculateICPQuantityFromCorrespondingCMPPolynomial1D(omega0, time, 0, cmpPolynomial, finalCapturePoint.getElement(dir.ordinal()));
         
         desiredCapturePointToPack.setElement(dir.ordinal(), icpPositionDesired);
      }
   }  
   
   public void computeDesiredCapturePointVelocity(double omega0, double time, FramePoint3D finalCapturePoint, FrameTrajectory3D cmpPolynomial3D,
                                                                  FrameVector3D desiredCapturePointVelocityToPack)
   {
      for(Axis dir : Axis.values)
      {
         Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);    
         double icpVelocityDesired = calculateICPQuantityFromCorrespondingCMPPolynomial1D(omega0, time, 1, cmpPolynomial, finalCapturePoint.getElement(dir.ordinal()));
         
         desiredCapturePointVelocityToPack.setElement(dir.ordinal(), icpVelocityDesired);
      }
   }
   
   public void computeDesiredCapturePointAcceleration(double omega0, double time, FramePoint3D finalCapturePoint, FrameTrajectory3D cmpPolynomial3D,
                                                                      FrameVector3D desiredCapturePointAccelerationToPack)
   {
      for(Axis dir : Axis.values)
      {
         Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);    
         double icpAccelerationDesired = calculateICPQuantityFromCorrespondingCMPPolynomial1D(omega0, time, 2, cmpPolynomial, finalCapturePoint.getElement(dir.ordinal()));
         
         desiredCapturePointAccelerationToPack.setElement(dir.ordinal(), icpAccelerationDesired);
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
   public void calculateICPQuantityFromCorrespondingCMPPolynomial3D(double omega0, double time, int icpDerivativeOrder, FrameTrajectory3D cmpPolynomial3D,
                                                                    FrameTuple3DReadOnly icpPositionDesiredFinal, FrameTuple3D<?, ?> icpQuantityDesired)
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
      double generalizedGammaPrime = calculateGeneralizedGammaPrimeOnCMPSegment3D(omega0, time, icpDerivativeOrder, cmpPolynomial3D);
      CommonOps.subtract(generalizedAlphaPrimeMatrix, generalizedBetaPrimeMatrix, generalizedAlphaBetaPrimeMatrix);

      calculateICPQuantity3D(generalizedAlphaBetaPrimeMatrix, generalizedGammaPrime, polynomialCoefficientCombinedVector, icpPositionDesiredFinal, icpQuantityDesired);
   }
   
   public double calculateICPQuantityFromCorrespondingCMPPolynomial1D(double omega0, double time, int icpDerivativeOrder, Trajectory cmpPolynomial, double icpPositionDesiredFinal)
   {      
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
   
      initializeMatrices1D(numberOfCoefficients);
      setPolynomialCoefficientVector1D(polynomialCoefficientVector, cmpPolynomial);
   
      calculateGeneralizedAlphaPrimeOnCMPSegment1D(omega0, time, generalizedAlphaPrimeMatrix, icpDerivativeOrder, cmpPolynomial);
      calculateGeneralizedBetaPrimeOnCMPSegment1D(omega0, time, generalizedBetaPrimeMatrix, icpDerivativeOrder, cmpPolynomial);
      double gammaPrimeDouble = calculateGeneralizedGammaPrimeOnCMPSegment1D(omega0, time, icpDerivativeOrder, cmpPolynomial);
      CommonOps.subtract(generalizedAlphaPrimeMatrix, generalizedBetaPrimeMatrix, generalizedAlphaBetaPrimeMatrix);
   
      return calculateICPQuantity1D(generalizedAlphaBetaPrimeMatrix, gammaPrimeDouble, polynomialCoefficientVector, icpPositionDesiredFinal);
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
    * @param generalizedGammaPrime
    * @param polynomialCoefficientVector
    * @param icpPositionDesiredFinal
    * @return
    */
   public void calculateICPQuantity3D(DenseMatrix64F generalizedAlphaBetaPrimeMatrix, double generalizedGammaPrime,
                                             DenseMatrix64F polynomialCoefficientCombinedVector, FrameTuple3DReadOnly icpPositionDesiredFinal,
                                             FrameTuple3D<?, ?> icpQuantityDesired)
   {
      int numRows = generalizedAlphaBetaPrimeMatrix.getNumRows();
      M1.reshape(numRows, 1);

      icpPositionDesiredFinal.get(M1);
      CommonOps.scale(generalizedGammaPrime, M1);
      CommonOps.multAdd(generalizedAlphaBetaPrimeMatrix, polynomialCoefficientCombinedVector, M1);

      icpQuantityDesired.set(M1);
   }

   public static double calculateICPQuantity1D(DenseMatrix64F generalizedAlphaBetaPrimeMatrix, double generalizedGammaPrime,
                                               DenseMatrix64F polynomialCoefficientVector, double icpPositionDesiredFinal)
   {
      return CommonOps.dot(generalizedAlphaBetaPrimeMatrix, polynomialCoefficientVector) + generalizedGammaPrime * icpPositionDesiredFinal;
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
   public void calculateGeneralizedAlphaPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedAlphaPrimeToPack, int alphaDerivativeOrder,
                                                            FrameTrajectory3D cmpPolynomial3D)
   {
      for(Axis dir : Axis.values)
      {
         Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);
         
         calculateGeneralizedAlphaPrimeOnCMPSegment1D(omega0, time, generalizedAlphaPrimeRow, alphaDerivativeOrder, cmpPolynomial);
         
         MatrixTools.setMatrixBlock(generalizedAlphaPrimeToPack, dir.ordinal(), dir.ordinal() * generalizedAlphaPrimeRow.numCols, generalizedAlphaPrimeRow, 0, 0,
                                    generalizedAlphaPrimeRow.numRows, generalizedAlphaPrimeRow.numCols, 1.0);
      }
   }
   
   public static void calculateGeneralizedAlphaPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedAlphaPrimeRow,
                                                                   int alphaDerivativeOrder, Trajectory cmpPolynomial)
   {
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      
      generalizedAlphaPrimeRow.reshape(1, numberOfCoefficients);
      generalizedAlphaPrimeRow.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         double scalar = Math.pow(omega0, -i);
         CommonOps.addEquals(generalizedAlphaPrimeRow, scalar, cmpPolynomial.getXPowersDerivativeVector(i + alphaDerivativeOrder, time));
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
   public void calculateGeneralizedBetaPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedBetaPrimeToPack,
                                                           int betaDerivativeOrder, FrameTrajectory3D cmpPolynomial3D)
   {                  
      for(Axis dir : Axis.values)
      {
         Trajectory cmpPolynomial = cmpPolynomial3D.getTrajectory(dir);
         
         calculateGeneralizedBetaPrimeOnCMPSegment1D(omega0, time, generalizedBetaPrimeRow, betaDerivativeOrder, cmpPolynomial);
         
         MatrixTools.setMatrixBlock(generalizedBetaPrimeToPack, dir.ordinal(), dir.ordinal() * generalizedBetaPrimeRow.numCols, generalizedBetaPrimeRow, 0, 0,
                                    generalizedBetaPrimeRow.numRows, generalizedBetaPrimeRow.numCols, 1.0);
      }
   }
   
   public static void calculateGeneralizedBetaPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedBetaPrimeRowToPack, int betaDerivativeOrder,
                                                           Trajectory cmpPolynomial)
   {                  
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      
      generalizedBetaPrimeRowToPack.reshape(1, numberOfCoefficients);
      generalizedBetaPrimeRowToPack.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         double scalar = Math.pow(omega0, betaDerivativeOrder-i) * Math.exp(omega0*(time-timeSegmentTotal));
         CommonOps.addEquals(generalizedBetaPrimeRowToPack, scalar, cmpPolynomial.getXPowersDerivativeVector(i, timeSegmentTotal));
      }
   }

   /**
    * Compute the i-th derivative of &gamma;<sub>ICP,&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &gamma;<sup>(i)</sup><sub>ICP,&phi;</sub>(t<sub>&phi;</sub>) = &omega;<sub>0</sub><sup>i</sup> * 
    * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    * 
    * @param gammaDerivativeOrder
    * @param cmpPolynomial3D
    * @param time
    * @return generalizedGammaPrime
    */
   public static double calculateGeneralizedGammaPrimeOnCMPSegment3D(double omega0, double time, int gammaDerivativeOrder, FrameTrajectory3D cmpPolynomial3D)
   {      
      double timeSegmentTotal = cmpPolynomial3D.getFinalTime();
      return Math.pow(omega0, gammaDerivativeOrder) * Math.exp(omega0 * (time - timeSegmentTotal));
   }

   public static double calculateGeneralizedGammaPrimeOnCMPSegment1D(double omega0, double time, int gammaDerivativeOrder, Trajectory cmpPolynomial)
   {
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      return Math.pow(omega0, gammaDerivativeOrder) * Math.exp(omega0 * (time - timeSegmentTotal));
   }

   public static double calculateGeneralizedMatricesPrimeOnCMPSegment1D(double omega0, double time, int derivativeOrder, Trajectory cmpPolynomial,
                                                               DenseMatrix64F generalizedAlphaPrime, DenseMatrix64F generalizedBetaPrime,
                                                               DenseMatrix64F generalizedAlphaBetaPrime)
   {
      calculateGeneralizedAlphaPrimeOnCMPSegment1D(omega0, time, generalizedAlphaPrime, derivativeOrder, cmpPolynomial);
      calculateGeneralizedBetaPrimeOnCMPSegment1D(omega0, time, generalizedBetaPrime, derivativeOrder, cmpPolynomial);
      double generalizedGammaPrime = calculateGeneralizedGammaPrimeOnCMPSegment1D(omega0, time, derivativeOrder, cmpPolynomial);
      CommonOps.subtract(generalizedAlphaPrime, generalizedBetaPrime, generalizedAlphaBetaPrime);

      return generalizedGammaPrime;
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
   }
   
   private void setPolynomialCoefficientVector3D(DenseMatrix64F polynomialCoefficientCombinedVectorToPack, FrameTrajectory3D cmpPolynomial3D)
   {
      int numRows = cmpPolynomial3D.getNumberOfCoefficients();
      int numCols = 1;
      for(Axis dir : Axis.values)
      {
         setPolynomialCoefficientVector1D(polynomialCoefficientVector, cmpPolynomial3D.getTrajectory(dir));
         MatrixTools.setMatrixBlock(polynomialCoefficientCombinedVectorToPack, dir.ordinal() * numRows, 0, polynomialCoefficientVector, 0, 0, numRows, numCols, 1.0);
      }
   }
   
   private static void setPolynomialCoefficientVector1D(DenseMatrix64F polynomialCoefficientVectorToPack, Trajectory cmpPolynomial)
   {
      double[] polynomialCoefficients = cmpPolynomial.getCoefficients();
      
      polynomialCoefficientVectorToPack.setData(polynomialCoefficients);
      polynomialCoefficientVectorToPack.reshape(cmpPolynomial.getNumberOfCoefficients(), 1);
   }
}
