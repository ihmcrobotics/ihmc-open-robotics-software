package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoTrajectory;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FrameTuple;

/**
 * @author Tim Seyde
 */

public class SmoothCapturePointTools extends CapturePointTools
{
   // private static final ThreadLocal<DenseMatrix64F> dummyName = new ThreadLocal<>();
   
   private static final int defaultSize = 1000;
   
   private static final DenseMatrix64F tPowersDerivativeVector = new DenseMatrix64F(defaultSize, 1);
   private static final DenseMatrix64F tPowersDerivativeVectorTranspose = new DenseMatrix64F(defaultSize, 1);
   
   private static final DenseMatrix64F generalizedAlphaPrimeRow = new DenseMatrix64F(1, defaultSize);
   private static final DenseMatrix64F generalizedBetaPrimeRow = new DenseMatrix64F(1, defaultSize);
   
   private static final DenseMatrix64F polynomialCoefficientCombinedVector= new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F polynomialCoefficientVector= new DenseMatrix64F(defaultSize, 1);
   
   private static final DenseMatrix64F generalizedAlphaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F generalizedBetaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F generalizedGammaPrimeMatrix = new DenseMatrix64F(1, 1);
   private static final DenseMatrix64F generalizedAlphaBetaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   
   private static final DenseMatrix64F M1 = new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F M2 = new DenseMatrix64F(defaultSize, defaultSize);
   
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
   public static void calculateICPQuantityFromCorrespondingCMPPolynomial3D(double omega0, double time, int icpDerivativeOrder, 
                                                                           YoFrameTrajectory3D cmpPolynomial3D, 
                                                                           FrameTuple<?, ?> icpPositionDesiredFinal, 
                                                                           FrameTuple<?, ?> icpQuantityDesired)
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
      
//      PrintTools.debug("A: " + generalizedAlphaPrimeMatrix.toString());
//      PrintTools.debug("B: " + generalizedBetaPrimeMatrix.toString());
//      PrintTools.debug("C: " + generalizedGammaPrimeMatrix.toString());
//      PrintTools.debug("AB: " + generalizedAlphaBetaPrimeMatrix.toString());
//      PrintTools.debug("P: " + polynomialCoefficientCombinedVector.toString());
   }
   
   public static double calculateICPQuantityFromCorrespondingCMPPolynomial1D(double omega0, double time, int icpDerivativeOrder, YoTrajectory cmpPolynomial, double icpPositionDesiredFinal)
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
    * (&alpha;<sup>(i)</sup><sub>&phi;</sub>(t<sub>&phi;</sub>)
    *  - &beta;<sup>(i)</sup><sub>&phi;</sub>(t<sub>&phi;</sub>)) * p<sub>&phi;</sub>
    *  + &gamma;<sup>(i)</sup><sub>&phi;</sub>(t<sub>&phi;</sub>) * &xi;<sub>ref,&phi;</sub>(T<sub>&phi;</sub>)
    * 
    * @param generalizedAlphaBetaPrimeMatrix
    * @param generalizedGammaPrimeMatrix
    * @param polynomialCoefficientVector
    * @param icpPositionDesiredFinal
    * @return
    */
   public static void calculateICPQuantity3D(DenseMatrix64F generalizedAlphaBetaPrimeMatrix, DenseMatrix64F generalizedGammaPrimeMatrix,
                                             DenseMatrix64F polynomialCoefficientCombinedVector, FrameTuple<?, ?> icpPositionDesiredFinal,
                                             FrameTuple<?, ?> icpQuantityDesired)
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
   
   public static double calculateICPQuantity1D(DenseMatrix64F generalizedAlphaBetaPrimeMatrix, DenseMatrix64F generalizedGammaPrimeMatrix,
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
    * Compute the i-th derivative of &alpha;<sub>&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &alpha;<sup>(i)</sup><sub>&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>j=0</sub><sup>n</sup> &omega;<sub>0</sub><sup>-j</sup> *
    * t<sup>(j+i)<sup>T</sup></sup> (t<sub>&phi;</sub>)
    * 
    * @param generalizedAlphaPrime
    * @param alphaDerivativeOrder
    * @param cmpPolynomial
    * @param time
    */
   public static void calculateGeneralizedAlphaPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedAlphaPrime, 
                                                                   int alphaDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);
         
         calculateGeneralizedAlphaPrimeOnCMPSegment1D(omega0, time, generalizedAlphaPrimeRow, alphaDerivativeOrder, cmpPolynomial);
         
         CommonOps.insert(generalizedAlphaPrimeRow, generalizedAlphaPrime, dir.ordinal(), dir.ordinal() * generalizedAlphaPrimeRow.numCols);
      }
   }
   
   public static void calculateGeneralizedAlphaPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedAlphaPrimeRow,
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
    * Compute the i-th derivative of &beta;<sub>&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &beta;<sup>(i)</sup><sub>&phi;</sub>(t<sub>&phi;</sub>) = &Sigma;<sub>j=0</sub><sup>n</sup> &omega;<sub>0</sub><sup>-(j-i)</sup> *
    * t<sup>(j)<sup>T</sup></sup> (T<sub>&phi;</sub>) * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    * 
    * @param generalizedBetaPrime
    * @param betaDerivativeOrder
    * @param cmpPolynomial
    * @param time
    */
   public static void calculateGeneralizedBetaPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedBetaPrime, 
                                                                  int betaDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {                  
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);
         
         calculateGeneralizedBetaPrimeOnCMPSegment1D(omega0, time, generalizedBetaPrimeRow, betaDerivativeOrder, cmpPolynomial);
         
         CommonOps.insert(generalizedBetaPrimeRow, generalizedBetaPrime, dir.ordinal(), dir.ordinal() * generalizedBetaPrimeRow.numCols);
      }
   }
   
   public static void calculateGeneralizedBetaPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedBetaPrimeRow, 
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
    * Compute the i-th derivative of &gamma;<sub>&phi;</sub> at time t<sub>&phi;</sub>:
    * <P>
    * &gamma;<sup>(i)</sup><sub>&phi;</sub>(t<sub>&phi;</sub>) = &omega;<sub>0</sub><sup>i</sup> * 
    * e<sup>&omega;<sub>0</sub>(t<sub>&phi;</sub>-T<sub>&phi;</sub>)</sup>
    * 
    * @param generalizedGammaPrime
    * @param gammaDerivativeOrder
    * @param cmpPolynomial3D
    * @param time
    */
   public static void calculateGeneralizedGammaPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedGammaPrime, 
                                                                   int gammaDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {      
      double timeSegmentTotal = cmpPolynomial3D.getFinalTime();
      double ddGamaPrimeValue = Math.pow(omega0, gammaDerivativeOrder)*Math.exp(omega0 * (time - timeSegmentTotal));
      generalizedGammaPrime.set(0, 0, ddGamaPrimeValue);
   }
   
   public static void calculateGeneralizedGammaPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedGammaPrime, 
                                                                   int gammaDerivativeOrder, YoTrajectory cmpPolynomial)
   {      
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      double ddGamaPrimeValue = Math.pow(omega0, gammaDerivativeOrder)*Math.exp(omega0 * (time - timeSegmentTotal));
      generalizedGammaPrime.set(0, 0, ddGamaPrimeValue);
   }
   
   public static void initializeMatrices3D(int numberOfCoefficients)
   {
      initializeMatrices(3, numberOfCoefficients);
   }
   
   public static void initializeMatrices1D(int numberOfCoefficients)
   {
      initializeMatrices(1, numberOfCoefficients);
   }
   
   public static void initializeMatrices(int dimension, int numberOfCoefficients)
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
   
   public static void setPolynomialCoefficientVector3D(DenseMatrix64F polynomialCoefficientCombinedVector, YoFrameTrajectory3D cmpPolynomial3D)
   {
      for(Direction dir : Direction.values())
      {
         setPolynomialCoefficientVector1D(polynomialCoefficientVector, cmpPolynomial3D.getYoTrajectory(dir));
         
         CommonOps.insert(polynomialCoefficientVector, polynomialCoefficientCombinedVector, dir.ordinal() * polynomialCoefficientVector.numRows, 0);
      }
   }
   
   public static void setPolynomialCoefficientVector1D(DenseMatrix64F polynomialCoefficientVector, YoTrajectory cmpPolynomial)
   {
      double[] polynomialCoefficients = cmpPolynomial.getCoefficients();
      
      polynomialCoefficientVector.reshape(cmpPolynomial.getNumberOfCoefficients(), 1);
      polynomialCoefficientVector.zero();
      
      polynomialCoefficientVector.setData(polynomialCoefficients);
   }
   
//   // MATRIX FORMULATION
//   // ICP = (M1)^(-1) * M2 * P = M3 * P
//   private void calculateICPFromCorrespondingCMPPolynomialMatrix(DenseMatrix64F icpVector, DenseMatrix64F identityMatrix, DenseMatrix64F gammaPrimeMatrix, DenseMatrix64F backwardIterationMatrix,
//                                                                 DenseMatrix64F alphaBetaPrimeMatrix, DenseMatrix64F terminalConstraintMatrix, DenseMatrix64F tPowersVector, DenseMatrix64F polynomialCoefficientVector)
//   {
//      M1.reshape(identityMatrix.getNumRows(), identityMatrix.getNumCols());
//      M1.zero();
//       
//      M2.reshape(alphaBetaPrimeMatrix.getNumRows(), alphaBetaPrimeMatrix.getNumCols());
//      M2.zero();
//       
//      M3.reshape(identityMatrix.getNumRows(), alphaBetaPrimeMatrix.getNumCols());
//      M3.zero();
//       
//      CommonOps.mult(gammaPrimeMatrix, backwardIterationMatrix, M1);
//      CommonOps.subtract(identityMatrix, M1, M1);
//       
//      CommonOps.mult(terminalConstraintMatrix, tPowersVector, M2);
//      CommonOps.mult(gammaPrimeMatrix, M2, M2);
//      CommonOps.addEquals(M2, alphaBetaPrimeMatrix);
//       
//      CommonOps.invert(M1, M3);
//      CommonOps.mult(M3, M2, M3);
//       
//      CommonOps.mult(M3, polynomialCoefficientVector, icpVector);
//   }
}
