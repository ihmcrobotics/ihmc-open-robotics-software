package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoTrajectory;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FrameTuple;

public class CapturePointMatrixTools extends CapturePointTools
{
   private final static int defaultSize = 1000;
   
   private static final DenseMatrix64F tPowersDerivativeVector = new DenseMatrix64F(defaultSize, 1);
   private static final DenseMatrix64F tPowersDerivativeVectorTranspose = new DenseMatrix64F(defaultSize, 1);
   
   private static final DenseMatrix64F generalizedAlphaPrimeRow = new DenseMatrix64F(1, defaultSize);
   private static final DenseMatrix64F generalizedBetaPrimeRow = new DenseMatrix64F(1, defaultSize);
   
   private static final DenseMatrix64F polynomialCoefficientVector = new DenseMatrix64F(defaultSize, defaultSize);
   
   private static final DenseMatrix64F generalizedAlphaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F generalizedBetaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F generalizedGammaPrimeMatrix = new DenseMatrix64F(1, 1);
   private static final DenseMatrix64F generalizedAlphaBetaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   
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
   public static void calculateICPQuantityFromCorrespondingCMPPolynomialScalar(double omega0, double time, int icpDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D, FrameTuple<?, ?> icpPositionDesiredFinal, FrameTuple<?, ?> icpPositionDesired)
   {        
//      int numberOfCoefficients = cmpPolynomial3D.getYoTrajectoryX().getNumberOfCoefficients(); //TODO: proper way to do it? x, y and z always same number of coefficients?
//
//      double[] polynomialCoefficients = cmpPolynomial.getCoefficients();
//      polynomialCoefficientVector.reshape(numberOfCoefficients, 1);
//      polynomialCoefficientVector.setData(polynomialCoefficients);
//      
//      generalizedAlphaPrimeMatrix.reshape(3, numberOfCoefficients);
//      generalizedBetaPrimeMatrix.reshape(3, numberOfCoefficients);
//      generalizedAlphaBetaPrimeMatrix.reshape(3, numberOfCoefficients);
//      generalizedGammaPrimeMatrix.reshape(1, 1);
//
//      generalizedAlphaPrimeMatrix.zero();
//      generalizedBetaPrimeMatrix.zero();
//      generalizedAlphaBetaPrimeMatrix.zero();
//      generalizedGammaPrimeMatrix.zero();
//
//      calculateGeneralizedAlphaPrimeOnCMPSegment(omega0, generalizedAlphaPrimeMatrix, icpDerivativeOrder, cmpPolynomial, time);
//      calculateGeneralizedBetaPrimeOnCMPSegment(generalizedBetaPrimeMatrix, icpDerivativeOrder, cmpPolynomial, time);
//      calculateGeneralizedGammaPrimeOnCMPSegment(generalizedGammaPrimeMatrix, icpDerivativeOrder, cmpPolynomial, time);
//      CommonOps.subtract(generalizedAlphaPrimeMatrix, generalizedBetaPrimeMatrix, generalizedAlphaBetaPrimeMatrix);
//      
////      PrintTools.debug("A: " +generalizedAlphaPrimeMatrix.toString());
////      PrintTools.debug("B: " +generalizedBetaPrimeMatrix.toString());
////      PrintTools.debug("C: " +generalizedGammaPrimeMatrix.toString());
////      PrintTools.debug("AB: " + generalizedAlphaBetaPrimeMatrix.toString());
////      PrintTools.debug("P: " + polynomialCoefficientVector.toString());
////      PrintTools.debug("ICP: " + icpPositionDesiredFinal);
////      PrintTools.debug("");
//      
//      return calculateICPQuantityScalar(generalizedAlphaBetaPrimeMatrix, generalizedGammaPrimeMatrix, polynomialCoefficientVector, icpPositionDesiredFinal);
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
   public static void calculateGeneralizedAlphaPrimeOnCMPSegment(double omega0, double time, DenseMatrix64F generalizedAlphaPrime, int alphaDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);
         
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
         
         CommonOps.insert(generalizedAlphaPrimeRow, generalizedAlphaPrime, dir.ordinal(), 0);
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
   public static void calculateGeneralizedBetaPrimeOnCMPSegment(double omega0, double time, DenseMatrix64F generalizedBetaPrime, int betaDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {                  
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);
         
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
         
         CommonOps.insert(generalizedBetaPrimeRow, generalizedBetaPrime, dir.ordinal(), 0);
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
    * @param cmpPolynomial
    * @param time
    */
   public static void calculateGeneralizedGammaPrimeOnCMPSegment(double omega0, double time, DenseMatrix64F generalizedGammaPrime, int gammaDerivativeOrder, YoFrameTrajectory3D cmpPolynomial)
   {      
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      double ddGamaPrimeValue = Math.pow(omega0, gammaDerivativeOrder)*Math.exp(omega0 * (time - timeSegmentTotal));
      generalizedGammaPrime.set(0, 0, ddGamaPrimeValue);
   } 
}
