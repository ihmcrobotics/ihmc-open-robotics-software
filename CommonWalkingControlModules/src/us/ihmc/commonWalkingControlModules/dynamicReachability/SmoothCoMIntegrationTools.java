package us.ihmc.commonWalkingControlModules.dynamicReachability;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoTrajectory;

public class SmoothCoMIntegrationTools extends CoMIntegrationTools
{
   private static final int defaultSize = 1000;
   
   private static final DenseMatrix64F tPowersDerivativeVector = new DenseMatrix64F(defaultSize, 1);
   private static final DenseMatrix64F tPowersDerivativeVectorTranspose = new DenseMatrix64F(defaultSize, 1);
   
   private static final DenseMatrix64F generalizedAlphaPrimeRow = new DenseMatrix64F(1, defaultSize);
   private static final DenseMatrix64F generalizedBetaPrimeRow = new DenseMatrix64F(1, defaultSize);
   private static final DenseMatrix64F generalizedDeltaPrimeRow = new DenseMatrix64F(1, defaultSize);
   
   private static final DenseMatrix64F polynomialCoefficientCombinedVector= new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F polynomialCoefficientVector= new DenseMatrix64F(defaultSize, 1);
   
   private static final DenseMatrix64F generalizedAlphaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F generalizedBetaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F generalizedGammaPrimeMatrix = new DenseMatrix64F(1, 1);
   private static final DenseMatrix64F generalizedAlphaBetaPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   
   private static final DenseMatrix64F M1 = new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F M2 = new DenseMatrix64F(defaultSize, defaultSize);
   
   public static void calculateGeneralizedAlphaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedAlphaCoMPrimeRow,
                                                                      int alphaCoMDerivativeOrder, YoTrajectory cmpPolynomial)
   {
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      
      generalizedAlphaCoMPrimeRow.reshape(1, numberOfCoefficients);
      generalizedAlphaCoMPrimeRow.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         for(int j = 0; j < numberOfCoefficients - i; i++)
         {
            tPowersDerivativeVector.zero();
            tPowersDerivativeVectorTranspose.zero();
            
            tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i + j + alphaCoMDerivativeOrder, time));
            CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
            
            double scalar = Math.pow(-1.0, j) * Math.pow(omega0, -(i + j));
            CommonOps.addEquals(generalizedAlphaCoMPrimeRow, scalar, tPowersDerivativeVectorTranspose);            
         }
      }
   }
   
   public static void calculateGeneralizedBetaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedBetaCoMPrimeRow, 
                                                                     int betaCoMDerivativeOrder, YoTrajectory cmpPolynomial)
   {                  
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      
      generalizedBetaCoMPrimeRow.reshape(1, numberOfCoefficients);
      generalizedBetaCoMPrimeRow.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         for(int j = 0; j < numberOfCoefficients - i; i++)
         {
            tPowersDerivativeVector.zero();
            tPowersDerivativeVectorTranspose.zero();
            
            tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i + j, timeSegmentTotal));
            CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
            
            double scalar =  Math.pow(-1.0, j + betaCoMDerivativeOrder) * Math.pow(omega0,  -(i + j) + betaCoMDerivativeOrder) * Math.exp(omega0*(timeSegmentTotal-time));
            CommonOps.addEquals(generalizedBetaCoMPrimeRow, scalar, tPowersDerivativeVectorTranspose);            
         }
      }
   }
   
   public static void calculateGeneralizedGammaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedGammaCoMPrime, 
                                                                   int gammaCoMDerivativeOrder, YoTrajectory cmpPolynomial)
   {      
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      double ddGamaPrimeValue = Math.pow(-1.0, gammaCoMDerivativeOrder) * Math.pow(omega0, gammaCoMDerivativeOrder) * Math.exp(omega0 * (timeSegmentTotal - time));
      generalizedGammaCoMPrime.set(0, 0, ddGamaPrimeValue);
   }
   
   public static void calculateGeneralizedDeltaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedDeltaCoMPrimeRow, 
                                                                      int deltaCoMDerivativeOrder, YoTrajectory cmpPolynomial)
   {                  
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      
      generalizedDeltaCoMPrimeRow.reshape(1, numberOfCoefficients);
      generalizedDeltaCoMPrimeRow.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.zero();
         tPowersDerivativeVectorTranspose.zero();
         
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i + deltaCoMDerivativeOrder, timeSegmentTotal));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
                  
         double scalar = Math.pow(omega0, -i);
         CommonOps.addEquals(generalizedBetaPrimeRow, scalar, tPowersDerivativeVectorTranspose);
      }
   }
}
