package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration;

import static us.ihmc.robotics.Assert.*;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.robotics.math.trajectories.YoTrajectory;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ReferenceICPTrajectoryGeneratorTest
{
   private static double EPSILON = 1e-6;
   double omega0 = 3.4;
   
   String namePrefix = "ReferenceICPTrajectoryGeneratorTest";
   
   @Test
   public void testCalculateICPOnSegmentScalar()
   {
      //linear polynomial: y(x) = a0 + a1*x
      YoRegistry registry = new YoRegistry(namePrefix);
      int numberOfCoefficients = 2;
      YoTrajectory linear = new YoTrajectory(namePrefix + "Linear", numberOfCoefficients, registry);
      
      int numTrials = 9;
      for(int i = 0; i < numTrials; i++)
      {
         double scaleX0 = 1.0 / Math.random(), scaleXf = 1.0 / Math.random();
         double scaleY0 = 1.0 / Math.random(), scaleYf = 1.0 / Math.random();
         
         double x0 = 0.0, xf = x0 + Math.random() * scaleXf;
         double y0 = Math.signum(Math.random()) * Math.random() * scaleY0, yf = Math.signum(Math.random()) * Math.random() * scaleYf;
               
         linear.setLinear(x0, xf, y0, yf);
         
         double x = x0 + Math.random() * (xf - x0);
         
         DMatrixRMaj alphaPrime = new DMatrixRMaj(1, numberOfCoefficients);
         DMatrixRMaj betaPrime = new DMatrixRMaj(1, numberOfCoefficients);
         DMatrixRMaj gammaPrime = new DMatrixRMaj(1, 1);
         DMatrixRMaj alphaBetaPrime = new DMatrixRMaj(1, numberOfCoefficients);
         
         calculateGeneralizedAlphaPrimeOnCMPSegment(alphaPrime, 0, linear, x, omega0);
         calculateGeneralizedBetaPrimeOnCMPSegment(betaPrime, 0, linear, x, omega0);
         calculateGeneralizedGammaPrimeOnCMPSegment(gammaPrime, 0, linear, x, omega0);
         
         CommonOps_DDRM.subtract(alphaPrime, betaPrime, alphaBetaPrime);
         
         DMatrixRMaj polynomialCoefficientVector =  linear.getCoefficientsVector();
         
         linear.compute(linear.getFinalTime());
         double icpPositionDesiredFinal = linear.getPosition();
         
         double icpPolynomial = calculateICPQuantityScalar(alphaBetaPrime, gammaPrime, polynomialCoefficientVector, icpPositionDesiredFinal);
         double icpManual = calculateICPLinearManual(linear, icpPositionDesiredFinal, x, omega0);
         
//         PrintTools.debug("(x0, xf, y0, yf) = (" + x0 + ", " + xf + ", " + y0 + ", " + yf + ")");
//         PrintTools.debug("Coefficients = " + Arrays.toString(linear.getCoefficients()));
//         PrintTools.debug("x = " + x);
//         
//         PrintTools.debug("ICP: " + icpPolynomial + " = " + icpManual);
         assertEquals(icpManual, icpPolynomial, EPSILON);
      }
   }
   
   @Test
   public void testMatricesPrimeLinear()
   {
      //linear polynomial: y(x) = a0 + a1*x
      YoRegistry registry = new YoRegistry(namePrefix);
      int numberOfCoefficients = 2;
      YoTrajectory linear = new YoTrajectory(namePrefix + "Linear", numberOfCoefficients, registry);
      
      int numTrials = 9;
      for(int i = 0; i < numTrials; i++)
      {
         double scaleX0 = 1.0 / Math.random(), scaleXf = 1.0 / Math.random();
         double scaleY0 = 1.0 / Math.random(), scaleYf = 1.0 / Math.random();
         
         double x0 = 0.0, xf = x0 + Math.random() * scaleXf;
         double y0 = Math.signum(Math.random()) * Math.random() * scaleY0, yf = Math.signum(Math.random()) * Math.random() * scaleYf;
               
         linear.setLinear(x0, xf, y0, yf);
         
         double x = x0 + Math.random() * (xf - x0);
      
         DMatrixRMaj alphaPrimeAutomatic = new DMatrixRMaj(1, numberOfCoefficients);
         DMatrixRMaj alphaPrimeManual = new DMatrixRMaj(1, numberOfCoefficients);
      
         calculateGeneralizedAlphaPrimeOnCMPSegment(alphaPrimeAutomatic, 0, linear, x, omega0);
         calculateAlphaPrimeLinear(alphaPrimeManual, x, xf, omega0);
         
         for(int j = 0; j < numberOfCoefficients; j++)
         {
            assertEquals(alphaPrimeAutomatic.get(j), alphaPrimeManual.get(j), EPSILON);
         }
         
         DMatrixRMaj betaPrimeAutomatic = new DMatrixRMaj(1, numberOfCoefficients);
         DMatrixRMaj betaPrimeManual = new DMatrixRMaj(1, numberOfCoefficients);
      
         calculateGeneralizedBetaPrimeOnCMPSegment(betaPrimeAutomatic, 0, linear, x, omega0);
         calculateBetaPrimeLinear(betaPrimeManual, x, xf, omega0);
         
         for(int j = 0; j < numberOfCoefficients; j++)
         {
            assertEquals(betaPrimeAutomatic.get(j), betaPrimeManual.get(j), EPSILON);
         }
         
         DMatrixRMaj gammaPrimeAutomatic = new DMatrixRMaj(1, 1);
         DMatrixRMaj gammaPrimeManual = new DMatrixRMaj(1, 1);
      
         calculateGeneralizedGammaPrimeOnCMPSegment(gammaPrimeAutomatic, 0, linear, x, omega0);
         calculateGammaPrimeLinear(gammaPrimeManual, x, xf, omega0);
         
         assertEquals(gammaPrimeAutomatic.get(0), gammaPrimeManual.get(0), EPSILON);
      }
   }
   
   public double calculateICPLinearManual(YoTrajectory linear, double icpPositionDesiredFinal, double t, double omega0)
   {
      double icpManual = 0.0;
      
      linear.compute(linear.getInitialTime());
      double cmpRefInit = linear.getPosition();
      
      linear.compute(linear.getFinalTime());
      double cmpRefFinal = linear.getPosition();
      
      double T = linear.getFinalTime();
      
      double sigmat = calculateSigmaLinear(t, T, omega0);
      double sigmaT = calculateSigmaLinear(T, T, omega0);
      
      icpManual = (1 - sigmat - Math.exp(omega0*(t-T)) * (1 - sigmaT)) * cmpRefInit
                  + (sigmat - Math.exp(omega0*(t-T)) * sigmaT) * cmpRefFinal
                  + Math.exp(omega0*(t-T)) * icpPositionDesiredFinal;
      
      return icpManual;
   }
   
   public double calculateSigmaLinear(double t, double T, double omega0)
   {
      double sigmaLinear = t/T + 1.0/omega0 * 1/T;
      return sigmaLinear;
   }
   
   private void calculateGeneralizedAlphaPrimeOnCMPSegment(DMatrixRMaj generalizedAlphaPrime, int alphaDerivativeOrder, YoTrajectory cmpPolynomial, double time, double omega0)
   {
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      
      DMatrixRMaj tPowersDerivativeVector = new DMatrixRMaj(numberOfCoefficients, 1);
      DMatrixRMaj tPowersDerivativeVectorTranspose = new DMatrixRMaj(1, numberOfCoefficients);
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.zero();
         tPowersDerivativeVectorTranspose.zero();
         
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i + alphaDerivativeOrder, time));
         CommonOps_DDRM.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
                  
         double scalar = Math.pow(omega0, -i);
         CommonOps_DDRM.addEquals(generalizedAlphaPrime, scalar, tPowersDerivativeVectorTranspose);
      }
   }
   
   private void calculateGeneralizedBetaPrimeOnCMPSegment(DMatrixRMaj generalizedBetaPrime, int betaDerivativeOrder, YoTrajectory cmpPolynomial, double time, double omega0)
   {            
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      double timeSegmentTotal = cmpPolynomial.getFinalTime() - cmpPolynomial.getInitialTime();
      
      DMatrixRMaj tPowersDerivativeVector = new DMatrixRMaj(numberOfCoefficients, 1);
      DMatrixRMaj tPowersDerivativeVectorTranspose = new DMatrixRMaj(1, numberOfCoefficients);

      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.zero();
         tPowersDerivativeVectorTranspose.zero();

         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i, timeSegmentTotal));
         CommonOps_DDRM.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
                  
         double scalar = Math.pow(omega0, betaDerivativeOrder-i) * Math.exp(omega0*(time-timeSegmentTotal));
         CommonOps_DDRM.addEquals(generalizedBetaPrime, scalar, tPowersDerivativeVectorTranspose);
      }
   }
   
   private void calculateGeneralizedGammaPrimeOnCMPSegment(DMatrixRMaj generalizedGammaPrime, int gammaDerivativeOrder, YoTrajectory cmpPolynomial, double time, double omega0)
   {      
      double timeSegmentTotal = cmpPolynomial.getFinalTime() - cmpPolynomial.getInitialTime();
      
      double ddGamaPrimeValue = Math.pow(omega0, gammaDerivativeOrder)*Math.exp(omega0 * (time - timeSegmentTotal));
      generalizedGammaPrime.set(0, 0, ddGamaPrimeValue);
   } 
   
   private double calculateICPQuantityScalar(DMatrixRMaj generalizedAlphaBetaPrimeMatrix, DMatrixRMaj generalizedGammaPrimeMatrix,
                                             DMatrixRMaj polynomialCoefficientVector, double icpPositionDesiredFinal)
   {
      DMatrixRMaj M1 = new DMatrixRMaj(generalizedAlphaBetaPrimeMatrix.getNumRows(), polynomialCoefficientVector.getNumCols());
      M1.zero();

      CommonOps_DDRM.mult(generalizedAlphaBetaPrimeMatrix, polynomialCoefficientVector, M1);

      DMatrixRMaj M2 = new DMatrixRMaj(generalizedGammaPrimeMatrix);
      CommonOps_DDRM.scale(icpPositionDesiredFinal, M2);
      
      CommonOps_DDRM.addEquals(M1, M2);
      
      return M1.get(0, 0);
   }
   
   public void calculateAlphaPrimeLinear(DMatrixRMaj alphaPrimeLinear, double time, double timeTotal, double omega0)
   {
      alphaPrimeLinear.set(0, 0, 1);
      alphaPrimeLinear.set(0, 1, time + 1.0/omega0);
   }
   
   public void calculateBetaPrimeLinear(DMatrixRMaj betaPrimeLinear, double time, double timeTotal, double omega0)
   {
      betaPrimeLinear.set(0, 0, Math.exp(omega0 * (time - timeTotal))*1);
      betaPrimeLinear.set(0, 1, Math.exp(omega0 * (time - timeTotal))*(timeTotal + 1.0/omega0));
   }
   
   public void calculateGammaPrimeLinear(DMatrixRMaj gammaPrimeLinear, double time, double timeTotal, double omega0)
   {
      gammaPrimeLinear.set(0, 0, Math.exp(omega0 * (time - timeTotal)));
   }
}
