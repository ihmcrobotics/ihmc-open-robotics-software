package us.ihmc.exampleSimulations.singgleLeggedRobot;

import java.util.Random;

public class SinggleLeggedTrajectoryGenerator
{
   private Random random;

   // For sin wave trajectory
   private double sinTime = 0.0;
   private double sinAmplitude;
   private double sinFrequency;
   private double sinOffset;

   private boolean isAmplitudeChange;
   private boolean isFrequencyChange;
   private boolean isOffsetChange;

   // Polynomial coefficient
   private double a0, a1, a2, a3;

   // For bazier polynomial trajectory
   private BazierPolynomial bazierPolynomial;
   double referenceVel;
   double referencePos;
   double referenceTime;
   double dS;
   double stanceTime;
   int numOfGridPoints;
   private MatrixForML GRFbazierCoefficient;
   private MatrixForML zdBazierCoefficient;

   public SinggleLeggedTrajectoryGenerator()
   {
      random = new Random();
      bazierPolynomial = new BazierPolynomial();
      sinAmplitude = 0.1;
      sinFrequency = 1;
      sinOffset = 0.3;
      sinTime = 0.0;
      isAmplitudeChange = true;
      isFrequencyChange = false;
      isOffsetChange = false;
   }

   private MatrixForML getPolynomialTrajectoryCoeff(double currentTime, double desiredTime, double currentPos, double desiredPos)
   {
      double t0 = currentTime;
      double t1 = desiredTime;
      double matA[][] = {{t0 * t0 * t0, t0 * t0, t0, 1}, {t1 * t1 * t1, t1 * t1, t1, 1}, {3 * t0 * t0, 2 * t0, 1, 0}, {3 * t1 * t1, 2 * t1, 1, 0}};

      double matB[][] = {{currentPos}, {desiredPos}, {0}, {0}};
      MatrixForML ret = new MatrixForML(4, 1);
      MatrixForML matrixA = new MatrixForML(4, 4, matA);
      MatrixForML testMatA = new MatrixForML(4, 4, matA);
      MatrixForML matrixB = new MatrixForML(4, 1, matB);
      MatrixForML adjMatA = new MatrixForML(4, 4, matrixA.adjoint(4));

      ret = matrixA.inverse().dot(matrixB);
      return ret;
   }
   
   /*
    * third order polynomial trajectory.
    */
   public double polynomialTrajectory(double currentTime)
   {
      double trajectory;
      trajectory = a3 * Math.pow(currentTime, 3.0) + a2 * Math.pow(currentTime, 2.0) + a1 * Math.pow(currentTime, 1.0) + a0;

      return trajectory;
   }
   
   public void nextPolynomialTrajectory(double currentTime, double desiredTime, double currentPosition, double desiredPosition)
   {
      MatrixForML coeffMat = new MatrixForML(4, 1);
      coeffMat = getPolynomialTrajectoryCoeff(currentTime, desiredTime, currentPosition, desiredPosition);
      System.out.print("next polynomial traj. z0, zd = " + currentPosition);
      System.out.println(" " + desiredPosition);

      a3 = coeffMat.getDoubleValue(0, 0);
      a2 = coeffMat.getDoubleValue(1, 0);
      a1 = coeffMat.getDoubleValue(2, 0);
      a0 = coeffMat.getDoubleValue(3, 0);
   }
   
   public void nextRandomPolynomialTrajectory(double currentTime, double desiredTime, double currentPosition)
   {
      double desiredPosition = 0.2 + 0.3 * random.nextDouble();
      MatrixForML coeffMat = new MatrixForML(4, 1);
      coeffMat = getPolynomialTrajectoryCoeff(currentTime, desiredTime, currentPosition, desiredPosition);
      System.out.print("next polynomial traj. z0, zd = " + currentPosition);
      System.out.println(" " + desiredPosition);

      a3 = coeffMat.getDoubleValue(0, 0);
      a2 = coeffMat.getDoubleValue(1, 0);
      a1 = coeffMat.getDoubleValue(2, 0);
      a0 = coeffMat.getDoubleValue(3, 0);
   }

   public double sinwaveTrajectory()
   {
      double trajectory;

      trajectory = sinAmplitude * Math.sin(2 * Math.PI * sinFrequency * sinTime) + sinOffset;

      sinTime += SinggleLeggedSimulation.DT;

      return trajectory;
   }

   public void nextSinwaveTrajectory(double offset, double amplitude, double frequency)
   {
      double tempRandom = 0.0;
      sinTime = 0;
      sinOffset = offset;
      sinAmplitude = amplitude;
      sinFrequency = frequency;
      
      System.out.print("new sin wave trajectory (offset, amplitude, frequency):");
      System.out.println(sinOffset + ",");
      System.out.print(sinAmplitude + ",");
      System.out.print(sinFrequency);
      
   }
   
   public void nextRandomSinwaveTrajectory()
   {
      double tempRandom = 0.0;
      sinTime = 0;
      sinOffset = 0.2 + 0.2 * random.nextDouble();
    
      while (true)
      {
         tempRandom = random.nextDouble();
         if (((sinOffset + 0.1 * tempRandom + 0.1) > 0.58) || (sinOffset - 0.1 * tempRandom - 0.1) < 0.09)
         {

         }
         else
         {
            sinAmplitude = 0.1 + 0.1 * tempRandom;
            break;
         }
      }
      
      tempRandom = random.nextDouble();
      sinFrequency = 0.1 + 0.4 * tempRandom;
      
      System.out.print("new sin wave trajectory (amp, freq, offset):");
      System.out.print(sinAmplitude + ",");
      System.out.print(sinFrequency + ",");
      System.out.println(sinOffset);
   }

   public void nextDelicateSinwaveTrajectory()
   {
      sinTime = 0;
      if (sinAmplitude * 2 > 0.55 - sinOffset)
      {
         sinAmplitude = 0.05;
         isFrequencyChange = true;
      }

      if (sinFrequency > 1.0)
      {
         sinFrequency = 0.1;
         isOffsetChange = true;
      }

      if (sinOffset > 0.45)
      {
         System.out.print("It's done!");
      }

      if (isAmplitudeChange)
      {
         sinAmplitude += 0.01;
      }

      if (isFrequencyChange)
      {
         sinFrequency += 0.1;
         isFrequencyChange = false;
      }

      if (isOffsetChange)
      {
         sinOffset += 0.01;
         isOffsetChange = false;
      }

      System.out.print("new sin wave trajectory (amp, freq, offset):");
      System.out.print(sinAmplitude + ",");
      System.out.print(sinFrequency + ",");
      System.out.println(sinOffset);
   }

   public double getSinOffset()
   {
      return sinOffset;
   }

   public void setBazierPolynomialTrajectory(double currentVel, double currentPos, double currentTime, double Tst, MatrixForML alpha)
   {
      referencePos = currentPos;
      referenceVel = currentVel;
      referenceTime = currentTime;
      numOfGridPoints = (int) (Tst / SinggleLeggedSimulation.DT);
      stanceTime = Tst;
      dS = SinggleLeggedSimulation.DT / Tst;
      double m = 5.0;
      double g = 9.81;
      GRFbazierCoefficient = new MatrixForML(1, alpha.getCol());
      GRFbazierCoefficient = alpha;

      zdBazierCoefficient = new MatrixForML(1, alpha.getCol() + 1);
      zdBazierCoefficient.set(0, 0, 0.0);
      zdBazierCoefficient.set(0, 1, zdBazierCoefficient.getDoubleValue(0, 0) + Tst / (alpha.getCol()) * (-g));
      zdBazierCoefficient.set(0, 2, zdBazierCoefficient.getDoubleValue(0, 1) + Tst / (alpha.getCol()) * (alpha.getDoubleValue(0, 1) / m - g));
      zdBazierCoefficient.set(0, 3, zdBazierCoefficient.getDoubleValue(0, 2) + Tst / (alpha.getCol()) * (alpha.getDoubleValue(0, 2) / m - g));
      zdBazierCoefficient.set(0, 4, zdBazierCoefficient.getDoubleValue(0, 3) + Tst / (alpha.getCol()) * (alpha.getDoubleValue(0, 3) / m - g));
      zdBazierCoefficient.set(0, 5, zdBazierCoefficient.getDoubleValue(0, 4) + Tst / (alpha.getCol()) * (alpha.getDoubleValue(0, 4) / m - g));
      zdBazierCoefficient.set(0, 6, zdBazierCoefficient.getDoubleValue(0, 5) + Tst / (alpha.getCol()) * (-g));
      zdBazierCoefficient.printMat();

      System.out.println(dS);
      //      0   -0.4496   -1.4516    4.8556   -5.0266    1.4446    0.9950

   }

   public double reconstructedBazierPolynomial(double currentTime)
   {
      double ret;
      double temp;
      double normalizedTime = (currentTime - referenceTime) / stanceTime;
      //      zdBazierCoefficient.printMat();
      System.out.println(normalizedTime);
      temp = bazierPolynomial.dBazierPolynomial(normalizedTime, stanceTime, zdBazierCoefficient);

      ret = referenceVel + dS * temp;
      referenceVel = ret;
      //      referencePos = trajectory;

      return ret;
   }

   public double bazierPolynomialTrajectory(double currentTime)
   {
      double ret;
      
      ret = referencePos + reconstructedBazierPolynomial(currentTime) * dS;
      referencePos = ret;

      System.out.println("desired z position : " + ret);
      return ret;
   }
   
   public MatrixForML bazierPolynomialTrajectoryTorque(double currentTime)
   {
      MatrixForML ret = new MatrixForML(1,2);
      double torque1, torque2;
      double z = bazierPolynomialTrajectory(currentTime);
      double q1 = Math.acos(z / 0.6); 
      double q2 = -2 * q1;
      double dz_q1, dz_q2;
      double normalizedTime = (currentTime - referenceTime) / stanceTime;
      double GRF = bazierPolynomial.BazierPolynomialFunction(normalizedTime, GRFbazierCoefficient);
      
       dz_q1 = - 0.3*Math.sin(q1) - 0.3*Math.sin(q1+q2);
       dz_q2 = - 0.3*Math.sin(q1+q2);
//      dz_q1 = - 0.6*Math.sin(q1);
//      dz_q2 = 0.3*Math.sin(q1);
      
      torque1 = dz_q1*GRF;
      torque2 = dz_q2*GRF;
      
      ret.set(0, 0, torque1);
      ret.set(0, 1, torque2);
      

      System.out.println("torque1 : " + torque1);
      System.out.println("torque2 : " + torque2);
      return ret;
   }

}
