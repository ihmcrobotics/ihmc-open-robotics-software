package us.ihmc.steppr.hardware.state;

public class StepprAnkleInterpolator implements StepprAnkleAngleCalculator
{
   //      private static final double px[] = { 0.000000000009934, -1.269023940640927, 1.269023940648429, -0.020516021439826, 0.020516020934548, 0.000000000508670,
   //            -0.115495045972189, 0.523312234721911, -0.523312234476478, 0.115495045733103 }; //parameters for cubic mapping of pulley angles to ankle x
   //   
   //      private static final double py[] = { 0.000022001170581, 0.463732921796384, 0.463732921942751, -0.048394600966524, -0.048394600971605, 0.111755077411411,
   //            -0.075022107557441, 0.057936364110976, 0.057936369906301, -0.075022109478369 }; //parameters for cubic mapping of pulley angles to ankle x
   //   
   //      private static final double pJitX[] = { -1.272997428093503, -0.036001688534274, -0.001270575344824, -0.411410632841345, -0.628599882888178,
   //            1.233612286706826, 0.128100494751386, -0.209457296600427, -0.014034155341702, 0.080071535526900 }; //parameters for cubic mapping of pulley angles to jacobian inverse transpose element 1,1
   //   
   //      private static final double pJitY[] = { 0.465442714729626, -0.079404497795577, 0.091522713969963, -0.239129828847536, 0.040327663912797, 0.141924039046489,
   //            -0.130148705434543, 0.342409812822250, -0.315199100250677, 0.111764660552245 }; //parameters for cubic mapping of pulley angles to jacobian inverse transpose element 2,1

   private static final double px[] = { 0.000000000011833, 1.269023941053808, -1.269023941048548, 0.020516020869627, -0.020516021471959, 0.000000000606667,
         0.115495040111334, -0.523312217421704, 0.523312217621650, -0.115495040305941 };

   private static final double py[] = { 0.000022001174462, 0.463732921959109, 0.463732921783293, -0.048394601071517, -0.048394601065684, 0.111755077611979,
         -0.075022109801143, 0.057936370829754, 0.057936363252024, -0.075022107299457 };

   private static final double pJitX[] = { 1.272997427777619, 0.036001701491556, 0.001270562468774, 0.411410640321290, 0.628599891388498, -1.233612302811653,
         -0.128100680179965, 0.209457856510320, 0.014033597275913, -0.080071351885635 };

   private static final double pJitY[] = { 0.465442714987602, -0.079404498198185, 0.091522714637596, -0.239129834635615, 0.040327656123728, 0.141924052147264,
         -0.130148712431276, 0.342409832258836, -0.315199115178924, 0.111764662233772 };

   private static final double pM1[] = { 0.001637333497514, 2.363024415727514, 6.448426531047804, 0.209647991746190, -0.132578850323121, -0.102263276506052,
         -0.223803048871049, 0.764689039287297, 0.459191580065332, 0.349362854084994 };
   
   private static final double pM2[] = { 0.001637333371246, -2.363024413970044, 6.448426532362931, 0.209647993650533, -0.132578850320880, 0.102263276373644,
         0.223803036060187, 0.764689036580238, -0.459191581057666, 0.349362851648649 };

   private static final int N = 6; // Cable reduction of pulleys
   // this returns the value a 2D cubic polynomial with given parameters p
   // the m1,m2 inputs are the motor1 and 2 pulley angles
   private static double ScaledCubicApprox(double p[], double m1, double m2)
   {

      m1 /= N; //parameters were given for pulley angle not motor angle, corrected here
      m2 /= N; //parameters were given for pulley angle not motor angle, corrected here

      return CubicApprox(p, m1, m2);

   }

   private static double CubicApprox(double[] p, double m1, double m2)
   {
      double val = p[0] + p[1] * m1 + p[2] * m2 + p[3] * m1 * m1 + p[4] * m2 * m2 + p[5] * m1 * m2 + p[6] * m1 * m1 * m1 + p[7] * m1 * m1 * m2 + p[8] * m1 * m2
            * m2 + p[9] * m2 * m2 * m2;

      return val;
   }

   //This should return a 1D array version of the Jacobian inverse transpose matrix for the linkage
   //pJX and pJY are the parameters for the cubic mappings
   //the m1,m2 inputs are the motor1 and 2 pulley angles
   private static void JacobianInverseTranspose(double[] Jit, double m1, double m2)
   {

      Jit[0] = ScaledCubicApprox(pJitX, m1, m2); //Jit11
      Jit[1] = ScaledCubicApprox(pJitY, m1, m2); //Jit12
      Jit[2] = -ScaledCubicApprox(pJitX, m2, m1); //Jit21
      Jit[3] = ScaledCubicApprox(pJitY, m2, m1); //Jit22

   }
   
   public static void twobytwoInverseTranspose(double JToPack[], double Jit[])
   {

      double det = Jit[0] * Jit[3] - Jit[1] * Jit[2];
      JToPack[0] = Jit[3] / det;
      JToPack[1] = -Jit[2] / det;
      JToPack[2] = -Jit[1] / det;
      JToPack[3] = Jit[0] / det;
   }
   
   public static void twobytwoInverse(double JToPack[], double Jit[])
   {

      double det = Jit[0] * Jit[3] - Jit[1] * Jit[2];
      JToPack[0] = Jit[3] / det;
      JToPack[1] = -Jit[1] / det;
      JToPack[2] = -Jit[2] / det;
      JToPack[3] = Jit[0] / det;
   }

   private final double[] Jit = new double[4];
   private final double[] Jt = new double[4];
   private final double[] J = new double[4];
   private double qAnkleX, qAnkleY;
   private double qdAnkleX, qdAnkleY;
   private double motorVelocityRight, motorVelocityLeft;

   private double tauRightActuator, tauLeftActuator;
   private double tauAnkleX, tauAnkleY;

   @Override
   public void updateAnkleState(double motorAngleRight, double motorAngleLeft, double motorVelocityRight, double motorVelocityLeft,
         double tauMeasureAnkleRight, double tauMeasureAnkleLeft)
   {
      qAnkleX = ScaledCubicApprox(px, motorAngleRight, motorAngleLeft);
      qAnkleY = ScaledCubicApprox(py, motorAngleRight, motorAngleLeft);

      JacobianInverseTranspose(Jit, motorAngleRight, motorAngleLeft);

      qdAnkleX = (Jit[0] * motorVelocityRight + Jit[2] * motorVelocityLeft) / N;
      qdAnkleY = (Jit[1] * motorVelocityRight + Jit[3] * motorVelocityLeft) / N;

      twobytwoInverse(Jt, Jit);
      tauAnkleX = (Jt[0] * tauMeasureAnkleRight + Jt[1] * tauMeasureAnkleLeft) * N; //this is desired torque at motor 1
      tauAnkleY = (Jt[2] * tauMeasureAnkleRight + Jt[3] * tauMeasureAnkleLeft) * N; //this is desired torque at motor 2

   }

   @Override
   public double getQAnkleX()
   {
      return qAnkleX;
   }

   @Override
   public double getQAnkleY()
   {
      return qAnkleY;
   }

   @Override
   public double getQdAnkleX()
   {
      return qdAnkleX;
   }

   @Override
   public double getQdAnkleY()
   {
      return qdAnkleY;
   }

   // motorTorque = Jit * ankleTorques
   @Override
   public void calculateDesiredTau(double motorAngleRight, double motorAngleLeft, double tauDesiredAnkleX, double tauDesiredAnkleY)
   {
      JacobianInverseTranspose(Jit, motorAngleRight, motorAngleLeft);
      tauRightActuator = (Jit[0] * tauDesiredAnkleX + Jit[1] * tauDesiredAnkleY) / N; //this is desired torque at motor 1
      tauLeftActuator = (Jit[2] * tauDesiredAnkleX + Jit[3] * tauDesiredAnkleY) / N; //this is desired torque at motor 2
   }

   
   @Override
   public double calculateRightMotorAngle(double ankleX, double ankleY)
   {
      return CubicApprox(pM1, ankleX, ankleY);
   }

   @Override
   public double calculateLeftMotorAngle(double ankleX, double ankleY)
   {
      return CubicApprox(pM2, ankleX, ankleY);
   }

   @Override
   public double getTauRightActuator()
   {
      return tauRightActuator;
   }
   

   @Override
   public double getTauLeftActuator()
   {
      return tauLeftActuator;
   }

   @Override
   public double getTauAnkleX()
   {
      return tauAnkleX;
   }
   @Override
   public double getTauAnkleY()
   {
      return tauAnkleY;
   }
   

   
   @Override
   public double getRatio()
   {
      return N;
   }

   
   @Override
   public void calculateActuatordQd(double motorAngleRight, double motorAngleLeft, double qdAnkleX, double qdAnkleY)
   {
      JacobianInverseTranspose(Jit, motorAngleRight, motorAngleLeft);
      twobytwoInverseTranspose(J, Jit);
      motorVelocityRight = (J[0] * qdAnkleX + Jit[1] * qdAnkleY) * N;
      motorVelocityLeft = (J[2] * qdAnkleX + Jit[3] * qdAnkleY) * N;
   }
   @Override
   public double getMotorVelocityRight()
   {
      return motorVelocityRight;
   }
   
   @Override
   public double getMotorVelocityLeft()
   {
      return motorVelocityLeft;
   }   


}
