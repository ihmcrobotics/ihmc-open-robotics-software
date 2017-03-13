package us.ihmc.commonWalkingControlModules.dynamicReachability;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.trajectories.YoPolynomial;

public class CoMIntegrationTools
{

   public static void computeFinalCoMPositionUsingConstantCMP(double segmentDuration, double omega0, FramePoint constantCMP, FramePoint initialICP,
         FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      computeCoMPositionUsingConstantCMP(0.0, segmentDuration, omega0, constantCMP, initialICP, initialCoM, finalCoMToPack);
   }

   public static void computeCoMPositionUsingConstantCMP(double initialTime, double finalTime, double omega0, FramePoint constantCMP, FramePoint initialICP,
         FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      double timeDelta = finalTime - initialTime;

      double xPosition = integrateCoMPositionWithConstantCMP(timeDelta, omega0, initialICP.getX(), initialCoM.getX(), constantCMP.getX());
      double yPosition = integrateCoMPositionWithConstantCMP(timeDelta, omega0, initialICP.getY(), initialCoM.getY(), constantCMP.getY());

      finalCoMToPack.set(initialCoM);
      finalCoMToPack.setX(xPosition);
      finalCoMToPack.setY(yPosition);
   }

   private static double integrateCoMPositionWithConstantCMP(double duration, double omega0, double initialICPPosition, double initialCoMPosition, double cmpPosition)
   {
      double position = Math.exp(omega0 * duration) * (initialICPPosition - cmpPosition);
      position += -Math.exp(-omega0 * duration) * (initialCoMPosition - initialICPPosition);
      position += cmpPosition;

      return position;
   }

   public static void computeFinalCoMPositionFromCubicICP(double segmentDuration, double omega0, YoPolynomial xPolynomial, YoPolynomial yPolynomial,
         FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      computeCoMPositionUsingCubicICP(0.0, segmentDuration, segmentDuration, omega0, xPolynomial, yPolynomial, initialCoM, finalCoMToPack);
   }

   public static void computeCoMPositionUsingCubicICP(double initialTime, double finalTime, double segmentDuration, double omega0, YoPolynomial xPolynomial,
         YoPolynomial yPolynomial, FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      if (xPolynomial.getNumberOfCoefficients() != yPolynomial.getNumberOfCoefficients() && yPolynomial.getNumberOfCoefficients() != 4)
         throw new RuntimeException("The number of coefficients in the polynomials are wrong!");

      double integrationDuration = finalTime - initialTime;
      integrationDuration = MathTools.clamp(integrationDuration, 0.0, segmentDuration);

      double xPosition = integrateCoMPositionOverPolynomial(initialCoM.getX(), integrationDuration, omega0, xPolynomial);
      double yPosition = integrateCoMPositionOverPolynomial(initialCoM.getY(), integrationDuration, omega0, yPolynomial);

      finalCoMToPack.set(initialCoM);
      finalCoMToPack.setX(xPosition);
      finalCoMToPack.setY(yPosition);
   }

   public static void computeFinalCoMPositionFromCubicDCM(double segmentDuration, double omega0, YoPolynomial xPolynomial, YoPolynomial yPolynomial,
         YoPolynomial zPolynomial, FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      computeCoMPositionUsingCubicDCM(0.0, segmentDuration, segmentDuration, omega0, xPolynomial, yPolynomial, zPolynomial, initialCoM, finalCoMToPack);
   }

   public static void computeCoMPositionUsingCubicDCM(double initialTime, double finalTime, double segmentDuration, double omega0, YoPolynomial xPolynomial,
         YoPolynomial yPolynomial, YoPolynomial zPolynomial, FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      if (xPolynomial.getNumberOfCoefficients() != yPolynomial.getNumberOfCoefficients() &&
            yPolynomial.getNumberOfCoefficients() != zPolynomial.getNumberOfCoefficients() && zPolynomial.getNumberOfCoefficients() != 4)
         throw new RuntimeException("The number of coefficients in the polynomials are wrong!");

      double integrationDuration = finalTime - initialTime;
      integrationDuration = MathTools.clamp(integrationDuration, 0.0, segmentDuration);

      double xPosition = integrateCoMPositionOverPolynomial(initialCoM.getX(), integrationDuration, omega0, xPolynomial);
      double yPosition = integrateCoMPositionOverPolynomial(initialCoM.getY(), integrationDuration, omega0, yPolynomial);
      double zPosition = integrateCoMPositionOverPolynomial(initialCoM.getZ(), integrationDuration, omega0, zPolynomial);

      finalCoMToPack.setToZero(initialCoM.getReferenceFrame());
      finalCoMToPack.set(xPosition, yPosition, zPosition);
   }

   private static double integrateCoMPositionOverPolynomial(double initialPosition, double integrationDuration, double omega0, YoPolynomial polynomial)
   {
      double position = polynomial.getCoefficient(3) * Math.pow(integrationDuration, 3.0);
      position += (polynomial.getCoefficient(2) + -3.0 / omega0 * polynomial.getCoefficient(3)) * Math.pow(integrationDuration, 2.0);
      position += (polynomial.getCoefficient(1) - 2.0 / omega0 * polynomial.getCoefficient(2) + 6.0 / Math.pow(omega0, 2.0) * polynomial.getCoefficient(3)) * integrationDuration;
      position += (polynomial.getCoefficient(0) - 1.0 / omega0 * polynomial.getCoefficient(1) + 2.0 / Math.pow(omega0, 2.0) * polynomial.getCoefficient(2) - 6.0 / Math.pow(omega0, 3.0) * polynomial.getCoefficient(3));
      position += Math.exp(-omega0 * integrationDuration) * (initialPosition - polynomial.getCoefficient(0) + 1.0 / omega0 * polynomial.getCoefficient(1) - 2.0 / Math.pow(omega0, 2.0) * polynomial.getCoefficient(2) + 6.0 / Math.pow(omega0, 3.0) * polynomial.getCoefficient(3));

      return position;
   }
}
