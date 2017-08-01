package us.ihmc.commonWalkingControlModules.dynamicReachability;

import java.util.List;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CoMIntegrationTools
{
   /**
    * Calculation of center of mass position at the end of a certain duration, assuming that the CMP location is held constant throughout that time interval.
    * This has an analytic solution to the center of mass dynamics.
    *
    * Utilizes the same method as {@link #integrateCoMPositionUsingConstantCMP(double, double, double, FramePoint, FramePoint, FramePoint, FramePoint)}, but
    * assumes the initial time is 0.0.
    *
    * @param duration time interval over which to integrate the center of mass dynamics. Assumes that the initial time is time = 0.0.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param constantCMP constant location of the CMP during the integration interval.
    * @param initialICP initial location of the ICP at the beginning of the integration interval.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionUsingConstantCMP(double duration, double omega0, YoFramePoint constantCMP, YoFramePoint initialICP,
         FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      integrateCoMPositionUsingConstantCMP(0.0, duration, omega0, constantCMP.getFrameTuple(), initialICP.getFrameTuple(), initialCoM, finalCoMToPack);
   }

   /**
    * Calculation of center of mass position at the end of a certain duration, assuming that the CMP location is held constant throughout that time interval.
    * This has an analytic solution to the center of mass dynamics.
    *
    * Utilizes the same method as {@link #integrateCoMPositionUsingConstantCMP(double, double, double, FramePoint, FramePoint, FramePoint, FramePoint)}, but
    * assumes the initial time is 0.0.
    *
    * @param duration time interval over which to integrate the center of mass dynamics. Assumes that the initial time is time = 0.0.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param constantCMP constant location of the CMP during the integration interval.
    * @param initialICP initial location of the ICP at the beginning of the integration interval.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionUsingConstantCMP(double duration, double omega0, FramePoint constantCMP, FramePoint initialICP,
         FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      integrateCoMPositionUsingConstantCMP(0.0, duration, omega0, constantCMP, initialICP, initialCoM, finalCoMToPack);
   }



   /**
    * Calculation of center of mass position at the end of a certain duration, assuming that the CMP location is held constant throughout that time interval.
    * This has an analytic solution to the center of mass dynamics.
    *
    * Utilizes the same method as {@link #integrateCoMPositionUsingConstantCMP(double, double, double, FramePoint, FramePoint, FramePoint, FramePoint)}, but
    * assumes the initial time is 0.0.
    *
    * @param duration time interval over which to integrate the center of mass dynamics. Assumes that the initial time is time = 0.0.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param constantCMP constant location of the CMP during the integration interval.
    * @param initialICP initial location of the ICP at the beginning of the integration interval.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionUsingConstantCMP(double duration, double omega0, YoFramePoint constantCMP, YoFramePoint initialICP,
         FramePoint2d initialCoM, FramePoint2d finalCoMToPack)
   {
      integrateCoMPositionUsingConstantCMP(0.0, duration, omega0, constantCMP.getFrameTuple(), initialICP.getFrameTuple(), initialCoM, finalCoMToPack);
   }

   /**
    * Calculation of center of mass position at the end of a certain duration, assuming that the CMP location is held constant throughout that time interval.
    * This has an analytic solution to the center of mass dynamics.
    *
    * Utilizes the same method as {@link #integrateCoMPositionUsingConstantCMP(double, double, double, FramePoint, FramePoint, FramePoint, FramePoint)}, but
    * assumes the initial time is 0.0.
    *
    * @param duration time interval over which to integrate the center of mass dynamics. Assumes that the initial time is time = 0.0.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param constantCMP constant location of the CMP during the integration interval.
    * @param initialICP initial location of the ICP at the beginning of the integration interval.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionUsingConstantCMP(double duration, double omega0, FramePoint constantCMP, FramePoint initialICP,
         FramePoint2d initialCoM, FramePoint2d finalCoMToPack)
   {
      integrateCoMPositionUsingConstantCMP(0.0, duration, omega0, constantCMP, initialICP, initialCoM, finalCoMToPack);
   }




   /**
    * Calculation of center of mass position at the end of a certain time interval, assuming that the CMP location is held constant throughout that time
    * interval. This has an analytic solution to the center of mass dynamics.
    *
    * @param initialTime initial time of the interval over which to integrate.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param constantCMP constant location of the CMP during the integration interval.
    * @param initialICP initial location of the ICP at the beginning of the integration interval.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionUsingConstantCMP(double initialTime, double finalTime, double omega0, FramePoint constantCMP, FramePoint initialICP,
         YoFramePoint initialCoM, FramePoint finalCoMToPack)
   {
      integrateCoMPositionUsingConstantCMP(initialTime, finalTime, omega0, constantCMP, initialICP, initialCoM.getFrameTuple(), finalCoMToPack);
   }

   /**
    * Calculation of center of mass position at the end of a certain time interval, assuming that the CMP location is held constant throughout that time
    * interval. This has an analytic solution to the center of mass dynamics.
    *
    * @param initialTime initial time of the interval over which to integrate.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param constantCMP constant location of the CMP during the integration interval.
    * @param initialICP initial location of the ICP at the beginning of the integration interval.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionUsingConstantCMP(double initialTime, double finalTime, double omega0, FramePoint constantCMP, FramePoint initialICP,
         FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      initialCoM.checkReferenceFrameMatch(constantCMP);
      initialCoM.checkReferenceFrameMatch(initialICP);
      initialCoM.checkReferenceFrameMatch(finalCoMToPack);

      finalCoMToPack.set(initialCoM);

      double timeDelta = Math.max(finalTime - initialTime, 0.0);
      if (timeDelta == 0.0)
         return;

      double xPosition = integrateCoMPositionWithConstantCMP(timeDelta, omega0, initialICP.getX(), initialCoM.getX(), constantCMP.getX());
      double yPosition = integrateCoMPositionWithConstantCMP(timeDelta, omega0, initialICP.getY(), initialCoM.getY(), constantCMP.getY());

      finalCoMToPack.setX(xPosition);
      finalCoMToPack.setY(yPosition);
   }



   /**
    * Calculation of center of mass position at the end of a certain time interval, assuming that the CMP location is held constant throughout that time
    * interval. This has an analytic solution to the center of mass dynamics.
    *
    * @param initialTime initial time of the interval over which to integrate.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param constantCMP constant location of the CMP during the integration interval.
    * @param initialICP initial location of the ICP at the beginning of the integration interval.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionUsingConstantCMP(double initialTime, double finalTime, double omega0, FramePoint constantCMP, FramePoint initialICP,
         YoFramePoint2d initialCoM, FramePoint2d finalCoMToPack)
   {
      integrateCoMPositionUsingConstantCMP(initialTime, finalTime, omega0, constantCMP, initialICP, initialCoM.getFrameTuple2d(), finalCoMToPack);
   }

   /**
    * Calculation of center of mass position at the end of a certain time interval, assuming that the CMP location is held constant throughout that time
    * interval. This has an analytic solution to the center of mass dynamics.
    *
    * @param initialTime initial time of the interval over which to integrate.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param constantCMP constant location of the CMP during the integration interval.
    * @param initialICP initial location of the ICP at the beginning of the integration interval.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionUsingConstantCMP(double initialTime, double finalTime, double omega0, FramePoint constantCMP, FramePoint initialICP,
         FramePoint2d initialCoM, FramePoint2d finalCoMToPack)
   {
      initialCoM.checkReferenceFrameMatch(constantCMP);
      initialCoM.checkReferenceFrameMatch(initialICP);
      initialCoM.checkReferenceFrameMatch(finalCoMToPack);

      finalCoMToPack.set(initialCoM);

      double timeDelta = Math.max(finalTime - initialTime, 0.0);

      if (timeDelta == 0.0)
         return;

      double xPosition = integrateCoMPositionWithConstantCMP(timeDelta, omega0, initialICP.getX(), initialCoM.getX(), constantCMP.getX());
      double yPosition = integrateCoMPositionWithConstantCMP(timeDelta, omega0, initialICP.getY(), initialCoM.getY(), constantCMP.getY());

      finalCoMToPack.setX(xPosition);
      finalCoMToPack.setY(yPosition);
   }

   private static double integrateCoMPositionWithConstantCMP(double duration, double omega0, double initialICPPosition, double initialCoMPosition,
         double cmpPosition)
   {
      if (duration == 0.0)
         return initialCoMPosition;

      double position = 0.5 * Math.exp(omega0 * duration) * (initialICPPosition - cmpPosition);
      position += Math.exp(-omega0 * duration) * (initialCoMPosition - 0.5 * (initialICPPosition + cmpPosition));
      position += cmpPosition;

      return position;
   }




   /**
    * Calculation of center of mass position at the end of a certain duration, assuming that the ICP trajectory is in the form of a cubic spline over that
    * time interval.
    *
    * Utilizes the same method as {@link #integrateCoMPositionUsingCubicICP(double, double, double, double, ReferenceFrame, YoPolynomial, YoPolynomial,
    * FramePoint, FramePoint)}, but assumes the initial time is 0.0, and segment duration is the integration duration.
    *
    * @param duration time interval over which to integrate the center of mass dynamics. Assumes that the initial time is time = 0.0, and that this is the
    *                 entire duration of the spline.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param polynomialFrame reference frame in which the polynomial coefficients are defined.
    * @param xPolynomial polynomial value that defines the coefficients for the ICP trajectory in the X-Direction.
    * @param yPolynomial polynomial value that defines the coefficients for the ICP trajectory in the Y-Direction.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionFromCubicICP(double duration, double omega0, ReferenceFrame polynomialFrame, YoPolynomial xPolynomial,
         YoPolynomial yPolynomial, YoFramePoint initialCoM, FramePoint finalCoMToPack)
   {
      integrateCoMPositionFromCubicICP(duration, omega0, polynomialFrame, xPolynomial, yPolynomial, initialCoM.getFrameTuple(), finalCoMToPack);
   }

   /**
    * Calculation of center of mass position at the end of a certain duration, assuming that the ICP trajectory is in the form of a cubic spline over that
    * time interval.
    *
    * Utilizes the same method as {@link #integrateCoMPositionUsingCubicICP(double, double, double, double, ReferenceFrame, YoPolynomial, YoPolynomial,
    * FramePoint, FramePoint)}, but assumes the initial time is 0.0, and segment duration is the integration duration.
    *
    * @param duration time interval over which to integrate the center of mass dynamics. Assumes that the initial time is time = 0.0, and that this is the
    *                 entire duration of the spline.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param polynomialFrame reference frame in which the polynomial coefficients are defined.
    * @param xPolynomial polynomial value that defines the coefficients for the ICP trajectory in the X-Direction.
    * @param yPolynomial polynomial value that defines the coefficients for the ICP trajectory in the Y-Direction.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionFromCubicICP(double duration, double omega0, ReferenceFrame polynomialFrame, YoPolynomial xPolynomial,
         YoPolynomial yPolynomial, FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      integrateCoMPositionUsingCubicICP(0.0, duration, duration, omega0, polynomialFrame, xPolynomial, yPolynomial, initialCoM, finalCoMToPack);
   }

   /**
    * Calculation of center of mass position at the end of a certain time, assuming that the ICP trajectory is in the form of a cubic spline over that
    * time interval.
    *
    * Utilizes the same method as {@link #integrateCoMPositionUsingCubicICP(double, double, double, double, ReferenceFrame, YoPolynomial, YoPolynomial,
    * FramePoint, FramePoint)}, but assumes the initial time is 0.0.
    *
    * @param time time interval over which to integrate the center of mass dynamics. Assumes that the initial time is time = 0.0
    * @param duration total duration for which the spline is defined.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param polynomialFrame reference frame in which the polynomial coefficients are defined.
    * @param xPolynomial polynomial value that defines the coefficients for the ICP trajectory in the X-Direction.
    * @param yPolynomial polynomial value that defines the coefficients for the ICP trajectory in the Y-Direction.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionUsingCubicICP(double time, double duration, double omega0, ReferenceFrame polynomialFrame, YoPolynomial xPolynomial,
         YoPolynomial yPolynomial, FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      integrateCoMPositionUsingCubicICP(0.0, MathTools.clamp(time, 0.0, duration), duration, omega0, polynomialFrame, xPolynomial, yPolynomial, initialCoM,
            finalCoMToPack);
   }

   /**
    * Calculation of center of mass position at the end of a certain time, assuming that the ICP trajectory is in the form of a cubic spline over that
    * time interval.
    *
    * Utilizes the same method as {@link #integrateCoMPositionUsingCubicICP(double, double, double, double, ReferenceFrame, YoPolynomial, YoPolynomial,
    * FramePoint, FramePoint)}, but assumes the initial time is 0.0.
    *
    * @param time time interval over which to integrate the center of mass dynamics. Assumes that the initial time is time = 0.0
    * @param duration total duration for which the spline is defined.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param polynomialFrame reference frame in which the polynomial coefficients are defined.
    * @param xPolynomial polynomial value that defines the coefficients for the ICP trajectory in the X-Direction.
    * @param yPolynomial polynomial value that defines the coefficients for the ICP trajectory in the Y-Direction.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionUsingCubicICP(double time, double duration, double omega0, ReferenceFrame polynomialFrame, YoPolynomial xPolynomial,
         YoPolynomial yPolynomial, FramePoint2d initialCoM, FramePoint2d finalCoMToPack)
   {
      integrateCoMPositionUsingCubicICP(0.0, MathTools.clamp(time, 0.0, duration), duration, omega0, polynomialFrame, xPolynomial, yPolynomial, initialCoM,
            finalCoMToPack);
   }


   /**
    * Calculation of center of mass position at the end of certain time interval, assuming that the ICP trajectory is in the form of a cubic spline over that
    * time interval.
    *
    * @param initialTime time at the beginning of the time interval
    * @param finalTime time at the end of the time interval
    * @param duration total duration for which the spline is defined.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param polynomialFrame reference frame in which the polynomial coefficients are defined.
    * @param xPolynomial polynomial value that defines the coefficients for the ICP trajectory in the X-Direction.
    * @param yPolynomial polynomial value that defines the coefficients for the ICP trajectory in the Y-Direction.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionUsingCubicICP(double initialTime, double finalTime, double duration, double omega0,
         ReferenceFrame polynomialFrame, YoPolynomial xPolynomial, YoPolynomial yPolynomial, YoFramePoint initialCoM, FramePoint finalCoMToPack)
   {
      integrateCoMPositionUsingCubicICP(initialTime, finalTime, duration, omega0, polynomialFrame, xPolynomial, yPolynomial, initialCoM.getFrameTuple(),
            finalCoMToPack);
   }

   /**
    * Calculation of center of mass position at the end of certain time interval, assuming that the ICP trajectory is in the form of a cubic spline over that
    * time interval.
    *
    * @param initialTime time at the beginning of the time interval
    * @param finalTime time at the end of the time interval
    * @param duration total duration for which the spline is defined.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param polynomialFrame reference frame in which the polynomial coefficients are defined.
    * @param xPolynomial polynomial value that defines the coefficients for the ICP trajectory in the X-Direction.
    * @param yPolynomial polynomial value that defines the coefficients for the ICP trajectory in the Y-Direction.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionUsingCubicICP(double initialTime, double finalTime, double duration, double omega0, ReferenceFrame polynomialFrame,
         YoPolynomial xPolynomial, YoPolynomial yPolynomial, FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      initialCoM.checkReferenceFrameMatch(polynomialFrame);
      initialCoM.checkReferenceFrameMatch(finalCoMToPack);

      if (xPolynomial.getNumberOfCoefficients() != yPolynomial.getNumberOfCoefficients() && yPolynomial.getNumberOfCoefficients() != 4)
         throw new RuntimeException("The number of coefficients in the polynomials are wrong!");

      double integrationDuration = finalTime - initialTime;
      integrationDuration = MathTools.clamp(integrationDuration, 0.0, duration);

      finalCoMToPack.set(initialCoM);

      if (integrationDuration == 0.0)
         return;

      double xPosition = integrateCoMPositionOverPolynomial(initialCoM.getX(), integrationDuration, omega0, xPolynomial);
      double yPosition = integrateCoMPositionOverPolynomial(initialCoM.getY(), integrationDuration, omega0, yPolynomial);

      finalCoMToPack.setX(xPosition);
      finalCoMToPack.setY(yPosition);
   }

   /**
    * Calculation of center of mass position at the end of certain time interval, assuming that the ICP trajectory is in the form of a cubic spline over that
    * time interval.
    *
    * @param initialTime time at the beginning of the time interval
    * @param finalTime time at the end of the time interval
    * @param duration total duration for which the spline is defined.
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param polynomialFrame reference frame in which the polynomial coefficients are defined.
    * @param xPolynomial polynomial value that defines the coefficients for the ICP trajectory in the X-Direction.
    * @param yPolynomial polynomial value that defines the coefficients for the ICP trajectory in the Y-Direction.
    * @param initialCoM initial location of the CoM at the beginning of the integration interval.
    * @param finalCoMToPack location of the center of mass at the end of the integration duration. Modified.
    */
   public static void integrateCoMPositionUsingCubicICP(double initialTime, double finalTime, double duration, double omega0, ReferenceFrame polynomialFrame,
         YoPolynomial xPolynomial, YoPolynomial yPolynomial, FramePoint2d initialCoM, FramePoint2d finalCoMToPack)
   {
      initialCoM.checkReferenceFrameMatch(polynomialFrame);
      initialCoM.checkReferenceFrameMatch(finalCoMToPack);

      if (xPolynomial.getNumberOfCoefficients() != yPolynomial.getNumberOfCoefficients() && yPolynomial.getNumberOfCoefficients() != 4)
         throw new RuntimeException("The number of coefficients in the polynomials are wrong!");

      double integrationDuration = finalTime - initialTime;
      integrationDuration = MathTools.clamp(integrationDuration, 0.0, duration);
      finalCoMToPack.set(initialCoM);

      if (integrationDuration == 0.0)
         return;

      double xPosition = integrateCoMPositionOverPolynomial(initialCoM.getX(), integrationDuration, omega0, xPolynomial);
      double yPosition = integrateCoMPositionOverPolynomial(initialCoM.getY(), integrationDuration, omega0, yPolynomial);

      finalCoMToPack.setX(xPosition);
      finalCoMToPack.setY(yPosition);
   }





   public static void integrateFinalCoMPositionFromCubicDCM(double segmentDuration, double omega0, ReferenceFrame polynomialFrame, YoPolynomial xPolynomial,
         YoPolynomial yPolynomial, YoPolynomial zPolynomial, FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      integrateCoMPositionUsingCubicDCM(0.0, segmentDuration, segmentDuration, omega0, polynomialFrame, xPolynomial, yPolynomial, zPolynomial, initialCoM,
            finalCoMToPack);
   }

   public static void integrateCoMPositionUsingCubicDCM(double initialTime, double finalTime, double segmentDuration, double omega0, ReferenceFrame polynomialFrame,
         YoPolynomial xPolynomial, YoPolynomial yPolynomial, YoPolynomial zPolynomial, FramePoint initialCoM, FramePoint finalCoMToPack)
   {
      initialCoM.checkReferenceFrameMatch(polynomialFrame);
      initialCoM.checkReferenceFrameMatch(finalCoMToPack);

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
      if (integrationDuration == 0.0)
         return initialPosition;

      double position = polynomial.getCoefficient(3) * Math.pow(integrationDuration, 3.0);
      position += (polynomial.getCoefficient(2) + -3.0 / omega0 * polynomial.getCoefficient(3)) * Math.pow(integrationDuration, 2.0);
      position += (polynomial.getCoefficient(1) - 2.0 / omega0 * polynomial.getCoefficient(2) + 6.0 / Math.pow(omega0, 2.0) * polynomial.getCoefficient(3))
            * integrationDuration;
      position += (polynomial.getCoefficient(0) - 1.0 / omega0 * polynomial.getCoefficient(1) + 2.0 / Math.pow(omega0, 2.0) * polynomial.getCoefficient(2)
            - 6.0 / Math.pow(omega0, 3.0) * polynomial.getCoefficient(3));
      position += Math.exp(-omega0 * integrationDuration) * (initialPosition - polynomial.getCoefficient(0) + 1.0 / omega0 * polynomial.getCoefficient(1)
            - 2.0 / Math.pow(omega0, 2.0) * polynomial.getCoefficient(2) + 6.0 / Math.pow(omega0, 3.0) * polynomial.getCoefficient(3));

      return position;
   }
   
   
   
   //TODO: implement validity checks
   public static void computeDesiredCenterOfMassCornerPoints(List<FramePoint> entryCornerPointsToPack, List<FramePoint> exitCornerPointsToPack,
                                                             List<FramePoint> entryCoMCornerPointsToPack, List<FramePoint> exitCoMCornerPointsToPack,
                                                             List<YoFrameTrajectory3D> cmpPolynomials3D, double omega0)
   {
      YoFrameTrajectory3D cmpPolynomial3D = cmpPolynomials3D.get(cmpPolynomials3D.size() - 1);
      
      cmpPolynomial3D.compute(cmpPolynomial3D.getFinalTime());
      FramePoint nextEntryCoMCornerPoint = cmpPolynomial3D.getFramePosition();
            
      PrintTools.debug("Step");
      for (int i = cmpPolynomials3D.size() - 1; i >= 0; i--)
      {
         cmpPolynomial3D = cmpPolynomials3D.get(i);
         
         FramePoint exitCornerPoint = exitCornerPointsToPack.get(i);
         
         FramePoint exitCoMCornerPoint = exitCoMCornerPointsToPack.get(i);
         FramePoint entryCoMCornerPoint = entryCoMCornerPointsToPack.get(i);
         
         exitCoMCornerPoint.set(nextEntryCoMCornerPoint);
         
         computeDesiredCenterOfMassPosition(omega0, cmpPolynomial3D.getInitialTime(), exitCornerPoint, exitCoMCornerPoint, cmpPolynomial3D, entryCoMCornerPoint);

//         PrintTools.debug("Omega0 = " + omega0);
//         PrintTools.debug("t0 = " + cmpPolynomial3D.getInitialTime());
//         PrintTools.debug("tF = " + cmpPolynomial3D.getFinalTime());
//         PrintTools.debug("a1 = " + cmpPolynomial3D.getYoTrajectoryX().getCoefficient(1));
//         PrintTools.debug("Entry ICP = " + entryCornerPointsToPack.get(i).toString());
//         PrintTools.debug("Exit ICP = " + exitCornerPoint.toString());
//         PrintTools.debug("Entry CoM = " + entryCoMCornerPoint.toString());
//         PrintTools.debug("Exit CoM = " + exitCoMCornerPoint.toString());
//         cmpPolynomial3D.compute(cmpPolynomial3D.getInitialTime());
//         PrintTools.debug("Entry CMP = " + cmpPolynomial3D.getFramePosition().toString());
//         PrintTools.debug("");
         
         nextEntryCoMCornerPoint = entryCoMCornerPoint;
      }
      PrintTools.debug("Exit CoMs = " + exitCoMCornerPointsToPack.subList(0,20).toString());
      PrintTools.debug("Entry CoMs = " + entryCoMCornerPointsToPack.subList(0,20).toString());
      PrintTools.debug("");
   }
   
   //TODO: implement validity checks
   public static void computeDesiredCenterOfMassPosition(double omega0, double time, FramePoint finalCapturePoint, FramePoint finalCenterOfMass, YoFrameTrajectory3D cmpPolynomial3D, 
                                                         FramePoint desiredCenterOfMassPositionToPack)
   {         
      SmoothCoMIntegrationTools.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 0, cmpPolynomial3D, finalCapturePoint, finalCenterOfMass, desiredCenterOfMassPositionToPack);
   }
   
   //TODO: implement validity checks
   public static void computeDesiredCenterOfMassVelocity(double omega0, double time, FramePoint finalCapturePoint, FramePoint finalCenterOfMass, YoFrameTrajectory3D cmpPolynomial3D, 
                                                         FrameVector desiredCenterOfMassVelocityToPack)
   {         
      SmoothCoMIntegrationTools.calculateCoMQuantityFromCorrespondingCMPPolynomial3D(omega0, time, 1, cmpPolynomial3D, finalCapturePoint, finalCenterOfMass, desiredCenterOfMassVelocityToPack);
   }
}
