/*
 * This class is based on interface and control code for STEPPR and WANDERER, 
 * two energy-efficient humanoid bipeds developed by the High Consequence Automation and Robotics Group at Sandia National Laboratories.
 *
 */
package us.ihmc.robotics.kinematics.fourbar;

public class FourbarCalculator
{
   private final FourbarProperties fourbar;
   private final double L1;
   private final double L2;
   private final double L3;
   private final double L4;

   private double tempRatio;
   private double ratioBasedOnCalculatedInputAngle;
   private double ratioBasedOnCalculatedOutputAngle;

   public FourbarCalculator(FourbarProperties fourbar)
   {
      this.fourbar = fourbar;
      L1 = this.fourbar.getGroundLink().getLength();
      L2 = this.fourbar.getInputLink().getLength();
      L3 = this.fourbar.getFloatingLink().getLength();
      L4 = this.fourbar.getOutputLink().getLength();
   }

   private double getElbowSign()
   {
      return fourbar.isElbowDown() ? -1.0 : 1.0;
   }

   /**
    * Calculated the input angle based on the output angle
    * 
    * @param beta External angle from GroundLink to OutputLink
    */
   public double calculateInputAngleFromOutputAngle(double beta)
   {
      double alpha = calculateInputAngleFromOutputAngle(beta, L1, L2, L3, L4);
      ratioBasedOnCalculatedInputAngle = tempRatio;
      return alpha;
   }

   public double getFourbarRatioBasedOnCalculatedInputAngle()
   {
      return ratioBasedOnCalculatedInputAngle;
   }
   
   public double getFourbarRatioBasedOnCalculatedOutputAngle()
   {
      return ratioBasedOnCalculatedOutputAngle;
   }

   public double calculateOutputAngleFromInputAngle(double beta)
   {
      // Flip input and output in the fourbar linkage and run the output to input calculator
      double alpha = calculateInputAngleFromOutputAngle(Math.PI - beta, L1, L4, L3, L2);
      ratioBasedOnCalculatedOutputAngle = tempRatio;
      return Math.PI - alpha;
   }

   private double calculateInputAngleFromOutputAngle(double beta, double L1, double L2, double L3, double L4)
   {
      double phi = Math.PI - beta; //Internal angle from GroundLink to OutputLink
      double x = L1 - L4 * Math.cos(phi); //Distance from L1-L2 joint to L4-L3 joint as measured parallel to L1
      double y = L4 * Math.sin(phi); //Distance from L1-L2 joint to L4-L3 joint as measured perpendicular to L1
      double Dsqrd = x * x + y * y; //Distance squared from L1-L2 joint to L4-L3
      double D = Math.sqrt(Dsqrd); //Distance from L1-L2 joint to L4-L3
      double a1 = Math.atan2(y, x); //Angle from L1 to the L4-L3 joint
      double a2 = getElbowSign() * Math.acos((L2 * L2 + Dsqrd - L3 * L3) / (2 * L2 * D)); //Angle from L2 to the L4-L3 joint
      double da1db = 1 - L1 * x / Dsqrd; //Derivative of a1 wrt beta
      double da2db = L1 * y * (D - L2 * Math.cos(a2)) / (L2 * Dsqrd * Math.sin(a2)); //Derivative of a2 wrt beta
      tempRatio = da1db + da2db;
      double inputAngle = a1 + a2;
      if(Double.isNaN(inputAngle))
      {
         return 0.0;
      }
      else
      {
         return inputAngle;
      }
   }
}
