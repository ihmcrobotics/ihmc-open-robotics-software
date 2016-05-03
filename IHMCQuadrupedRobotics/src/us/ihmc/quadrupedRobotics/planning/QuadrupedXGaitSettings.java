package us.ihmc.quadrupedRobotics.planning;

public class QuadrupedXGaitSettings
{
   /**
    * Nominal x offset between front and hind feet.
    */
   private double stanceLength;

   /**
    * Nominal y offset between left and right feet.
    */
   private double stanceWidth;

   /**
    * Ground clearance for each step (in meters).
    */
   private double groundClearance;


   /**
    * Time duration of each swing phase.
    */
   private double swingDuration;

   /**
    * Time duration that both hind or both front feet feet are in support (in seconds).
    */
   private double endPairSupportDuration = 1.0;

   /**
    * Nominal phase shift between front and hind steps in degrees. (0: pace, 90: amble, 180: trot)
    */
   private double xGaitPhase;

   public QuadrupedXGaitSettings()
   {
      stanceLength = 1.0;
      stanceWidth = 0.25;
      groundClearance = 0.2;
      swingDuration = 0.35;
      endPairSupportDuration = 0.0;
      xGaitPhase = 90.0;
   }

   public double getStanceLength()
   {
      return stanceLength;
   }

   public void setStanceLength(double stanceLength)
   {
      this.stanceLength = stanceLength;
   }

   public double getStanceWidth()
   {
      return stanceWidth;
   }

   public void setStanceWidth(double stanceWidth)
   {
      this.stanceWidth = stanceWidth;
   }

   public double getGroundClearance()
   {
      return groundClearance;
   }

   public void setGroundClearance(double groundClearance)
   {
      this.groundClearance = groundClearance;
   }

   public double getSwingDuration()
   {
      return swingDuration;
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration = swingDuration;
   }

   public double getEndPairSupportDuration()
   {
      return endPairSupportDuration;
   }

   public void setEndPairSupportDuration(double endPairSupportDuration)
   {
      this.endPairSupportDuration = endPairSupportDuration;
   }

   public double getXGaitPhase()
   {
      return xGaitPhase;
   }

   public void setXGaitPhase(double xGaitPhase)
   {
      this.xGaitPhase = xGaitPhase;
   }
}
