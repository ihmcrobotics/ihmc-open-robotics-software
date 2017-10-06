package us.ihmc.robotics.controllers;

import us.ihmc.robotics.controllers.pidGains.GainCalculator;

/**
 * @author twan
 *         Date: 6/15/13
 */
public class CylindricalPDGains
{
   private final double kpRadius;
   private final double kpAngle;
   private final double kpZ;

   private final double kdRadius;
   private final double kdAngle;
   private final double kdZ;

   public CylindricalPDGains(double kpRadius, double kpAngle, double kpZ, double zeta)
   {
      this.kpRadius = kpRadius;
      this.kpAngle = kpAngle;
      this.kpZ = kpZ;

      this.kdRadius = GainCalculator.computeDerivativeGain(kpRadius, zeta);
      this.kdAngle = GainCalculator.computeDerivativeGain(kpAngle, zeta);
      this.kdZ = GainCalculator.computeDerivativeGain(kpZ, zeta);
   }

   public double getKpRadius()
   {
      return kpRadius;
   }

   public double getKpAngle()
   {
      return kpAngle;
   }

   public double getKpZ()
   {
      return kpZ;
   }

   public double getKdRadius()
   {
      return kdRadius;
   }

   public double getKdAngle()
   {
      return kdAngle;
   }

   public double getKdZ()
   {
      return kdZ;
   }
}
