package us.ihmc.simulationConstructionSetTools.util.perturbance;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;

public class ParticleCollisionCalculator
{
   private ParticleCollisionCalculator()
   {
      // disallow construction
   }

   /**
    * 
    * @param velocity1ToPack
    * @param velocity2ToPack
    * @param m1
    * @param m2
    * @param coefficientOfRestitution
    * Coefficient of restitution is always between 0 and 1
    * coefficientOfRestitution = 0 is a perfectly inelastic collision
    * coefficientOfRestitution = 1 is a perfectly elastic collision
    */
   public static void handleCollision(Vector3D velocity1ToPack, Vector3D velocity2ToPack, double m1, double m2, double coefficientOfRestitution)
   {
      MathTools.checkIntervalContains(coefficientOfRestitution, 0.0, 1.0);
      double totalMass = m1 + m2;

      Vector3D v1 = velocity1ToPack;
      Vector3D v2 = velocity2ToPack;

      double v1NewX = (coefficientOfRestitution * m2 * (v2.getX() - v1.getX()) + m1 * v1.getX() + m2 * v2.getX()) / totalMass;
      double v1NewY = (coefficientOfRestitution * m2 * (v2.getY() - v1.getY()) + m1 * v1.getY() + m2 * v2.getY()) / totalMass;
      double v1NewZ = (coefficientOfRestitution * m2 * (v2.getZ() - v1.getZ()) + m1 * v1.getZ() + m2 * v2.getZ()) / totalMass;

      double v2NewX = (coefficientOfRestitution * m1 * (v1.getX() - v2.getX()) + m1 * v1.getX() + m2 * v2.getX()) / totalMass;
      double v2NewY = (coefficientOfRestitution * m1 * (v1.getY() - v2.getY()) + m1 * v1.getY() + m2 * v2.getY()) / totalMass;
      double v2NewZ = (coefficientOfRestitution * m1 * (v1.getZ() - v2.getZ()) + m1 * v1.getZ() + m2 * v2.getZ()) / totalMass;

      v1.set(v1NewX, v1NewY, v1NewZ);
      v2.set(v2NewX, v2NewY, v2NewZ);
   }
}
