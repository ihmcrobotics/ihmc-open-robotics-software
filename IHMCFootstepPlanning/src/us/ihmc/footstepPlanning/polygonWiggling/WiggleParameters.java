package us.ihmc.footstepPlanning.polygonWiggling;

public class WiggleParameters
{
   /**
    * Relative weight given to polygon rotation in comparison to translation in the cost function. Lowering this value will cause the
    * polygon to rotate more.
    */
   public double rotationWeight = 0.2;

   /**
    * Maximum x translation that the wiggler is allowed to use.
    */
   public double maxX = 0.5;

   /**
    * Minimum x translation that the wiggler is allowed to use.
    */
   public double minX = -0.5;

   /**
    * Maximum y translation that the wiggler is allowed to use.
    */
   public double maxY = 0.5;

   /**
    * Minimum y translation that the wiggler is allowed to use.
    */
   public double minY = -0.5;

   /**
    * Maximum rotation that the wiggler is allowed to use.
    */
   public double maxYaw = Math.toRadians(15.0);

   /**
    * Minimum rotation that the wiggler is allowed to use.
    */
   public double minYaw = -Math.toRadians(15.0);

   /**
    * Amount that the polygon to wiggle needs to be inside the polygon to wiggle into. Negative means it can be outside.
    */
   public double deltaInside = 0.0;
}