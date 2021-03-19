package us.ihmc.commonWalkingControlModules.contact.particleFilter;


public class FlatGroundBasedContactEvaluator implements ContactPointEvaluator
{
   private static final double GROUND_HEIGHT = 0.0;
   private static final double GROUND_VARIATION = 0.25;
   private static final double MIN_PROBABILITY = 0.4;

   @Override
   /**
    * Probability that a point at height h is in contact given by a peice-wise linear function:
    * p = 1.0, p <= groundHeight
    * p = 1.0 - (1.0 - minProbability) * (h - groundHeight) / groundVariation
    * p = minProbability, p > groundHeight + groundVariation
    */
   public double computeProbability(ContactPointParticle particle)
   {
      double height = particle.getContactPointPosition().getZ();
      if (height <= GROUND_HEIGHT)
      {
         return 1.0;
      }
      else if (height > GROUND_HEIGHT + GROUND_VARIATION)
      {
         return MIN_PROBABILITY;
      }
      else
      {
         return 1.0 - (1.0 - MIN_PROBABILITY) * (height - GROUND_HEIGHT) / GROUND_VARIATION;
      }
   }
}
