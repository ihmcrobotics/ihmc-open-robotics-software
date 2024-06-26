package us.ihmc.perception.detections;

/**
 * Used to sort matches of instant and persistent detections by proximity.
 */
public class DetectionPair implements Comparable<DetectionPair>
{
   private final PersistentDetection persistentDetection;
   private final InstantDetection instantDetection;

   public DetectionPair(PersistentDetection persistentDetection, InstantDetection instantDetection)
   {
      this.persistentDetection = persistentDetection;
      this.instantDetection = instantDetection;
   }

   public double getDistanceSquared()
   {
      return persistentDetection.getMostRecentDetection().getPose().getPosition().distanceSquared(instantDetection.getPose().getPosition());
   }

   @Override
   public int compareTo(DetectionPair other)
   {
      return Double.compare(getDistanceSquared(), other.getDistanceSquared());
   }

   public PersistentDetection getPersistentDetection()
   {
      return persistentDetection;
   }

   public InstantDetection getInstantDetection()
   {
      return instantDetection;
   }
}
