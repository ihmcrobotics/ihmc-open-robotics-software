package us.ihmc.perception.detections;

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

   public void confirmDetectionMatch()
   {
      persistentDetection.addDetection(instantDetection);
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
