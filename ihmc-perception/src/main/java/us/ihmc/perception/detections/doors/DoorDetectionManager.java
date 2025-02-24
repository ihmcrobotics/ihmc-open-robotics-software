package us.ihmc.perception.detections.doors;

import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Set;

public class DoorDetectionManager
{
   static final String DOOR_STRING = "door";
   static final String PANEL_STRING = "door_panel";

   // Detections
   private final List<DetectedDoor> detectedDoors = new LinkedList<>();

   // Parameters
   private double maxDetectionJumpDistance = 0.5;

   public DoorDetectionManager()
   {

   }

   public List<DetectedDoor> getDetectedDoors()
   {
      return new LinkedList<>(detectedDoors);
   }

   public synchronized void update()
   {
      // Remove old detections
      detectedDoors.removeIf(doorDetection -> doorDetection.getLastDetectedTime().plusSeconds(2).isBefore(Instant.now()));
   }

   public synchronized void registerNewDetections(List<InstantDetection> newDetections)
   {
      // Find opening hardware and panel detections
      List<InstantDetection> openingHardwareDetections = new ArrayList<>();
      List<InstantDetection> panelDetections = new ArrayList<>();
      for (InstantDetection newDetection : newDetections)
      {
         String objectClass = newDetection.getDetectedObjectClass();
         if (objectClass.contains(PANEL_STRING))
            panelDetections.add(newDetection);
         else if (objectClass.startsWith(DOOR_STRING))
            openingHardwareDetections.add(newDetection);
      }

      // Update opening hardware detections
      registerNewDetectionsInternal(openingHardwareDetections);

      // Update panel detections
      registerNewDetectionsInternal(panelDetections);
   }

   private void registerNewDetectionsInternal(List<InstantDetection> newDetections)
   {
      double maxDistanceSquared = maxDetectionJumpDistance * maxDetectionJumpDistance;

      // Create a priority queue of potential matches
      PriorityQueue<Entry<DetectedDoor, InstantDetection>> potentialMatches = new PriorityQueue<>(Comparator.comparingDouble(entry -> entry.getKey()
                                                                                                                                           .distanceSquared(
                                                                                                                                                 entry.getValue())));
      for (DetectedDoor doorDetection : detectedDoors)
      {
         for (InstantDetection newDetection : newDetections)
         {
            double distanceSquared = doorDetection.distanceSquared(newDetection);
            if (distanceSquared < maxDistanceSquared)
               potentialMatches.add(Map.entry(doorDetection, newDetection));
         }
      }

      // Update matched detections
      Set<DetectedDoor> unmatchedDoorDetections = new HashSet<>(detectedDoors);
      Set<InstantDetection> unmatchedNewDetections = new HashSet<>(newDetections);
      while (!unmatchedDoorDetections.isEmpty() && !unmatchedNewDetections.isEmpty() && !potentialMatches.isEmpty())
      {
         Entry<DetectedDoor, InstantDetection> nextBestMatch = potentialMatches.poll();
         DetectedDoor doorDetection = nextBestMatch.getKey();
         InstantDetection openingHardwareDetection = nextBestMatch.getValue();

         if (unmatchedDoorDetections.contains(doorDetection) && unmatchedNewDetections.contains(openingHardwareDetection))
         {
            doorDetection.updateDetection(openingHardwareDetection);
            unmatchedDoorDetections.remove(doorDetection);
            unmatchedNewDetections.remove(openingHardwareDetection);
         }
      }

      for (InstantDetection unmatchedNewDetection : unmatchedNewDetections)
      {
         DetectedDoor newlyDetectedDoor = new DetectedDoor();
         newlyDetectedDoor.updateDetection(unmatchedNewDetection);
         detectedDoors.add(newlyDetectedDoor);
      }
   }

   public synchronized void updatePlanarRegions(FramePlanarRegionsList newPlanarRegions)
   {
      PlanarRegionsList planarRegionsList = newPlanarRegions.getPlanarRegionsList();
      planarRegionsList.applyTransform(newPlanarRegions.getSensorToWorldFrameTransform());

      for (DetectedDoor doorDetection : detectedDoors)
         doorDetection.updatePlanarRegion(planarRegionsList);
   }
}
