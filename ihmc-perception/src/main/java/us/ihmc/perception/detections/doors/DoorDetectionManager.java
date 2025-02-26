package us.ihmc.perception.detections.doors;

import perception_msgs.msg.dds.DetectedDoorListMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

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
   // Parameters
   private static final double MAX_DETECTION_JUMP_DISTANCE = 0.75;
   static final double DETECTION_EXPIRATION_SECONDS = 2.0;

   static final String DOOR_STRING = "door";
   static final String PANEL_STRING = "door_panel";

   private static final double PUBLISH_FREQUENCY = 15.0;

   // ROS2
   private final ROS2Publisher<DetectedDoorListMessage> detectedDoorsPublisher;
   private final DetectedDoorListMessage detectedDoorsMessage;
   private final Throttler publishThrottler;

   // Detections
   private final List<DetectedDoor> detectedDoors = new LinkedList<>();

   public DoorDetectionManager(ROS2Node ros2Node)
   {
      detectedDoorsPublisher = ros2Node.createPublisher(PerceptionAPI.DETECTED_DOORS);
      detectedDoorsMessage = new DetectedDoorListMessage();
      publishThrottler = new Throttler().setFrequency(PUBLISH_FREQUENCY);
   }

   public List<DetectedDoor> getDetectedDoors()
   {
      return new LinkedList<>(detectedDoors);
   }

   public synchronized void update()
   {
      // Remove old detections
      detectedDoors.removeIf(doorDetection -> doorDetection.getLastDetectedTime()
                                                           .plusNanos(Conversions.secondsToNanoseconds(DETECTION_EXPIRATION_SECONDS))
                                                           .isBefore(Instant.now()));
      // Publish detected doors message
      if (publishThrottler.run())
      {
         detectedDoorsMessage.getDetectedDoors().clear();
         for (int i = 0; i < detectedDoors.size() && i < detectedDoorsMessage.getDetectedDoors().getCurrentCapacity(); ++i)
         {
            DetectedDoor detectedDoor = detectedDoors.get(i);
            detectedDoor.toMessage(detectedDoorsMessage.getDetectedDoors().add());
         }
         detectedDoorsPublisher.publish(detectedDoorsMessage);
      }
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
      double maxDistanceSquared = MAX_DETECTION_JUMP_DISTANCE * MAX_DETECTION_JUMP_DISTANCE;

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
