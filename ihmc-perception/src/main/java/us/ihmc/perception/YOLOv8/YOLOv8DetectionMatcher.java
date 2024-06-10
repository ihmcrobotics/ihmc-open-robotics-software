package us.ihmc.perception.YOLOv8;

import us.ihmc.perception.RawImage;
import us.ihmc.perception.filters.DetectionFilter;
import us.ihmc.perception.opencl.OpenCLDepthImageSegmenter;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeAddition;
import us.ihmc.tools.time.FrequencyCalculator;

import java.util.AbstractMap.SimpleEntry;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

public class YOLOv8DetectionMatcher
{
   private static final double DEFAULT_FILTER_HISTORY = 20.0;
   private static final float DEFAULT_ACCEPTANCE_THRESHOLD = 0.2f;

   private final OpenCLPointCloudExtractor extractor = new OpenCLPointCloudExtractor();
   private final OpenCLDepthImageSegmenter segmenter = new OpenCLDepthImageSegmenter();

   private final Map<YOLOv8SegmentedDetection, DetectionFilter> existingDetections = new HashMap<>();
   private final Map<YOLOv8SegmentedDetection, YOLOv8Node> detectionToNodeMap = new HashMap<>();
   private final Map<YOLOv8SegmentedDetection, DetectionFilter> candidateDetections = new HashMap<>();

   private final FrequencyCalculator updateFrequencyCalculator = new FrequencyCalculator();

   private final Object existingDetectionsSyncObject = new Object();
   private final Object candidateDetectionsSyncObject = new Object();

   public void matchDetections(Map<YOLOv8Detection, RawImage> newDetections,
                               RawImage depthImage,
                               double distanceThreshold,
                               int erosionKernelRadius,
                               float zScoreThreshold,
                               float candidateAcceptanceThreshold)
   {
      // Get the set of new detections segmented
      Set<YOLOv8SegmentedDetection> newSegmentedDetections = new HashSet<>();
      for (Entry<YOLOv8Detection, RawImage> entry : newDetections.entrySet())
      {
         newSegmentedDetections.add(new YOLOv8SegmentedDetection(entry.getKey(),
                                                                 entry.getValue(),
                                                                 depthImage,
                                                                 erosionKernelRadius,
                                                                 segmenter::removeBackground,
                                                                 extractor::extractPointCloud,
                                                                 zScoreThreshold));
      }

      synchronized (existingDetectionsSyncObject)
      {
         // Match existing and new detections
         Map<YOLOv8SegmentedDetection, YOLOv8SegmentedDetection> detectionMatches = matchDetections(existingDetections.keySet(),
                                                                                                    newSegmentedDetections,
                                                                                                    distanceThreshold);
         // Remove the new detections that matched to existing detections
         newSegmentedDetections.removeIf(detectionMatches::containsValue);

         // Replace the existing detections with the new one, and register the detection
         detectionMatches.forEach((existingDetection, newDetection) ->
         {
            DetectionFilter filter = existingDetections.remove(existingDetection);
            if (filter != null)
            {
               filter.registerDetection(newDetection.getDetection().confidence());
               existingDetections.put(newDetection, filter);
               detectionToNodeMap.put(newDetection, detectionToNodeMap.remove(existingDetection));
            }
         });
      }

      synchronized (candidateDetectionsSyncObject)
      {
         // Match remaining new detections & candidate detections
         Map<YOLOv8SegmentedDetection, YOLOv8SegmentedDetection> candidateMatches = matchDetections(candidateDetections.keySet(),
                                                                                                    newSegmentedDetections,
                                                                                                    distanceThreshold);
         // Remove the new detections that matched to candidates
         newSegmentedDetections.removeIf(candidateMatches::containsValue);

         // Replace the old candidate detections with new ones, and register the detection
         candidateMatches.forEach((oldDetection, newDetection) ->
         {
            DetectionFilter filter = candidateDetections.remove(oldDetection);
            if (filter != null)
            {
               filter.registerDetection(newDetection.getDetection().confidence());
               candidateDetections.put(newDetection, filter);
            }
         });

         // Only unmatched new detection remain; add as new candidate detections
         newSegmentedDetections.forEach(newDetection ->
         {
            DetectionFilter newFilter = new DetectionFilter((int) updateFrequencyCalculator.getFrequencyOrDefault(DEFAULT_FILTER_HISTORY),
                                                            candidateAcceptanceThreshold);
            newFilter.registerDetection(newDetection.getDetection().confidence());
            candidateDetections.put(newDetection, newFilter);
         });
      }
   }

   public void updateSceneGraph(SceneGraph sceneGraph)
   {
      updateFrequencyCalculator.ping();

      // Handle candidate detections
      sceneGraph.modifyTree(modificationQueue ->
      {
         synchronized (candidateDetectionsSyncObject)
         {
            // accept or reject candidate detections
            Iterator<Entry<YOLOv8SegmentedDetection, DetectionFilter>> candidateIterator = candidateDetections.entrySet().iterator();
            while (candidateIterator.hasNext())
            {
               Entry<YOLOv8SegmentedDetection, DetectionFilter> candidateEntry = candidateIterator.next();
               YOLOv8SegmentedDetection candidate = candidateEntry.getKey();
               DetectionFilter filter = candidateEntry.getValue();
               filter.setHistoryLength((int) updateFrequencyCalculator.getFrequencyOrDefault(DEFAULT_FILTER_HISTORY));
               filter.update();

               // Check if detection has been candidate long enough
               if (filter.hasEnoughSamples())
               {
                  // if the candidate has stable detection, add it to existing detections
                  if (filter.isStableDetectionResult())
                  {

                     long newNodeID = sceneGraph.getNextID().getAndIncrement();
                     String newNodeName = candidate.getDetection().objectClass().getDefaultNodeName() + newNodeID;
                     YOLOv8Node newYoloNode = new YOLOv8Node(newNodeID,
                                                             newNodeName,
                                                             candidate.getDetection(),
                                                             candidate.getObjectPointCloud(),
                                                             candidate.getCentroid());
                     modificationQueue.accept(new SceneGraphNodeAddition(newYoloNode, sceneGraph.getRootNode()));

                     existingDetections.put(candidate, filter);
                     detectionToNodeMap.put(candidate, newYoloNode);

                     filter.setAcceptanceThreshold(DEFAULT_ACCEPTANCE_THRESHOLD);
                  }

                  // remove the candidate
                  candidateIterator.remove();
               }
            }
         }
      });

      synchronized (existingDetectionsSyncObject)
      {
         // Handle existing detections
         Iterator<Entry<YOLOv8SegmentedDetection, YOLOv8Node>> detectionToNodeMapIterator = detectionToNodeMap.entrySet().iterator();
         while (detectionToNodeMapIterator.hasNext())
         {
            Entry<YOLOv8SegmentedDetection, YOLOv8Node> entry = detectionToNodeMapIterator.next();
            YOLOv8SegmentedDetection detection = entry.getKey();
            YOLOv8Node yoloNode = entry.getValue();

            // Node may have been removed by user
            if (!sceneGraph.getIDToNodeMap().containsKey(yoloNode.getID()))
            {
               detectionToNodeMapIterator.remove();
               existingDetections.remove(detection);
            }
            else // Node still exists; update
            {
               DetectionFilter filter = existingDetections.get(detection);
               filter.setAcceptanceThreshold(yoloNode.getDetectionAcceptanceThreshold());
               filter.update();

               yoloNode.setDetection(detection.getDetection());
               yoloNode.setObjectPointCloud(detection.getObjectPointCloud());
               yoloNode.setObjectCentroid(detection.getCentroid());
               yoloNode.setCurrentlyDetected(filter.isStableDetectionResult());
               yoloNode.update();
            }
         }
      }
   }

   /**
    * <p>
    *    Matches old and new detections based on distance.
    *    Detections are matched when they are within a distance threshold,
    *    and matches are prioritized based on how close (in distance) the old and new detections are.
    * </p>
    *
    * @param oldDetections set of old detections
    * @param newDetections set of new detections
    * @param distanceThreshold distance threshold in meters.
    *                          The distance between old and new detections' centroids are compared to the threshold.
    * @return map of matching old detections (keys) and new detections (values)
    */
   private Map<YOLOv8SegmentedDetection, YOLOv8SegmentedDetection> matchDetections(Set<YOLOv8SegmentedDetection> oldDetections,
                                                                                   Set<YOLOv8SegmentedDetection> newDetections,
                                                                                   double distanceThreshold)
   {
      Map<YOLOv8SegmentedDetection, YOLOv8SegmentedDetection> matchingDetections = new HashMap<>();
      double distanceThresholdSquared = distanceThreshold * distanceThreshold;

      // Possible matches queued based of distance between the old and new detections
      PriorityQueue<Map.Entry<YOLOv8SegmentedDetection, YOLOv8SegmentedDetection>> possibleMatches
            = new PriorityQueue<>(Comparator.comparingDouble(entry -> entry.getKey().getCentroid().distanceSquared(entry.getValue().getCentroid())));

      // Compare all old and new detections
      for (YOLOv8SegmentedDetection oldDetection : oldDetections)
      {
         for (YOLOv8SegmentedDetection  newDetection : newDetections)
         {
            // if they have the same class
            if (oldDetection.getDetection().objectClass() == newDetection.getDetection().objectClass())
            {
               // calculate distance & compare to threshold
               double distanceSquared = oldDetection.getCentroid().distanceSquared(newDetection.getCentroid());
               if (distanceSquared < distanceThresholdSquared)
                  possibleMatches.add(new SimpleEntry<>(oldDetection, newDetection)); // add to possible matches if within distance threshold
            }
         }
      }

      // Keep track of unmatched old and new detections
      Set<YOLOv8SegmentedDetection> unmatchedOldDetections = new HashSet<>(oldDetections);
      Set<YOLOv8SegmentedDetection> unmatchedNewDetections = new HashSet<>(newDetections);

      // While unmatched detections exist
      while (!unmatchedOldDetections.isEmpty() && !unmatchedNewDetections.isEmpty())
      {
         // and while possible matches exist
         Map.Entry<YOLOv8SegmentedDetection, YOLOv8SegmentedDetection> bestMatch = possibleMatches.poll();
         if (bestMatch == null)
            break;

         if (unmatchedOldDetections.contains(bestMatch.getKey()) && unmatchedNewDetections.contains(bestMatch.getValue()))
         {
            // add the match to the map
            matchingDetections.put(bestMatch.getKey(), bestMatch.getValue());
            unmatchedOldDetections.remove(bestMatch.getKey());
            unmatchedNewDetections.remove(bestMatch.getValue());
         }
      }

      return matchingDetections;
   }

   public void destroy()
   {
      extractor.destroy();
      segmenter.destroy();
   }
}
