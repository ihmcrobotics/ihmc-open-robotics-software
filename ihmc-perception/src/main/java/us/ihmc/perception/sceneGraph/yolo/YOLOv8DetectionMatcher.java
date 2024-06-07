package us.ihmc.perception.sceneGraph.yolo;

import us.ihmc.perception.RawImage;
import us.ihmc.perception.YOLOv8.YOLOv8Detection;
import us.ihmc.perception.filters.DetectionFilter;
import us.ihmc.perception.opencl.OpenCLDepthImageSegmenter;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.perception.sceneGraph.SceneGraph;
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

public class YOLOv8DetectionMatcher
{
   private static final int DEFAULT_FILTER_HISTORY = 20;
   private static final float DEFAULT_ACCPETANCE_THRESHOLD = 0.2f;

   private final OpenCLPointCloudExtractor extractor = new OpenCLPointCloudExtractor();
   private final OpenCLDepthImageSegmenter segmenter = new OpenCLDepthImageSegmenter();

   private final Map<YOLOv8SegmentedDetection, DetectionFilter> existingDetections = new HashMap<>();
   private final Map<YOLOv8SegmentedDetection, YOLOv8Node> detectionToNodeMap = new HashMap<>();
   private final Map<YOLOv8SegmentedDetection, DetectionFilter> candidateDetections = new HashMap<>();

   private long numberOfFrequencyPings = 0L;
   private final FrequencyCalculator updateFrequencyCalculator = new FrequencyCalculator();

   public void matchDetections(Map<YOLOv8Detection, RawImage> newDetections,
                               RawImage depthImage,
                               double distanceThreshold,
                               int erosionKernelRadius,
                               float zScoreThreshold,
                               float candidateAcceptanceThreshold,
                               SceneGraph sceneGraph)
   {
      updateFrequencyCalculator.ping();
      numberOfFrequencyPings++;

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
            filter.update();
            existingDetections.put(newDetection, filter);
         }
      });

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
            filter.update();
            candidateDetections.put(newDetection, filter);
         }
      });

      // Only unmatched new detection remain; add as new candidate detections
      newSegmentedDetections.forEach(newDetection ->
      {
         DetectionFilter newFilter = new DetectionFilter(numberOfFrequencyPings > 5 ? (int) updateFrequencyCalculator.getFrequency() : DEFAULT_FILTER_HISTORY,
                                                         candidateAcceptanceThreshold);
         newFilter.registerDetection(newDetection.getDetection().confidence());
         candidateDetections.put(newDetection, newFilter);
      });

      // accept or reject candidate detections
      Iterator<Entry<YOLOv8SegmentedDetection, DetectionFilter>> candidateIterator = candidateDetections.entrySet().iterator();
      while (candidateIterator.hasNext())
      {
         Entry<YOLOv8SegmentedDetection, DetectionFilter> candidateEntry = candidateIterator.next();
         YOLOv8SegmentedDetection candidate = candidateEntry.getKey();
         DetectionFilter filter = candidateEntry.getValue();

         // Check if detection has been candidate long enough
         if (filter.hasEnoughSamples())
         {
            // if the candidate has stable detection, add it to existing detections
            if (filter.isStableDetectionResult())
            {
               existingDetections.put(candidate, filter);

               long newNodeID = sceneGraph.getNextID().getAndIncrement();
               String newNodeName = candidate.getDetection().objectClass().getDefaultNodeName() + newNodeID;
               YOLOv8Node newYoloNode = new YOLOv8Node(newNodeID,
                                                       newNodeName,
                                                       candidate.getDetection(),
                                                       candidate.getObjectPointCloud(),
                                                       candidate.getCentroid());
               detectionToNodeMap.put(candidate, newYoloNode);

               filter.setAcceptanceThreshold(DEFAULT_ACCPETANCE_THRESHOLD);
            }

            // remove the candidate
            candidateIterator.remove();
         }
      }
   }

   public void updateSceneGraph(SceneGraph sceneGraph)
   {
      Set<YOLOv8Node> yoloNodesInScene = getYOLONodes(sceneGraph);
      for (YOLOv8Node node : yoloNodesInScene)
      {
         nod
      }

   }

   private Map<YOLOv8SegmentedDetection, YOLOv8SegmentedDetection> matchDetections(Set<YOLOv8SegmentedDetection> oldDetections,
                                                                                   Set<YOLOv8SegmentedDetection> newDetections,
                                                                                   double distanceThreshold)
   {
      Map<YOLOv8SegmentedDetection, YOLOv8SegmentedDetection> matchingDetections = new HashMap<>();
      double distanceThresholdSquared = distanceThreshold * distanceThreshold;

      PriorityQueue<Map.Entry<YOLOv8SegmentedDetection, YOLOv8SegmentedDetection>> possibleMatches
            = new PriorityQueue<>(Comparator.comparingDouble(entry -> entry.getKey().getCentroid().distanceSquared(entry.getValue().getCentroid())));

      for (YOLOv8SegmentedDetection oldDetection : oldDetections)
      {
         for (YOLOv8SegmentedDetection  newDetection : newDetections)
         {
            if (oldDetection.getDetection().objectClass() == newDetection.getDetection().objectClass())
            {
               double distanceSquared = oldDetection.getCentroid().distanceSquared(newDetection.getCentroid());
               if (distanceSquared < distanceThresholdSquared)
                  possibleMatches.add(new SimpleEntry<>(oldDetection, newDetection));
            }
         }
      }

      Set<YOLOv8SegmentedDetection> unmatchedOldDetections = new HashSet<>(oldDetections);
      Set<YOLOv8SegmentedDetection> unmatchedNewDetections = new HashSet<>(newDetections);
      while (!unmatchedOldDetections.isEmpty() && !unmatchedNewDetections.isEmpty())
      {
         Map.Entry<YOLOv8SegmentedDetection, YOLOv8SegmentedDetection> bestMatch = possibleMatches.poll();
         if (bestMatch == null)
            break;

         if (unmatchedOldDetections.contains(bestMatch.getKey()) && unmatchedNewDetections.contains(bestMatch.getValue()))
         {
            matchingDetections.put(bestMatch.getKey(), bestMatch.getValue());
            unmatchedOldDetections.remove(bestMatch.getKey());
            unmatchedNewDetections.remove(bestMatch.getValue());
         }
      }

      return matchingDetections;
   }


   private Set<YOLOv8Node> getYOLONodes(SceneGraph sceneGraph)
   {
      Set<YOLOv8Node> yoloNodes = new HashSet<>();
      sceneGraph.getIDToNodeMap().forEachEntry((id, sceneNode) ->
      {
         if (sceneNode instanceof YOLOv8Node yoloNode)
            yoloNodes.add(yoloNode);

         return true;
      });

      return yoloNodes;
   }
}
