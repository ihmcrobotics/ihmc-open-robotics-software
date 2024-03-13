package us.ihmc.perception.opencv;

import gnu.trove.iterator.TIntObjectIterator;
import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import gnu.trove.set.hash.TIntHashSet;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ArUcoMarkerPoses;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.filters.DetectionFilter;
import us.ihmc.tools.time.FrequencyCalculator;

import java.util.function.Function;

/**
 * Used to publish detected ArUco marker poses over ROS 2.
 * We send all the ArUco marker poses all in one message, so it can be
 * determined which ids got detected and which ones didn't, without
 * having to publish "detected == false" for non-detected markers --
 * they just won't be present in the message if not detected.
 *
 * We just send over the minimal amount of information without
 * associating the ArUco detection with specific objects.
 *
 * We also apply a detection filter so intermittent noisy detections
 * don't get published.
 */
public class OpenCVArUcoMarkerROS2Publisher
{
   private final OpenCVArUcoMarkerDetectionResults arUcoMarkerDetectionResults;
   private final ArUcoMarkerPoses arUcoMarkerPoses = new ArUcoMarkerPoses();
   private final ROS2PublishSubscribeAPI ros2;
   private final Function<Integer, Double> markerSizes;
   private final ReferenceFrame sensorFrame;
   private final TIntObjectMap<DetectionFilter> detectionFilters = new TIntObjectHashMap<>();
   private final TIntHashSet stableIDs = new TIntHashSet();
   private final FrequencyCalculator updateFrequencyCalculator = new FrequencyCalculator();

   public OpenCVArUcoMarkerROS2Publisher(OpenCVArUcoMarkerDetectionResults arUcoMarkerDetectionResults,
                                         ROS2PublishSubscribeAPI ros2,
                                         Function<Integer, Double> markerSizes,
                                         ReferenceFrame sensorFrame)
   {
      this.arUcoMarkerDetectionResults = arUcoMarkerDetectionResults;
      this.ros2 = ros2;
      this.markerSizes = markerSizes;
      this.sensorFrame = sensorFrame;
   }

   public void update()
   {
      updateFrequencyCalculator.ping();

      Mat ids = arUcoMarkerDetectionResults.getIDs();
      arUcoMarkerPoses.getMarkerId().clear();
      arUcoMarkerPoses.getOrientation().clear();
      arUcoMarkerPoses.getPosition().clear();
      for (int i = 0; i < ids.rows(); i++)
      {
         int markerID = ids.ptr(i, 0).getInt();

         DetectionFilter detectionFilter = detectionFilters.get(markerID);

         if (detectionFilter == null)
         {
            detectionFilter = new DetectionFilter();
            detectionFilters.put(markerID, detectionFilter);
         }

         // FIXME: Edge case if updateFrequencyCalculator has 0 or 1 pings.
         detectionFilter.setHistoryLength((int) updateFrequencyCalculator.getFrequency());
         detectionFilter.registerDetection();

         if (detectionFilter.isStableDetectionResult())
         {
            stableIDs.add(markerID);
         }

         if (stableIDs.contains(markerID))
         {
            arUcoMarkerPoses.getMarkerId().add(markerID);

            double markerSize = markerSizes.apply(markerID);
            arUcoMarkerDetectionResults.getPose(markerID,
                                                markerSize,
                                                sensorFrame,
                                                ReferenceFrame.getWorldFrame(),
                                                arUcoMarkerPoses.getPosition().add(),
                                                arUcoMarkerPoses.getOrientation().add());
         }
      }
      ros2.publish(PerceptionAPI.ARUCO_MARKER_POSES, arUcoMarkerPoses);

      // Update detection filters
      for (TIntObjectIterator<DetectionFilter> iterator = detectionFilters.iterator(); iterator.hasNext(); )
      {
         iterator.advance();
         DetectionFilter detectionFilter = iterator.value();
         detectionFilter.update();
      }
   }
}
