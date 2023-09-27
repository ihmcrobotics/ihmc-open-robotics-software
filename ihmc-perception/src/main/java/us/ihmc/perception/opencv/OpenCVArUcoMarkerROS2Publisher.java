package us.ihmc.perception.opencv;

import gnu.trove.map.TIntObjectMap;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ArUcoMarkerPoses;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;

/**
 * Used to publish detected ArUco marker poses over ROS 2.
 * We send all the ArUco marker poses all in one message, so it can be
 * determined which ids got detected and which ones didn't, without
 * having to publish "detected == false" for non-detected markers --
 * they just won't be present in the message if not detected.
 *
 * We just send over the minimal amount of information without
 * associating the ArUco detection with specific objects. That
 * needs to be done on the receiving end if necessary.
 */
public class OpenCVArUcoMarkerROS2Publisher
{
   private final OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private final ArUcoMarkerPoses arUcoMarkerPoses = new ArUcoMarkerPoses();
   private final ROS2PublishSubscribeAPI ros2;
   private final TIntObjectMap<ArUcoMarkerNode> markerIDToNodeMap;

   public OpenCVArUcoMarkerROS2Publisher(OpenCVArUcoMarkerDetection arUcoMarkerDetection,
                                         ROS2PublishSubscribeAPI ros2,
                                         TIntObjectMap<ArUcoMarkerNode> markerIDToNodeMap)
   {
      this.arUcoMarkerDetection = arUcoMarkerDetection;
      this.ros2 = ros2;
      this.markerIDToNodeMap = markerIDToNodeMap;
   }

   public void update()
   {
      if (arUcoMarkerDetection.isEnabled())
      {
         synchronized (arUcoMarkerDetection.getSyncObject())
         {
            Mat ids = arUcoMarkerDetection.getIDsMat();
            arUcoMarkerPoses.getMarkerId().clear();
            arUcoMarkerPoses.getOrientation().clear();
            arUcoMarkerPoses.getPosition().clear();
            for (int i = 0; i < ids.rows(); i++)
            {
               int markerID = ids.ptr(i, 0).getInt();

               if (markerIDToNodeMap.containsKey(markerID))
               {
                  arUcoMarkerPoses.getMarkerId().add(markerID);

                  double markerSize = markerIDToNodeMap.get(markerID).getMarkerSize();
                  arUcoMarkerDetection.getPose(markerID,
                                               markerSize,
                                               ReferenceFrame.getWorldFrame(),
                                               arUcoMarkerPoses.getPosition().add(),
                                               arUcoMarkerPoses.getOrientation().add());
               }
            }
            ros2.publish(PerceptionAPI.ARUCO_MARKER_POSES, arUcoMarkerPoses);
         }
      }
   }
}
