package us.ihmc.perception;

import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ArUcoMarkerPoses;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.tools.thread.SwapReference;

import java.util.HashMap;
import java.util.List;

public class OpenCVArUcoMarkerROS2Publisher
{
   private final OpenCVArUcoMarkerDetection arUcoMarkerDetection;
   private final HashMap<Integer, OpenCVArUcoMarker> arUcoMarkersToTrack = new HashMap<>();
   private final FramePose3D framePoseOfMarker = new FramePose3D();
   private final ArUcoMarkerPoses arUcoMarkerPoses = new ArUcoMarkerPoses();
   private final ReferenceFrame cameraFrame;
   private final ROS2PublishSubscribeAPI ros2;

   public OpenCVArUcoMarkerROS2Publisher(OpenCVArUcoMarkerDetection arUcoMarkerDetection,
                                         List<OpenCVArUcoMarker> arUcoMarkersToTrack,
                                         ReferenceFrame cameraFrame,
                                         ROS2PublishSubscribeAPI ros2)
   {
      this.arUcoMarkerDetection = arUcoMarkerDetection;
      this.cameraFrame = cameraFrame;
      this.ros2 = ros2;

      for (OpenCVArUcoMarker openCVArUcoMarker : arUcoMarkersToTrack)
      {
         this.arUcoMarkersToTrack.put(openCVArUcoMarker.getId(), openCVArUcoMarker);
      }
   }

   public void update()
   {
      SwapReference<Mat> ids = arUcoMarkerDetection.getIds();
      arUcoMarkerPoses.getMarkerId().clear();
      arUcoMarkerPoses.getOrientation().clear();
      arUcoMarkerPoses.getPosition().clear();
      for (int i = 0; i < ids.getForThreadTwo().rows(); i++)
      {
         int markerID = ids.getForThreadTwo().ptr(i, 0).getInt();
         OpenCVArUcoMarker markerToTrack = arUcoMarkersToTrack.get(markerID);

         if (markerToTrack != null)
         {
            framePoseOfMarker.setIncludingFrame(cameraFrame, arUcoMarkerDetection.getPose(markerToTrack));
            framePoseOfMarker.changeFrame(ReferenceFrame.getWorldFrame());

            arUcoMarkerPoses.getMarkerId().add(markerID);
            arUcoMarkerPoses.getOrientation().add().set(framePoseOfMarker.getOrientation());
            arUcoMarkerPoses.getPosition().add().set(framePoseOfMarker.getX(), framePoseOfMarker.getY(), framePoseOfMarker.getZ());
         }
      }

      ros2.publish(ROS2Tools.ARUCO_MARKER_POSES, arUcoMarkerPoses);
   }
}