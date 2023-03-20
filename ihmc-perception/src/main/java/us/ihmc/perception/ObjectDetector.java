package us.ihmc.perception;

import perception_msgs.msg.dds.ArUcoMarkerPoses;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.perception.objects.ArUcoMarkerObject;
import us.ihmc.perception.objects.ObjectInfo;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BiConsumer;

public class ObjectDetector
{
   private final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("object_detector");
   public final ROS2Topic<ArUcoMarkerPoses> ARUCO_MARKER_POSES = BASE_TOPIC.withType(ArUcoMarkerPoses.class).withSuffix("aruco_marker_poses");

   private final IHMCROS2Input<ArUcoMarkerPoses> arUcoMarkerPosesSubscription;
   private final ROS2Helper ros2;
   private final ArrayList<OpenCVArUcoMarker> arUcoMarkersToTrack;
   private boolean objectDetected = false;
   private ArUcoMarkerObject objectWithArUcoMarker;
   private ObjectInfo objectInfo;
   private String objectName = "";

   public ObjectDetector(ObjectInfo objectInfo, ArrayList<OpenCVArUcoMarker> arUcoMarkersToTrack)
   {
      ros2 = new ROS2Helper(DomainFactory.PubSubImplementation.FAST_RTPS, "object_detector");
      arUcoMarkerPosesSubscription = ros2.subscribe(ARUCO_MARKER_POSES);

      this.arUcoMarkersToTrack = arUcoMarkersToTrack;
      this.objectInfo = objectInfo;
   }

   public void update()
   {
      if (arUcoMarkerPosesSubscription.getMessageNotification().poll())
      {
         ArUcoMarkerPoses arUcoMarkerPosesMessage = arUcoMarkerPosesSubscription.getMessageNotification().read();
         if (arUcoMarkerPosesMessage.getMarkerId().size() > 0)
            objectDetected = true;
         else
            objectDetected = false;
         for (int i = 0; i < arUcoMarkerPosesMessage.getMarkerId().size(); i++)
         {
            int objectId = (int) arUcoMarkerPosesMessage.getMarkerId().get(i);
            var arUcoMarker = arUcoMarkersToTrack.get(objectId);
            if (arUcoMarker != null)
            {
               objectName = objectInfo.getObjectName(objectId);
               //TODO - EXTENSION TO SIMULTANEOUS DETECTION MULTIPLE OBJECTS
               // if multiple objects detected,
               // use VR eye tracking to see what we are focusing on (closer object to where the eye is focusing)
               //               objectWithArUcoMarker.add(new ArUcoMarkerObject(marker.getId(),arUcoInfo)); // get object with attached marker
               objectWithArUcoMarker = new ArUcoMarkerObject(objectId, objectInfo);
               // get marker pose in camera frame
               FramePose3DBasics markerPose = new FramePose3D();
               markerPose.getPosition().set(arUcoMarkerPosesMessage.getPosition().get(i));
               markerPose.getOrientation().set(arUcoMarkerPosesMessage.getOrientation().get(i));
               // transform in world frame
               markerPose.changeFrame(ReferenceFrame.getWorldFrame());
               // set transform MarkerToWorld to markerPose
               markerPose.get(objectWithArUcoMarker.getMarkerToWorld());
               // update frame of the object
               objectWithArUcoMarker.updateFrame();
               // compute object pose from marker pose
               objectWithArUcoMarker.computeObjectPose(markerPose);
            }
         }
      }
   }

   public String getObjectName()
   {
      return objectName;
   }

   public FramePose3D getObjectPose()
   {
      return objectWithArUcoMarker.getObjectPose();
   }

   public FramePose3D getObjectAppendixPose()
   {
      return objectWithArUcoMarker.getAppendixPose();
   }

   public ReferenceFrame getObjectFrame()
   {
      return objectWithArUcoMarker.getObjectFrame();
   }

   public boolean hasDetectedObject()
   {
      return objectDetected;
   }
}
