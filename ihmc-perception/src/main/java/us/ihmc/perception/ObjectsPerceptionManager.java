package us.ihmc.perception;

import perception_msgs.msg.dds.ArUcoMarkerPoses;
import perception_msgs.msg.dds.DetectedObjectMessage;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.objects.*;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BiConsumer;

public class ObjectsPerceptionManager
{
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("objects_perception_manager");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_OBJECT
         = BASE_TOPIC.withType(DetectedObjectMessage.class).withSuffix("detected_object");

   private final IHMCROS2Input<ArUcoMarkerPoses> arUcoMarkerPosesSubscription;
   private final HashMap<Long, BiConsumer<Tuple3DReadOnly, QuaternionReadOnly>> markerUpdaters = new HashMap<>();
   private final DoorPerceptionManager pullDoorManager;
   private final DoorPerceptionManager pushDoorManager;
   private final ROS2Helper ros2;

   private final ArrayList<DetectedObjectPublisher> detectedObjectPublishers = new ArrayList<>();

   public ObjectsPerceptionManager(ReferenceFrame cameraFrame, ObjectInfo objectInfo)
   {
      ros2 = new ROS2Helper(DomainFactory.PubSubImplementation.FAST_RTPS, "objects_perception_manager");

      arUcoMarkerPosesSubscription = ros2.subscribe(ROS2Tools.ARUCO_MARKER_POSES);

      pullDoorManager = new DoorPerceptionManager(PULL_DOOR_MARKER_ID, "Pull", cameraFrame);
      pullDoorManager.getDoorFrame().setObjectTransformToMarker(transform -> transform.set(PULL_DOOR_FRAME_TRANSFORM_TO_MARKER));
      pullDoorManager.getDoorPanel().setObjectTransformToMarker(transform -> transform.set(PULL_DOOR_PANEL_TRANSFORM_TO_MARKER));
      markerUpdaters.put(PULL_DOOR_MARKER_ID, pullDoorManager::updateMarkerTransform);

      pushDoorManager = new DoorPerceptionManager(PUSH_DOOR_MARKER_ID, "Push", cameraFrame);
      pushDoorManager.getDoorFrame().setObjectTransformToMarker(transform -> transform.set(PUSH_DOOR_FRAME_TRANSFORM_TO_MARKER));
      pushDoorManager.getDoorPanel().setObjectTransformToMarker(transform -> transform.set(PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER));
      markerUpdaters.put(PUSH_DOOR_MARKER_ID, pushDoorManager::updateMarkerTransform);

      detectedObjectPublishers.add(new DetectedObjectPublisher(ros2,
                                                               DETECTED_OBJECT,
                                                               PULL_DOOR_MARKER_ID,
                                                               pullDoorManager.getDoorFrame().getObjectFrame()));
      detectedObjectPublishers.add(new DetectedObjectPublisher(ros2,
                                                               DETECTED_OBJECT,
                                                               PULL_DOOR_MARKER_ID,
                                                               pullDoorManager.getDoorPanel().getObjectFrame()));
      detectedObjectPublishers.add(new DetectedObjectPublisher(ros2,
                                                               DETECTED_OBJECT,
                                                               PUSH_DOOR_MARKER_ID,
                                                               pushDoorManager.getDoorFrame().getObjectFrame()));
      detectedObjectPublishers.add(new DetectedObjectPublisher(ros2,
                                                               DETECTED_OBJECT,
                                                               PUSH_DOOR_MARKER_ID,
                                                               pushDoorManager.getDoorPanel().getObjectFrame()));
   }

   public void update()
   {
      if (arUcoMarkerPosesSubscription.getMessageNotification().poll())
      {
         for (DetectedObjectPublisher detectedObjectPublisher : detectedObjectPublishers)
         {
            detectedObjectPublisher.reset();
         }

         ArUcoMarkerPoses arUcoMarkerPosesMessage = arUcoMarkerPosesSubscription.getMessageNotification().read();
         for (int i = 0; i < arUcoMarkerPosesMessage.getMarkerId().size(); i++)
         {
            var markerUpdater = markerUpdaters.get(arUcoMarkerPosesMessage.getMarkerId().get(i));
            if (markerUpdater != null)
            {
               markerUpdater.accept(arUcoMarkerPosesMessage.getPosition().get(i),
                                    arUcoMarkerPosesMessage.getOrientation().get(i));
            }

            for (DetectedObjectPublisher detectedObjectPublisher : detectedObjectPublishers)
            {
               detectedObjectPublisher.markDetected(arUcoMarkerPosesMessage.getMarkerId().get(i));
            }
         }

         for (DetectedObjectPublisher detectedObjectPublisher : detectedObjectPublishers)
         {
            detectedObjectPublisher.publish();
         }
      }
   }
}