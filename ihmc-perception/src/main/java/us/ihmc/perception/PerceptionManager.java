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
import us.ihmc.perception.objects.DetectedObjectPublisher;
import us.ihmc.perception.objects.DoorPerceptionManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BiConsumer;

public class PerceptionManager
{
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("perception_manager");
   public static final ROS2Topic<ArUcoMarkerPoses> ARUCO_MARKER_POSES = BASE_TOPIC.withType(ArUcoMarkerPoses.class).withSuffix("aruco_marker_poses");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_PULL_DOOR_FRAME
         = BASE_TOPIC.withType(DetectedObjectMessage.class).withSuffix("detected_pull_door_frame");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_PULL_DOOR_PANEL
         = BASE_TOPIC.withType(DetectedObjectMessage.class).withSuffix("detected_pull_door_panel");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_PUSH_DOOR_FRAME
         = BASE_TOPIC.withType(DetectedObjectMessage.class).withSuffix("detected_push_door_frame");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_PUSH_DOOR_PANEL
         = BASE_TOPIC.withType(DetectedObjectMessage.class).withSuffix("detected_push_door_panel");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_BOX = BASE_TOPIC.withType(DetectedObjectMessage.class).withSuffix("detected_box");

   public static final double REAL_MARKER_WIDTH = 0.1680;

   /** This refers to the edges of the black parts with no margin. The margins included will be wider than this. */
   public static final double DOOR_MARKER_WIDTH = 0.2032;

   public static final long PULL_DOOR_MARKER_ID = 0;
   public static final RigidBodyTransform PULL_DOOR_FRAME_TRANSFORM_TO_MARKER = new RigidBodyTransform(
         new YawPitchRoll(Math.toRadians(180.0), 0.0, Math.toRadians(180.0)),
         new Point3D(0.0, -0.678702 - 0.005 - 0.006, 1.14141 + 0.02)
   );
   public static final RigidBodyTransform PULL_DOOR_PANEL_TRANSFORM_TO_MARKER = new RigidBodyTransform(
         new YawPitchRoll(Math.toRadians(180.0), 0.0, Math.toRadians(180.0)),
         new Point3D(0.0, -0.678702, 1.14141)
   );

   public static final long PUSH_DOOR_MARKER_ID = 1;
   public static final RigidBodyTransform PUSH_DOOR_FRAME_TRANSFORM_TO_MARKER = new RigidBodyTransform(
         new YawPitchRoll(0.0, 0.0, Math.toRadians(180.0)),
         new Point3D(0.0, 0.678702 + 0.005 - 0.006, 1.14141 + 0.02)
   );
   public static final RigidBodyTransform PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER = new RigidBodyTransform(
         new YawPitchRoll(0.0, 0.0, Math.toRadians(180.0)),
         new Point3D(0.0, 0.678702, 1.14141)
   );

   public static final long BOX_MARKER_ID = 2;
   public static final double BOX_MARKER_WIDTH = 0.210101;
   public static final RigidBodyTransform BOX_TRANSFORM_TO_MARKER = new RigidBodyTransform(
         new YawPitchRoll(Math.toRadians(180.0), Math.toRadians(0.0), Math.toRadians(-90.0)),
         new Point3D(0.07, 0.15, 0.17)
   );

   private final IHMCROS2Input<ArUcoMarkerPoses> arUcoMarkerPosesSubscription;
   private final HashMap<Long, BiConsumer<Tuple3DReadOnly, QuaternionReadOnly>> markerUpdaters = new HashMap<>();
   private final DoorPerceptionManager pullDoorManager;
   private final DoorPerceptionManager pushDoorManager;
   private final ArUcoMarkerObject box = new ArUcoMarkerObject(BOX_MARKER_ID, "Box");
   private final ROS2Helper ros2;

   private final ArrayList<DetectedObjectPublisher> detectedObjectPublishers = new ArrayList<>();

   public PerceptionManager(ReferenceFrame cameraFrame)
   {
      ros2 = new ROS2Helper(DomainFactory.PubSubImplementation.FAST_RTPS, "perception_manager");

      arUcoMarkerPosesSubscription = ros2.subscribe(ARUCO_MARKER_POSES);

      pullDoorManager = new DoorPerceptionManager(PULL_DOOR_MARKER_ID, "Pull", cameraFrame);
      pullDoorManager.getDoorFrame().setObjectTransformToMarker(transform -> transform.set(PULL_DOOR_FRAME_TRANSFORM_TO_MARKER));
      pullDoorManager.getDoorPanel().setObjectTransformToMarker(transform -> transform.set(PULL_DOOR_PANEL_TRANSFORM_TO_MARKER));
      markerUpdaters.put(PULL_DOOR_MARKER_ID, pullDoorManager::updateMarkerTransform);

      pushDoorManager = new DoorPerceptionManager(PUSH_DOOR_MARKER_ID, "Push", cameraFrame);
      pushDoorManager.getDoorFrame().setObjectTransformToMarker(transform -> transform.set(PUSH_DOOR_FRAME_TRANSFORM_TO_MARKER));
      pushDoorManager.getDoorPanel().setObjectTransformToMarker(transform -> transform.set(PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER));
      markerUpdaters.put(PUSH_DOOR_MARKER_ID, pushDoorManager::updateMarkerTransform);

      box.setObjectTransformToMarker(transform -> transform.set(BOX_TRANSFORM_TO_MARKER));
      markerUpdaters.put(BOX_MARKER_ID, box::updateMarkerTransform);

      detectedObjectPublishers.add(new DetectedObjectPublisher(ros2,
                                                               DETECTED_PULL_DOOR_FRAME,
                                                               PULL_DOOR_MARKER_ID,
                                                               pullDoorManager.getDoorFrame().getObjectFrame()));
      detectedObjectPublishers.add(new DetectedObjectPublisher(ros2,
                                                               DETECTED_PULL_DOOR_PANEL,
                                                               PULL_DOOR_MARKER_ID,
                                                               pullDoorManager.getDoorPanel().getObjectFrame()));
      detectedObjectPublishers.add(new DetectedObjectPublisher(ros2,
                                                               DETECTED_PUSH_DOOR_FRAME,
                                                               PUSH_DOOR_MARKER_ID,
                                                               pushDoorManager.getDoorFrame().getObjectFrame()));
      detectedObjectPublishers.add(new DetectedObjectPublisher(ros2,
                                                               DETECTED_PUSH_DOOR_PANEL,
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

   public DoorPerceptionManager getPushDoorManager()
   {
      return pushDoorManager;
   }

   public DoorPerceptionManager getPullDoorManager()
   {
      return pullDoorManager;
   }

   public ArUcoMarkerObject getBox()
   {
      return box;
   }
}
