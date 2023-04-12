package us.ihmc.perception;

import perception_msgs.msg.dds.ArUcoMarkerPoses;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.perception.objects.*;
import us.ihmc.perception.scene.ROS2DetectableSceneObject;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BiConsumer;

/**
 * On-robot process that currently takes pre-registered ArUco markers detected and re-publishes them
 * as our predefined objects, like doors, boxes, etc.
 *
 * TODO: Should just be DetectedObjectsPerceptionManager or something.
 *   It should support detected objects abstractly, using our ArUco & other implementations.
 */
public class ArUcoObjectsPerceptionManager
{

   public static final double REAL_MARKER_WIDTH = 0.1680;

   /** This refers to the edges of the black parts with no margin. The margins included will be wider than this. */
   public static final double DOOR_MARKER_WIDTH = 0.2032;

   static
   {
      DoorSceneObjects.PUSH_DOOR_FRAME_TRANSFORM_TO_MARKER.getTranslation().add(0.0,
                                                                                -DoorModelParameters.DOOR_FRAME_HINGE_OFFSET,
                                                                                -DoorModelParameters.DOOR_PANEL_GROUND_GAP_HEIGHT);
   }

   public static final long BOX_MARKER_ID = 2;
   public static final double BOX_MARKER_WIDTH = 0.210101;
   public static final RigidBodyTransform BOX_TRANSFORM_TO_MARKER = new RigidBodyTransform(
         new YawPitchRoll(Math.toRadians(180.0), Math.toRadians(0.0), Math.toRadians(-90.0)),
         new Point3D(0.07, 0.15, 0.17)
   );

   private final IHMCROS2Input<ArUcoMarkerPoses> arUcoMarkerPosesSubscription;
   private final HashMap<Integer, BiConsumer<FrameTuple3DReadOnly, FrameQuaternionReadOnly>> markerUpdaters = new HashMap<>();
   private final FramePoint3D detectedMarkerTranslation = new FramePoint3D();
   private final FrameQuaternion detectedMarkerOrientation = new FrameQuaternion();
   private final ArrayList<ArUcoMarkerObject> markers = new ArrayList<>();
   private final DoorPerceptionManager pullDoorManager;
   private final DoorPerceptionManager pushDoorManager;
   private final ArUcoMarkerObject box = new ArUcoMarkerObject((int) BOX_MARKER_ID, null, "Box");
   private final ROS2PublishSubscribeAPI ros2;

   private ArrayList<String> objectNames;
   private final ArrayList<ROS2DetectableSceneObject> detectedObjectPublishers = new ArrayList<>();

   public ArUcoObjectsPerceptionManager(ROS2PublishSubscribeAPI ros2, ArUcoMarkerObjectsInfo objectsInfo, ReferenceFrame cameraFrame)
   {
      this.ros2 = ros2;
      arUcoMarkerPosesSubscription = ros2.subscribe(ROS2Tools.ARUCO_MARKER_POSES);

      ArrayList<Integer> IDs = objectsInfo.getIds();
      objectNames = objectsInfo.getObjectNames();
      for (int i = 0; i < IDs.size(); i++)
      {
         markers.add(new ArUcoMarkerObject(IDs.get(i), objectsInfo, objectNames.get(IDs.get(i)) + "Marker"));
         markerUpdaters.put(IDs.get(i), markers.get(i)::updateMarkerTransform);
         detectedObjectPublishers.add(new ROS2DetectableSceneObject(ros2,
                                                                  DETECTED_OBJECT,
                                                                  objectNames.get(IDs.get(i)),
                                                                  markers.get(i).getObjectFrame()));
                                                                
      }
      

      pullDoorManager = new DoorPerceptionManager(DoorSceneObjects.PULL_DOOR_MARKER_ID, "Pull", cameraFrame);
      pullDoorManager.getDoorFrame().setObjectTransformToMarker(transform -> transform.set(DoorSceneObjects.PULL_DOOR_FRAME_TRANSFORM_TO_MARKER));
      pullDoorManager.getDoorPanel().setObjectTransformToMarker(transform -> transform.set(DoorSceneObjects.PULL_DOOR_PANEL_TRANSFORM_TO_MARKER));
      markerUpdaters.put((int) DoorSceneObjects.PULL_DOOR_MARKER_ID, pullDoorManager::updateMarkerTransform);

      pushDoorManager = new DoorPerceptionManager(DoorSceneObjects.PUSH_DOOR_MARKER_ID, "Push", cameraFrame);
      pushDoorManager.getDoorFrame().setObjectTransformToMarker(transform -> transform.set(DoorSceneObjects.PUSH_DOOR_FRAME_TRANSFORM_TO_MARKER));
      pushDoorManager.getDoorPanel().setObjectTransformToMarker(transform -> transform.set(DoorSceneObjects.PUSH_DOOR_PANEL_TRANSFORM_TO_MARKER));
      markerUpdaters.put((int) DoorSceneObjects.PUSH_DOOR_MARKER_ID, pushDoorManager::updateMarkerTransform);

      box.setObjectTransformToMarker(transform -> transform.set(BOX_TRANSFORM_TO_MARKER));
      markerUpdaters.put((int) BOX_MARKER_ID, box::updateMarkerTransform);

   }

   public void update()
   {
      if (arUcoMarkerPosesSubscription.getMessageNotification().poll())
      {
         for (ROS2DetectableSceneObject detectedObjectPublisher : detectedObjectPublishers)
         {
            detectedObjectPublisher.reset();
         }

         ArUcoMarkerPoses arUcoMarkerPosesMessage = arUcoMarkerPosesSubscription.getMessageNotification().read();
         for (int i = 0; i < arUcoMarkerPosesMessage.getMarkerId().size(); i++)
         {
            int markerId = (int) arUcoMarkerPosesMessage.getMarkerId().get(i);
            var markerUpdater = markerUpdaters.get(markerId);
//            var markerUpdater = markerUpdaters.get(arUcoMarkerPosesMessage.getMarkerId().get(i));
            if (markerUpdater != null)
            {
               //detectedMarkerTranslation.set(arUcoMarkerPosesMessage.getPosition().get(i));
               //detectedMarkerOrientation.set(arUcoMarkerPosesMessage.getOrientation().get(i));
               //markerUpdater.accept(detectedMarkerTranslation, detectedMarkerOrientation);

               
               markerUpdater.accept(new FramePoint3D(ReferenceFrame.getWorldFrame(), arUcoMarkerPosesMessage.getPosition().get(i)),
                                    new FrameQuaternion(ReferenceFrame.getWorldFrame(), arUcoMarkerPosesMessage.getOrientation().get(i)));
            }

            for (ROS2DetectableSceneObject detectedObjectPublisher : detectedObjectPublishers)
            {
               //detectedObjectPublisher.objectDetected(objectNames.get(markerId));
               //detectedObjectPublisher.publish();
//               detectedObjectPublisher.markDetected(arUcoMarkerPosesMessage.getMarkerId().get(i));
            }
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
