package us.ihmc.perception;

import perception_msgs.msg.dds.ArUcoMarkerPoses;
import perception_msgs.msg.dds.DetectedObjectMessage;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.perception.objects.*;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BiConsumer;

public class ArUcoObjectsPerceptionManager
{
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("objects_perception_manager");
   public static final ROS2Topic<DetectedObjectMessage> DETECTED_OBJECT
         = BASE_TOPIC.withType(DetectedObjectMessage.class).withSuffix("detected_object");

   private final IHMCROS2Input<ArUcoMarkerPoses> arUcoMarkerPosesSubscription;
   private final HashMap<Integer, BiConsumer<Tuple3DReadOnly, QuaternionReadOnly>> markerUpdaters = new HashMap<>();
   private final ArrayList<ArUcoMarkerObject> markers = new ArrayList<>();
   private final ROS2Helper ros2;

   private ArrayList<String> objectNames;
   private final ArrayList<DetectedObjectPublisher> detectedObjectPublishers = new ArrayList<>();

   public ArUcoObjectsPerceptionManager(ArUcoMarkerObjectsInfo objectsInfo)
   {
      ros2 = new ROS2Helper(DomainFactory.PubSubImplementation.FAST_RTPS, "objects_perception_manager");

      arUcoMarkerPosesSubscription = ros2.subscribe(ROS2Tools.ARUCO_MARKER_POSES);

      ArrayList<Integer> IDs = objectsInfo.getIds();
      objectNames = objectsInfo.getObjectNames();
      for (int i = 0; i < IDs.size(); i++)
      {
         markers.add(new ArUcoMarkerObject(IDs.get(i), objectsInfo));
         markerUpdaters.put(IDs.get(i), markers.get(i)::updateMarkerTransform);
         detectedObjectPublishers.add(new DetectedObjectPublisher(ros2,
                                                                  DETECTED_OBJECT,
                                                                  objectNames.get(IDs.get(i)),
                                                                  markers.get(i).getObjectFrame()));
      }
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
            int markerId = (int) arUcoMarkerPosesMessage.getMarkerId().get(i);
            var markerUpdater = markerUpdaters.get(markerId);
            if (markerUpdater != null)
            {
               markerUpdater.accept(arUcoMarkerPosesMessage.getPosition().get(i),
                                    arUcoMarkerPosesMessage.getOrientation().get(i));
            }

            for (DetectedObjectPublisher detectedObjectPublisher : detectedObjectPublishers)
            {
               detectedObjectPublisher.objectDetected(objectNames.get(markerId));
               detectedObjectPublisher.publish();
            }
         }
      }
   }
}