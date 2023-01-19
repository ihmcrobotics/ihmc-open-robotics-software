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
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Topic;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BiConsumer;

public class ObjectDetector
{
   private static final ROS2Topic<?> BASE_TOPIC = ROS2Tools.IHMC_ROOT.withModule("perception_manager");
   public static final ROS2Topic<ArUcoMarkerPoses> ARUCO_MARKER_POSES = BASE_TOPIC.withType(ArUcoMarkerPoses.class).withSuffix("aruco_marker_poses");

   private final IHMCROS2Input<ArUcoMarkerPoses> arUcoMarkerPosesSubscription;
   private final HashMap<Long, BiConsumer<Tuple3DReadOnly, QuaternionReadOnly>> markerUpdaters = new HashMap<>();
   private final ROS2Helper ros2;

   public ObjectDetector()
   {
      ros2 = new ROS2Helper(DomainFactory.PubSubImplementation.FAST_RTPS, "perception_manager");
      arUcoMarkerPosesSubscription = ros2.subscribe(ARUCO_MARKER_POSES);


   }

   public void update()
   {
      if (arUcoMarkerPosesSubscription.getMessageNotification().poll())
      {
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
