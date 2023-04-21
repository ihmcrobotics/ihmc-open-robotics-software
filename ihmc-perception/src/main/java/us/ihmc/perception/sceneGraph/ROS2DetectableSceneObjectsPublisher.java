package us.ihmc.perception.sceneGraph;

import perception_msgs.msg.dds.DetectableSceneObjectMessage;
import perception_msgs.msg.dds.DetectableSceneObjectsMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2PublishSubscribeAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;

/**
 * Publishes the current state of the complete collection of detectable scene objects.
 * Publishing all scene objects in one messages can simplify synchronization and
 * reduce the complexity of logic in figuring out when objects are currently under
 * consideraion.
 */
public class ROS2DetectableSceneObjectsPublisher
{
   private final DetectableSceneObjectsMessage detectableSceneObjectsMessage = new DetectableSceneObjectsMessage();
   private final FramePose3D sceneObjectPose = new FramePose3D();
   private final RigidBodyTransform sceneObjectToWorldTransform = new RigidBodyTransform();

   public void publish(Iterable<DetectableSceneObject> detectableSceneObjects, ROS2PublishSubscribeAPI ros2PublishSubscribeAPI)
   {
      detectableSceneObjectsMessage.getDetectableSceneObjects().clear();
      for (DetectableSceneObject detectableSceneObject : detectableSceneObjects)
      {
         DetectableSceneObjectMessage detectableSceneObjectMessage = detectableSceneObjectsMessage.getDetectableSceneObjects().add();
         detectableSceneObjectMessage.setName(detectableSceneObject.getName());
         detectableSceneObjectMessage.setCurrentlyDetected(detectableSceneObject.getCurrentlyDetected());
         sceneObjectPose.setIncludingFrame(detectableSceneObject.getReferenceFrame(), detectableSceneObject.getTransformToParent());
         sceneObjectPose.changeFrame(ReferenceFrame.getWorldFrame());
         sceneObjectPose.get(sceneObjectToWorldTransform);
         MessageTools.toMessage(sceneObjectToWorldTransform, detectableSceneObjectMessage.getTransformToWorld());
      }
      ros2PublishSubscribeAPI.publish(SceneObjectAPI.DETECTABLE_SCENE_OBJECTS, detectableSceneObjectsMessage);
   }
}
