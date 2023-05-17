package us.ihmc.perception.sceneGraph;

import perception_msgs.msg.dds.DetectableSceneNodeMessage;
import perception_msgs.msg.dds.DetectableSceneNodesMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2IOTopicQualifier;
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
public class ROS2DetectableSceneNodesPublisher
{
   private final DetectableSceneNodesMessage detectableSceneObjectsMessage = new DetectableSceneNodesMessage();
   private final FramePose3D sceneObjectPose = new FramePose3D();
   private final RigidBodyTransform sceneObjectToWorldTransform = new RigidBodyTransform();

   /**
    * @param ioQualifier Depending whether on the robot or in some other process.
    *                    If in the on-robot perception process, STATUS, else COMMAND.
    */
   public void publish(Iterable<DetectableSceneNode> detectableSceneObjects,
                       ROS2PublishSubscribeAPI ros2PublishSubscribeAPI,
                       ROS2IOTopicQualifier ioQualifier)
   {
      detectableSceneObjectsMessage.getDetectableSceneNodes().clear();
      for (DetectableSceneNode detectableSceneObject : detectableSceneObjects)
      {
         DetectableSceneNodeMessage detectableSceneNodeMessage = detectableSceneObjectsMessage.getDetectableSceneNodes().add();
         detectableSceneNodeMessage.setName(detectableSceneObject.getName());
         detectableSceneNodeMessage.setCurrentlyDetected(detectableSceneObject.getCurrentlyDetected());
         detectableSceneNodeMessage.setIsPoseOverriddenByOperator(detectableSceneObject.getPoseOverriddenByOperator());

         // If the pose is overridden by the operator, the operator side packs the pose
         // else, the on robot process packs the pose
         boolean isOperatorSide = ioQualifier == ROS2IOTopicQualifier.COMMAND;
         boolean isOnRobot = ioQualifier == ROS2IOTopicQualifier.STATUS;
         boolean shouldPackPose = detectableSceneObject.getPoseOverriddenByOperator() && isOperatorSide;
         shouldPackPose |= !detectableSceneObject.getPoseOverriddenByOperator() && isOnRobot;
         if (shouldPackPose)
         {
            sceneObjectPose.setToZero(detectableSceneObject.getNodeFrame());
            sceneObjectPose.changeFrame(ReferenceFrame.getWorldFrame());
            sceneObjectPose.get(sceneObjectToWorldTransform);
            MessageTools.toMessage(sceneObjectToWorldTransform, detectableSceneNodeMessage.getTransformToWorld());
         }
      }
      ros2PublishSubscribeAPI.publish(SceneGraphAPI.DETECTABLE_SCENE_NODES.getTopic(ioQualifier), detectableSceneObjectsMessage);
   }
}
