package us.ihmc.perception.sceneGraph.ros2;

import perception_msgs.msg.dds.*;

import java.util.ArrayList;
import java.util.List;

public class ROS2SceneGraphSubscriptionNode
{
   private byte type;
   private SceneNodeMessage sceneNodeMessage;
   private DetectableSceneNodeMessage detectableSceneNodeMessage;
   private PredefinedRigidBodySceneNodeMessage predefinedRigidBodySceneNodeMessage;
   private ArUcoMarkerNodeMessage arUcoMarkerNodeMessage;
   private StaticRelativeSceneNodeMessage staticRelativeSceneNodeMessage;

   private final List<ROS2SceneGraphSubscriptionNode> children = new ArrayList<>();

   public byte getType()
   {
      return type;
   }

   public void setType(byte type)
   {
      this.type = type;
   }

   public SceneNodeMessage getSceneNodeMessage()
   {
      return sceneNodeMessage;
   }

   public void setSceneNodeMessage(SceneNodeMessage sceneNodeMessage)
   {
      this.sceneNodeMessage = sceneNodeMessage;
   }

   public DetectableSceneNodeMessage getDetectableSceneNodeMessage()
   {
      return detectableSceneNodeMessage;
   }

   public void setDetectableSceneNodeMessage(DetectableSceneNodeMessage detectableSceneNodeMessage)
   {
      this.detectableSceneNodeMessage = detectableSceneNodeMessage;
   }

   public PredefinedRigidBodySceneNodeMessage getPredefinedRigidBodySceneNodeMessage()
   {
      return predefinedRigidBodySceneNodeMessage;
   }

   public void setPredefinedRigidBodySceneNodeMessage(PredefinedRigidBodySceneNodeMessage predefinedRigidBodySceneNodeMessage)
   {
      this.predefinedRigidBodySceneNodeMessage = predefinedRigidBodySceneNodeMessage;
   }

   public ArUcoMarkerNodeMessage getArUcoMarkerNodeMessage()
   {
      return arUcoMarkerNodeMessage;
   }

   public void setArUcoMarkerNodeMessage(ArUcoMarkerNodeMessage arUcoMarkerNodeMessage)
   {
      this.arUcoMarkerNodeMessage = arUcoMarkerNodeMessage;
   }

   public StaticRelativeSceneNodeMessage getStaticRelativeSceneNodeMessage()
   {
      return staticRelativeSceneNodeMessage;
   }

   public void setStaticRelativeSceneNodeMessage(StaticRelativeSceneNodeMessage staticRelativeSceneNodeMessage)
   {
      this.staticRelativeSceneNodeMessage = staticRelativeSceneNodeMessage;
   }

   public List<ROS2SceneGraphSubscriptionNode> getChildren()
   {
      return children;
   }
}
