package us.ihmc.perception.sceneGraph.ros2;

import perception_msgs.msg.dds.ArUcoMarkerNodeMessage;
import perception_msgs.msg.dds.CenterposeNodeMessage;
import perception_msgs.msg.dds.DetectableSceneNodeMessage;
import perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage;
import perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage;
import perception_msgs.msg.dds.SceneNodeMessage;
import perception_msgs.msg.dds.StaticRelativeSceneNodeMessage;
import perception_msgs.msg.dds.YOLOv8NodeMessage;

import java.util.ArrayList;
import java.util.List;

/**
 * An intermediate tree representation of the ROS 2 message to assist in synchronizing
 * with the actual tree.
 */
public class ROS2SceneGraphSubscriptionNode
{
   private byte type;
   private SceneNodeMessage sceneNodeMessage;
   private DetectableSceneNodeMessage detectableSceneNodeMessage;
   private PredefinedRigidBodySceneNodeMessage predefinedRigidBodySceneNodeMessage;
   private ArUcoMarkerNodeMessage arUcoMarkerNodeMessage;
   private CenterposeNodeMessage centerposeNodeMessage;
   private YOLOv8NodeMessage yoloNodeMessage;
   private StaticRelativeSceneNodeMessage staticRelativeSceneNodeMessage;
   private PrimitiveRigidBodySceneNodeMessage primitiveRigidBodySceneNodeMessage;
   private final List<ROS2SceneGraphSubscriptionNode> children = new ArrayList<>();

   public void clear()
   {
      type = -1;
      sceneNodeMessage = null;
      detectableSceneNodeMessage = null;
      predefinedRigidBodySceneNodeMessage = null;
      arUcoMarkerNodeMessage = null;
      centerposeNodeMessage = null;
      yoloNodeMessage = null;
      staticRelativeSceneNodeMessage = null;
      children.clear();
   }

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

   public CenterposeNodeMessage getCenterposeNodeMessage()
   {
      return centerposeNodeMessage;
   }

   public void setCenterposeNodeMessage(CenterposeNodeMessage centerposeNodeMessage)
   {
      this.centerposeNodeMessage = centerposeNodeMessage;
   }

   public YOLOv8NodeMessage getYOLONodeMessage()
   {
      return yoloNodeMessage;
   }

   public void setYOLONodeMessage(YOLOv8NodeMessage yoloNodeMessage)
   {
      this.yoloNodeMessage = yoloNodeMessage;
   }

   public StaticRelativeSceneNodeMessage getStaticRelativeSceneNodeMessage()
   {
      return staticRelativeSceneNodeMessage;
   }

   public void setStaticRelativeSceneNodeMessage(StaticRelativeSceneNodeMessage staticRelativeSceneNodeMessage)
   {
      this.staticRelativeSceneNodeMessage = staticRelativeSceneNodeMessage;
   }

   public PrimitiveRigidBodySceneNodeMessage getPrimitiveRigidBodySceneNodeMessage()
   {
      return primitiveRigidBodySceneNodeMessage;
   }

   public void setPrimitiveRigidBodySceneNodeMessage(PrimitiveRigidBodySceneNodeMessage primitiveRigidBodySceneNodeMessage)
   {
      this.primitiveRigidBodySceneNodeMessage = primitiveRigidBodySceneNodeMessage;
   }

   public List<ROS2SceneGraphSubscriptionNode> getChildren()
   {
      return children;
   }
}
