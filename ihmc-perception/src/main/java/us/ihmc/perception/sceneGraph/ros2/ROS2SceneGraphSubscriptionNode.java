package us.ihmc.perception.sceneGraph.ros2;

import perception_msgs.msg.dds.*;

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
   private YOLOv8ICPNodeMessage yoloICPNodeMessage;
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
      yoloICPNodeMessage = null;
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

   public YOLOv8ICPNodeMessage getYOLOv8ICPNodeMessage()
   {
      return yoloICPNodeMessage;
   }

   public void setYOLOv8ICPNodeMessage(YOLOv8ICPNodeMessage yoloICPNodeMessage)
   {
      this.yoloICPNodeMessage = yoloICPNodeMessage;
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
