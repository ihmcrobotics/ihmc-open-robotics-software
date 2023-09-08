package us.ihmc.perception.sceneGraph.ros2;

import perception_msgs.msg.dds.ArUcoMarkerNodeMessage;
import perception_msgs.msg.dds.DetectableSceneNodeMessage;
import perception_msgs.msg.dds.SceneNodeMessage;
import perception_msgs.msg.dds.StaticRelativeSceneNodeMessage;

import java.util.ArrayList;
import java.util.List;

public class ROS2SceneGraphSubscriptionTree
{
   private byte type;
   private SceneNodeMessage sceneNodeMessage;
   private DetectableSceneNodeMessage detectableSceneNodeMessage;
   private ArUcoMarkerNodeMessage arUcoMarkerNodeMessage;
   private StaticRelativeSceneNodeMessage staticRelativeSceneNodeMessageeMessage;

   private final List<ROS2SceneGraphSubscriptionTree> children = new ArrayList<>();

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

   public ArUcoMarkerNodeMessage getArUcoMarkerNodeMessage()
   {
      return arUcoMarkerNodeMessage;
   }

   public void setArUcoMarkerNodeMessage(ArUcoMarkerNodeMessage arUcoMarkerNodeMessage)
   {
      this.arUcoMarkerNodeMessage = arUcoMarkerNodeMessage;
   }

   public StaticRelativeSceneNodeMessage getStaticRelativeSceneNodeMessageeMessage()
   {
      return staticRelativeSceneNodeMessageeMessage;
   }

   public void setStaticRelativeSceneNodeMessageeMessage(StaticRelativeSceneNodeMessage staticRelativeSceneNodeMessageeMessage)
   {
      this.staticRelativeSceneNodeMessageeMessage = staticRelativeSceneNodeMessageeMessage;
   }

   public List<ROS2SceneGraphSubscriptionTree> getChildren()
   {
      return children;
   }
}
