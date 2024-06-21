package us.ihmc.perception.sceneGraph.rigidBody;

import gnu.trove.map.TLongObjectMap;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeMove;

public class RigidBodySceneNode extends SceneNode
{
   private final TLongObjectMap<SceneNode> sceneGraphIDToNodeMap;
   private final long initialParentNodeID;
   private final RigidBodyTransform initialTransformToParent = new RigidBodyTransform();
   private transient final FramePose3D initialPose = new FramePose3D();

   public RigidBodySceneNode(long id,
                             String name,
                             TLongObjectMap<SceneNode> sceneGraphIDToNodeMap,
                             long initialParentNodeID,
                             RigidBodyTransformReadOnly initialTransformToParent,
                             CRDTInfo crdtInfo)
   {
      super(id, name, crdtInfo);
      this.sceneGraphIDToNodeMap = sceneGraphIDToNodeMap;
      this.initialParentNodeID = initialParentNodeID;
      this.initialTransformToParent.set(initialTransformToParent);

      getNodeToParentFrameTransform().set(initialTransformToParent);
      getNodeFrame().update();
   }

   public void setTrackInitialParent(boolean trackInitialParent, SceneGraphModificationQueue modificationQueue)
   {
      boolean previousTrackingInitialParent = getTrackingInitialParent();
      if (previousTrackingInitialParent != trackInitialParent)
      {
         SceneNode rootNode = sceneGraphIDToNodeMap.get(SceneGraph.ROOT_NODE_ID);
         SceneNode initialParentNode = sceneGraphIDToNodeMap.get(initialParentNodeID);

         SceneNode previousParent = trackInitialParent ? rootNode : initialParentNode;
         SceneNode newParent = trackInitialParent ? initialParentNode : rootNode;
         modificationQueue.accept(new SceneGraphNodeMove(this, previousParent, newParent));
      }
   }

   public boolean getTrackingInitialParent()
   {
      if (sceneGraphIDToNodeMap.containsKey(initialParentNodeID))
      {
         return getNodeFrame().getParent() == sceneGraphIDToNodeMap.get(initialParentNodeID).getNodeFrame();
      }
      return false;
   }

   /**
    * This sets the transform to the parent node back to the original one.
    * This is robust to whether or not this node is currently tracking the detected pose.
    */
   public void clearOffset()
   {
      SceneNode initialParentNode = sceneGraphIDToNodeMap.get(initialParentNodeID);
      if (getNodeFrame().getParent() != initialParentNode.getNodeFrame())
      {
         initialPose.setIncludingFrame(initialParentNode.getNodeFrame(), initialTransformToParent);
         initialPose.changeFrame(getNodeFrame().getParent());
         initialPose.get(getNodeToParentFrameTransform());
      }
      else
      {
         getNodeToParentFrameTransform().set(initialTransformToParent);
      }
      getNodeFrame().update();
   }

   public TLongObjectMap<SceneNode> getSceneGraphIDToNodeMap()
   {
      return sceneGraphIDToNodeMap;
   }

   public long getInitialParentNodeID()
   {
      return initialParentNodeID;
   }

   public void setInitialTransformToParent(RigidBodyTransform initialTransformToParent)
   {
      this.initialTransformToParent.set(initialTransformToParent);
   }

   public RigidBodyTransform getInitialTransformToParent()
   {
      return initialTransformToParent;
   }
}
