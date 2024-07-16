package us.ihmc.perception.sceneGraph.modification;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.sceneGraph.SceneNode;

/**
 * An actionable scene node replacement back into the tree,
 * used primary for network synchronized subscriptions where the tree
 * is disassembled entirely and put back together on every newly
 * recieved message.
 *
 * In this case, you don't want to freeze the parent and you don't
 * need to recursively check children frames, as the tree is rebuilt
 * in depth first order.
 */
public class SceneGraphNodeReplacement implements SceneGraphTreeModification
{
   private final SceneNode nodeToAdd;
   private final SceneNode parent;
   private transient final FramePose3D nodeToAddPose = new FramePose3D();

   public SceneGraphNodeReplacement(SceneNode nodeToAdd, SceneNode parent, RigidBodyTransformReadOnly nodeTransformToWorld)
   {
      this.nodeToAdd = nodeToAdd;
      this.parent = parent;
      nodeToAddPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), nodeTransformToWorld);
   }

   @Override
   public void performOperation()
   {
      parent.getChildren().add(nodeToAdd);
      nodeToAdd.ensureParentFrameIsConsistent(parent.getNodeFrame());
      nodeToAddPose.changeFrame(parent.getNodeFrame());
      nodeToAdd.setNodeToParentFrameTransformAndUpdate(nodeToAddPose);
   }

   protected SceneNode getNodeToAdd()
   {
      return nodeToAdd;
   }

   protected SceneNode getParent()
   {
      return parent;
   }
}
