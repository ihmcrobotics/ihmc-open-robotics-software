package us.ihmc.perception.sceneGraph;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.RequestConfirmFreezable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a node on the scene graph which is
 * built as a CRDT, so part of this node are present
 * only to address that.
 *
 * We give each node a name and a reference frame.
 */
public class SceneNode extends RequestConfirmFreezable
{
   /** The node's unique ID. */
   private final long id;
   private final String name;
   private final MutableReferenceFrame nodeFrame;
   private final List<SceneNode> children = new ArrayList<>();

   public SceneNode(long id, String name, CRDTInfo crdtInfo)
   {
      super(crdtInfo);

      this.id = id;
      this.name = name;
      this.nodeFrame = new MutableReferenceFrame(name, ReferenceFrame.getWorldFrame());
   }

   public void update(SceneGraph sceneGraph, SceneGraphModificationQueue modificationQueue)
   {

   }

   public long getID()
   {
      return id;
   }

   public String getName()
   {
      return name;
   }

   public ReferenceFrame getNodeFrame()
   {
      return nodeFrame.getReferenceFrame();
   }

   public MutableReferenceFrame getModifiableNodeFrame()
   {
      return nodeFrame;
   }

   /**
    * Used to get and set the transform to the parent frame.
    * If you modify this transform, you must then call {@link ReferenceFrame#update()} on {@link #getNodeFrame()}.
    *
    * If you don't need to modify the frame, consider using {@link #getNodeToParentFrameTransformReadOnly()} for access safety.
    *
    * If you want to modify the transform directly, consider using {@link #setNodeToParentFrameTransformAndUpdate(RigidBodyTransformReadOnly)}, which avoids
    * needing to call {@link #getNodeFrame()}.
    * @return the transform to the parent frame
    */
   public RigidBodyTransform getNodeToParentFrameTransform()
   {
      return nodeFrame.getTransformToParent();
   }

   /**
    * Used to get and set a read-only transform to the parent frame.
    * @return read-only access to the transform to the parent frame
    */
   public RigidBodyTransformReadOnly getNodeToParentFrameTransformReadOnly()
   {
      return nodeFrame.getTransformToParent();
   }

   public void setNodeToParentFrameTransformAndUpdate(RigidBodyTransformReadOnly transformToParent)
   {
      nodeFrame.getTransformToParent().set(transformToParent);
      nodeFrame.getReferenceFrame().update();
   }

   /**
    * This makes sure this node's ReferenceFrame's parent is the same instance
    * as this node's parent node's ReferenceFrame.
    */
   public void ensureParentFrameIsConsistent(ReferenceFrame desiredParentFrame)
   {
      if (desiredParentFrame != nodeFrame.getReferenceFrame().getParent())
         nodeFrame.setParentFrame(desiredParentFrame);
   }

   /** See {@link MutableReferenceFrame#changeFrame} */
   public void changeFrame(ReferenceFrame newParentFrame)
   {
      nodeFrame.changeFrame(newParentFrame);
   }

   /**
    * @return The scene node's children.
    *
    * Warning! Only modify this collection via queued modifications using
    * {@link SceneGraph#modifyTree}. Otherwise, inconsistency can occur
    * which may cause bad behavior or crashes.
    */
   public List<SceneNode> getChildren()
   {
      return children;
   }

   /**
    * This method is called when the node is removed from the scene graph.
    * Note that this method does not destroy this node's children as they may want to be adopted by another node.
    */
   public void destroy(SceneGraph sceneGraph)
   {

   }
}
