package us.ihmc.perception.sceneGraph;

import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.RequestConfirmFreezable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
    * @return the transform to the parent frame
    */
   public RigidBodyTransform getNodeToParentFrameTransform()
   {
      return nodeFrame.getTransformToParent();
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
}
