package us.ihmc.perception.sceneGraph;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.tools.Timer;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a node on the scene graph which is
 * built as a CRDT, so part of this node are present
 * only to address that.
 *
 * We give each node a name and a reference frame.
 */
public class SceneNode
{
   /** The node's unique ID. */
   private final long id;
   private final String name;
   private final ModifiableReferenceFrame nodeFrame;
   private final List<SceneNode> children = new ArrayList<>();
   /**
    * Certain changes to this node will cause a freeze of that data
    * from being modified from incoming messages.
    * Scene nodes are usually being synced at 30 Hz or faster, so 1 second
    * should allow plenty of time for changes to propagate.
    */
   public static final double FREEZE_DURATION_ON_MODIFICATION = 1.0;
   /**
    * This timer is used in the case that an operator can "mark modified" this node's
    * data so it won't accept updates from other sources for a short period of time.
    * This is to allow the changes to propagate elsewhere.
    */
   private final Timer modifiedTimer = new Timer();

   public SceneNode(long id, String name)
   {
      this.id = id;
      this.name = name;
      this.nodeFrame = new ModifiableReferenceFrame(name, ReferenceFrame.getWorldFrame());
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

   public ModifiableReferenceFrame getModifiableNodeFrame()
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
         nodeFrame.changeParentFrame(desiredParentFrame);
   }

   public void changeParentFrameWithoutMoving(ReferenceFrame newParentFrame)
   {
      nodeFrame.changeParentFrameWithoutMoving(newParentFrame);
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

   public void freezeFromModification()
   {
      modifiedTimer.reset();
   }

   public boolean isFrozenFromModification()
   {
      return modifiedTimer.isRunning(FREEZE_DURATION_ON_MODIFICATION);
   }
}
