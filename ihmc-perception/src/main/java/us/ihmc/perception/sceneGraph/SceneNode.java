package us.ihmc.perception.sceneGraph;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;

/**
 * Represents a node on the Scene Knowledge Graph (to be defined).
 *
 * We give each node a name and a reference frame.
 */
public abstract class SceneNode
{
   /** A linearly increasing ID */
   private final long id;
   private final String name;
   private final ModifiableReferenceFrame nodeFrame;

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

   /**
    * Used to get and set the transform to the parent frame.
    * If you modify this transform, you must then call {@link ReferenceFrame#update()} on {@link #getNodeFrame()}.
    * @return the transform to the parent frame
    */
   public RigidBodyTransform getNodeToParentFrameTransform()
   {
      return nodeFrame.getTransformToParent();
   }

   protected void changeParentFrame(ReferenceFrame newParentFrame)
   {
      nodeFrame.changeParentFrame(newParentFrame);
   }

   protected void changeParentFrameWithoutMoving(ReferenceFrame newParentFrame)
   {
      nodeFrame.changeParentFrameWithoutMoving(newParentFrame);
   }
}
