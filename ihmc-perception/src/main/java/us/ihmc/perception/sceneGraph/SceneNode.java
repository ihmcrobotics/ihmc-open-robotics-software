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
   private final String name;
   private final ModifiableReferenceFrame nodeFrame;
   /**
    * Sometimes, the operator will want to override the pose of a scene node.
    * This can be for test cases, to make up for detection errors, or
    * to place objects for which no detection mechnism exists yet.
    */
   private boolean poseOverriddenByOperator = false;

   public SceneNode(String name)
   {
      this(name, ReferenceFrame.getWorldFrame());
   }

   public SceneNode(String name, ReferenceFrame parentFrame)
   {
      this.name = name;
      this.nodeFrame = new ModifiableReferenceFrame(parentFrame);
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

   public void setPoseOverriddenByOperator(boolean poseOverriddenByOperator)
   {
      this.poseOverriddenByOperator = poseOverriddenByOperator;
   }

   public boolean getPoseOverriddenByOperator()
   {
      return poseOverriddenByOperator;
   }
}
