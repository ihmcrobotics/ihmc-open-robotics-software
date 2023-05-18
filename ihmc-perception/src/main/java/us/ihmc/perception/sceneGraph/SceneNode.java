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
   /** We allow the operator to override the pose of any node in the scene graph. */
   private boolean isPoseOverriddenByOperator = false;
//   /**
//    * We have to change the parent frame to world when the operator overrides
//    * the pose, so we need to keep track of this.
//    */
//   private ReferenceFrame parentFrameWhenNotOverridden;

   public SceneNode(String name)
   {
      this(name, ReferenceFrame.getWorldFrame());
   }

   public SceneNode(String name, ReferenceFrame parentFrame)
   {
      this.name = name;
      this.nodeFrame = new ModifiableReferenceFrame(name, parentFrame);
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

   public boolean getPoseOverriddenByOperator()
   {
      return isPoseOverriddenByOperator;
   }

   public void setPoseOverriddenByOperator(boolean poseOverriddenByOperator)
   {
      this.isPoseOverriddenByOperator = poseOverriddenByOperator;
   }

//   /**
//    * Overriding a node's pose means that it becomes specified in world frame.
//    *
//    * Warning! If any nodes are children of this, they will be abandoned in the
//    * frame tree. TODO: Think about this more. This currently is never the case.
//    * We probably need to create the tree in the scene graph and it can keep
//    * the ReferenceFrame tree up to date.
//    */
//   public void setPoseOverriddenByOperator(boolean poseOverriddenByOperator)
//   {
//      if (isPoseOverriddenByOperator != poseOverriddenByOperator)
//      {
//         isPoseOverriddenByOperator = poseOverriddenByOperator;
//         if (poseOverriddenByOperator)
//         {
//            parentFrameWhenNotOverridden = nodeFrame.getReferenceFrame().getParent();
//            nodeFrame.changeParentFrameWithoutMoving(ReferenceFrame.getWorldFrame());
//         }
//         else
//         {
//            nodeFrame.changeParentFrameWithoutMoving(parentFrameWhenNotOverridden);
//         }
//      }
//   }
}
