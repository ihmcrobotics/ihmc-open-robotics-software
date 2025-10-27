package us.ihmc.behaviors.behaviorTree.scene;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public class BehaviorTreeSceneObject
{
   protected final RigidBodyTransform transform = new RigidBodyTransform();

   public String getName()
   {
      return "";
   }

   public void clearOffset()
   {

   }

   public void freeze()
   {

   }

   public ReferenceFrame getReferenceFrame()
   {
      return null;
   }

   public RigidBodyTransformReadOnly getTransformToWorld()
   {
      return transform;
   }
}
