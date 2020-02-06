package us.ihmc.exampleSimulations.footPathRunners;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.simulationconstructionset.Joint;

public class JointReferenceFrame extends ReferenceFrame
{
   private final Joint joint;

   public JointReferenceFrame(Joint joint)
   {
      super(joint.getName() + "ReferenceFrame", ReferenceFrame.getWorldFrame(), false, false);
      this.joint = joint;
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      joint.getTransformToWorld(transformToParent);
   }

}
