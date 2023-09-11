package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.perception.sceneGraph.rigidBodies.PredefinedRigidBodySceneNode;
import us.ihmc.rdx.ui.RDX3DPanel;

public class RDXPredefinedRigidBodySceneNode extends RDXSceneNode
{
   private final PredefinedRigidBodySceneNode predefinedRigidBodySceneNode;

   public RDXPredefinedRigidBodySceneNode(PredefinedRigidBodySceneNode predefinedRigidBodySceneNode, RDX3DPanel panel3D)
   {
      super(predefinedRigidBodySceneNode, panel3D);

      this.predefinedRigidBodySceneNode = predefinedRigidBodySceneNode;
   }
}
