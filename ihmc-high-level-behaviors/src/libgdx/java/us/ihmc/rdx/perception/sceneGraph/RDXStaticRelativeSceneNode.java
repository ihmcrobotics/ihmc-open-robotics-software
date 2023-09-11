package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;
import us.ihmc.rdx.ui.RDX3DPanel;

public class RDXStaticRelativeSceneNode extends RDXPredefinedRigidBodySceneNode
{
   private final StaticRelativeSceneNode staticRelativeSceneNode;

   public RDXStaticRelativeSceneNode(StaticRelativeSceneNode staticRelativeSceneNode, RDX3DPanel panel3D)
   {
      super(staticRelativeSceneNode, panel3D);

      this.staticRelativeSceneNode = staticRelativeSceneNode;
   }
}
