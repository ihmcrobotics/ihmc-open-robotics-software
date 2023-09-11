package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.rdx.ui.RDX3DPanel;

public class RDXDetectableSceneNode extends RDXSceneNode
{
   private final DetectableSceneNode detectableSceneNode;

   public RDXDetectableSceneNode(DetectableSceneNode detectableSceneNode, RDX3DPanel panel3D)
   {
      super(detectableSceneNode, panel3D);

      this.detectableSceneNode = detectableSceneNode;
   }
}
