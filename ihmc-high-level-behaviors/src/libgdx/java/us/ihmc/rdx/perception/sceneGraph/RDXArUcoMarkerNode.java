package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.rdx.ui.RDX3DPanel;

public class RDXArUcoMarkerNode extends RDXDetectableSceneNode
{
   private final ArUcoMarkerNode arUcoMarkerNode;

   public RDXArUcoMarkerNode(ArUcoMarkerNode arUcoMarkerNode, RDX3DPanel panel3D)
   {
      super(arUcoMarkerNode, panel3D);

      this.arUcoMarkerNode = arUcoMarkerNode;
   }
}
