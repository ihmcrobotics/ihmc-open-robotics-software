package us.ihmc.rdx.perception.sceneGraph;

import imgui.ImGui;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;

public class RDXCenterposeNode extends RDXDetectableSceneNode
{
   private final CenterposeNode centerposeNode;

   public RDXCenterposeNode(CenterposeNode centerposeNode)
   {
      super(centerposeNode);
      this.centerposeNode = centerposeNode;
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text("Marker ID: %d".formatted(centerposeNode.getMarkerID()));
      ImGui.sameLine();
   }
}
