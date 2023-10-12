package us.ihmc.rdx.perception.sceneGraph;

import imgui.ImGui;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.rdx.imgui.ImGuiInputDoubleWrapper;

public class RDXArUcoMarkerNode extends RDXDetectableSceneNode
{
   private final ArUcoMarkerNode arUcoMarkerNode;
   private final ImGuiInputDoubleWrapper alphaFilterValueSlider;

   public RDXArUcoMarkerNode(ArUcoMarkerNode arUcoMarkerNode)
   {
      super(arUcoMarkerNode);

      this.arUcoMarkerNode = arUcoMarkerNode;

      alphaFilterValueSlider = new ImGuiInputDoubleWrapper("Break frequency:",
                                                           "%.2f",
                                                           0.2,
                                                           5.0,
                                                           arUcoMarkerNode::getBreakFrequency,
                                                           arUcoMarkerNode::setBreakFrequency,
                                                           arUcoMarkerNode::freezeFromModification);
      alphaFilterValueSlider.setWidgetWidth(100.0f);
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();
      ImGui.text("Marker ID: %d   Size: %.2f m".formatted(arUcoMarkerNode.getMarkerID(), arUcoMarkerNode.getMarkerSize()));
      ImGui.sameLine();
      alphaFilterValueSlider.render();
   }
}
