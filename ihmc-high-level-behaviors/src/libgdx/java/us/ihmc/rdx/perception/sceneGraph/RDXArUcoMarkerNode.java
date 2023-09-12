package us.ihmc.rdx.perception.sceneGraph;

import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.rdx.imgui.ImGuiSliderDoubleWrapper;
import us.ihmc.rdx.ui.RDX3DPanel;

public class RDXArUcoMarkerNode extends RDXDetectableSceneNode
{
   private final ArUcoMarkerNode arUcoMarkerNode;
   private ImGuiSliderDoubleWrapper alphaFilterValueSlider;

   public RDXArUcoMarkerNode(ArUcoMarkerNode arUcoMarkerNode)
   {
      super(arUcoMarkerNode);
      this.arUcoMarkerNode = arUcoMarkerNode;
      alphaFilterValueSlider = new ImGuiSliderDoubleWrapper("Break frequency", "%.2f", 0.2, 5.0,
                                                            arUcoMarkerNode::getBreakFrequency,
                                                            arUcoMarkerNode::setBreakFrequency,
                                                            arUcoMarkerNode::markModifiedByOperator);
   }

   @Override
   public void renderImGuiWidgets()
   {
      super.renderImGuiWidgets();

      alphaFilterValueSlider.render();
   }
}
