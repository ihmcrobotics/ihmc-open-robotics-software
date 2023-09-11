package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

import java.util.Set;

/**
 * A "ghost" colored model, with reference frame graphic in the virtual scene
 * to be used for debugging and know where the frame is.
 *
 * TODO: Add pose "override", via right click context menu, and gizmo.
 *   Possibly do this in a higher level class or class that extends this.
 */
public class RDXSceneNode
{
   private final SceneNode sceneNode;
   private final RDXReferenceFrameGraphic referenceFrameGraphic;
   private boolean showing = false;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiEnumPlot currentlyDetectedPlot;
   private ImGuiSliderDoubleWrapper alphaFilterValueSlider;
   private ImGuiInputDoubleWrapper distanceToDisableTrackingInput;

   public RDXSceneNode(SceneNode sceneNode, RDX3DPanel panel3D)
   {
      this.sceneNode = sceneNode;


      referenceFrameGraphic = new RDXReferenceFrameGraphic(0.05, Color.BLUE);

      currentlyDetectedPlot = new ImGuiEnumPlot(predefinedRigidBodySceneNode.getName());

      if (sceneNode instanceof ArUcoMarkerNode arUcoMarkerNode)
      {
         alphaFilterValueSlider = new ImGuiSliderDoubleWrapper("Break frequency", "%.2f", 0.2, 5.0,
                                                               arUcoMarkerNode::getBreakFrequency,
                                                               arUcoMarkerNode::setBreakFrequency,
                                                               sceneNode::markModifiedByOperator);
      }
      if (sceneNode instanceof StaticRelativeSceneNode staticRelativeNode)
      {
         distanceToDisableTrackingInput = new ImGuiInputDoubleWrapper("Distance to disable tracking", "%.2f", 0.1, 0.5,
                                                                      staticRelativeNode::getDistanceToDisableTracking,
                                                                      staticRelativeNode::setDistanceToDisableTracking,
                                                                      sceneNode::markModifiedByOperator);
      }
   }

   public void update()
   {
      showing = sceneNode.getCurrentlyDetected() || !sceneNode.getTrackingInitialParent();

      referenceFrameGraphic.setToReferenceFrame(sceneNode.getNodeFrame());

   }

   public void renderImGuiWidgets()
   {
      if (sceneNode instanceof DetectableSceneNode detectableSceneNode)
      {
         boolean currentlyDetected = detectableSceneNode.getCurrentlyDetected();
         currentlyDetectedPlot.setWidgetTextColor(currentlyDetected ? ImGuiTools.GREEN : ImGuiTools.RED);
         currentlyDetectedPlot.render(currentlyDetected ? 1 : 0, currentlyDetected ? "CURRENTLY DETECTED" : "NOT DETECTED");
      }

      if (sceneNode instanceof ArUcoMarkerNode)
      {
         alphaFilterValueSlider.render();
      }
      if (sceneNode instanceof StaticRelativeSceneNode staticRelativeNode)
      {
         ImGui.text("Current distance: %.2f".formatted(staticRelativeNode.getCurrentDistance()));
         distanceToDisableTrackingInput.render();
      }

      ImGui.separator();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (showing)
      {
         if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
            referenceFrameGraphic.getRenderables(renderables, pool);
      }
   }

   void setShowing(boolean showing)
   {
      this.showing = showing;
   }

   boolean isShowing()
   {
      return showing;
   }
}
