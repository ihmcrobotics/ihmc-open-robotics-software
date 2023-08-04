package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoDetectableNode;
import us.ihmc.rdx.imgui.*;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

import java.util.Set;

/**
 * A "ghost" colored model, with reference frame graphic in the virtual scene
 * to be used for debugging and know where the frame is.
 *
 * TODO: Add pose "override", via right click context menu, and gizmo.
 *   Possibly do this in a higher level class or class that extends this.
 */
public class RDXPredefinedRigidBodySceneNode
{
   private static final ColorDefinition GHOST_COLOR = ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.5);
   private final PredefinedRigidBodySceneNode sceneNode;
   private final RDXReferenceFrameGraphic referenceFrameGraphic;
   private boolean showing = false;
   private final RDXModelInstance modelInstance;
   private final FramePose3D nodePose = new FramePose3D();
   private final RigidBodyTransform visualModelToWorldTransform = new RigidBodyTransform();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiEnumPlot currentlyDetectedPlot;
   private final RDXSelectablePose3DGizmo offsetPoseGizmo;
   private final ImBooleanWrapper trackDetectedPoseWrapper;
   private ImGuiSliderDoubleWrapper alphaFilterValueSlider;

   public RDXPredefinedRigidBodySceneNode(PredefinedRigidBodySceneNode predefinedRigidBodySceneNode, RDX3DPanel panel3D)
   {
      this.sceneNode = predefinedRigidBodySceneNode;

      modelInstance = new RDXModelInstance(RDXModelLoader.load(sceneNode.getVisualModelFilePath()));
      modelInstance.setColor(GHOST_COLOR);

      referenceFrameGraphic = new RDXReferenceFrameGraphic(0.05, Color.BLUE);
      offsetPoseGizmo = new RDXSelectablePose3DGizmo(sceneNode.getNodeFrame(),
                                                     sceneNode.getNodeToParentFrameTransform());
      offsetPoseGizmo.createAndSetupDefault(panel3D);
      trackDetectedPoseWrapper = new ImBooleanWrapper(sceneNode::getTrackDetectedPose,
                                                      trackDetectedPose ->
                                                      {
                                                         sceneNode.setTrackDetectedPose(trackDetectedPose);
                                                         ensureGizmoFrameIsSceneNodeFrame();
                                                         sceneNode.markModifiedByOperator();
                                                      },
                                                      imBoolean -> ImGui.checkbox(labels.get("Track Detected Pose"), imBoolean));

      int bufferSize = 1000;
      int heightPixels = 20;
      currentlyDetectedPlot = new ImGuiEnumPlot(predefinedRigidBodySceneNode.getName(), bufferSize, heightPixels);

      if (sceneNode instanceof ArUcoDetectableNode arUcoDetectableNode)
      {
         alphaFilterValueSlider = new ImGuiSliderDoubleWrapper("Break frequency", "%.2f", 0.2, 5.0,
                                                               arUcoDetectableNode::getBreakFrequency,
                                                               arUcoDetectableNode::setBreakFrequency);
      }
   }

   public void update()
   {
      showing = sceneNode.getCurrentlyDetected() || !sceneNode.getTrackDetectedPose();

      referenceFrameGraphic.setToReferenceFrame(sceneNode.getNodeFrame());
      ensureGizmoFrameIsSceneNodeFrame();
      if (!showing)
         offsetPoseGizmo.getSelected().set(false);

      if (offsetPoseGizmo.getPoseGizmo().getGizmoModifiedByUser().poll())
      {
         sceneNode.markModifiedByOperator();
      }

      nodePose.setToZero(sceneNode.getNodeFrame());
      nodePose.set(sceneNode.getVisualModelToNodeFrameTransform());
      nodePose.changeFrame(ReferenceFrame.getWorldFrame());
      nodePose.get(visualModelToWorldTransform);
      modelInstance.setTransformToWorldFrame(visualModelToWorldTransform);
   }

   public void renderImGuiWidgets()
   {
      boolean currentlyDetected = sceneNode.getCurrentlyDetected();
      currentlyDetectedPlot.setWidgetTextColor(currentlyDetected ? ImGuiTools.GREEN : ImGuiTools.RED);
      currentlyDetectedPlot.render(currentlyDetected ? 1 : 0, currentlyDetected ? "CURRENTLY DETECTED" : "NOT DETECTED");

      trackDetectedPoseWrapper.renderImGuiWidget();
      ImGui.sameLine();
      ImGui.beginDisabled(!showing);
      ImGui.checkbox(labels.get("Show Offset Gizmo"), offsetPoseGizmo.getSelected());
      ImGui.endDisabled();
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear Offset")))
      {
         sceneNode.clearOffset();
         ensureGizmoFrameIsSceneNodeFrame();
         sceneNode.markModifiedByOperator();
      }
      if (sceneNode instanceof ArUcoDetectableNode)
      {
         alphaFilterValueSlider.render();
         sceneNode.markModifiedByOperator();
      }

      ImGui.separator();
   }

   private void ensureGizmoFrameIsSceneNodeFrame()
   {
      if (offsetPoseGizmo.getPoseGizmo().getGizmoFrame() != sceneNode.getNodeFrame())
         offsetPoseGizmo.getPoseGizmo().setGizmoFrame(sceneNode.getNodeFrame());
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (showing)
      {
         if (sceneLevels.contains(RDXSceneLevel.MODEL))
            modelInstance.getRenderables(renderables, pool);

         if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
            referenceFrameGraphic.getRenderables(renderables, pool);
      }
   }
}
