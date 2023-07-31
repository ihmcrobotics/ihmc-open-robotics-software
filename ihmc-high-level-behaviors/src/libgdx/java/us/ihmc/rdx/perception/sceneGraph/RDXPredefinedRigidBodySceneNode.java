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
import us.ihmc.perception.sceneGraph.rigidBodies.StaticRelativeSceneNode;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiEnumPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
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
   /** For resetting the pose after overriding. */
   private final RigidBodyTransform originalTransformToParent = new RigidBodyTransform();
   private final RigidBodyTransform visualModelToWorldTransform = new RigidBodyTransform();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiEnumPlot currentlyDetectedPlot;
   private final RDXSelectablePose3DGizmo overridePoseGizmo;
   private final ImBooleanWrapper overridePoseWrapper;

   public RDXPredefinedRigidBodySceneNode(PredefinedRigidBodySceneNode predefinedRigidBodySceneNode, RDX3DPanel panel3D)
   {
      this.sceneNode = predefinedRigidBodySceneNode;

      originalTransformToParent.set(sceneNode.getNodeToParentFrameTransform());

      modelInstance = new RDXModelInstance(RDXModelLoader.load(sceneNode.getVisualModelFilePath()));
      modelInstance.setColor(GHOST_COLOR);

      referenceFrameGraphic = new RDXReferenceFrameGraphic(0.05, Color.BLUE);
      overridePoseGizmo = new RDXSelectablePose3DGizmo(sceneNode.getNodeFrame(),
                                                       sceneNode.getNodeToParentFrameTransform());
      overridePoseGizmo.createAndSetupDefault(panel3D);
      overridePoseWrapper = new ImBooleanWrapper(sceneNode::getPoseOverriddenByOperator,
                                                 poseOverriddenByOperator ->
                                                 {
                                                    sceneNode.setPoseOverriddenByOperator(poseOverriddenByOperator);
                                                    sceneNode.markModifiedByOperator();

                                                    if (sceneNode.getPoseOverriddenByOperator()) // Convenience
                                                    {
                                                       overridePoseGizmo.getSelected().set(true);
                                                    }
                                                    else if (sceneNode instanceof StaticRelativeSceneNode staticRelativeNode)
                                                    {
                                                       // TODO
                                                    }
                                                    else
                                                    {
                                                       sceneNode.getNodeToParentFrameTransform().set(originalTransformToParent);
                                                       sceneNode.getNodeFrame().update();
                                                       overridePoseGizmo.getPoseGizmo().update();
                                                       sceneNode.markModifiedByOperator();
                                                    }
                                                 },
                                                 imBoolean -> ImGui.checkbox(labels.get("Override pose"), imBoolean));

      int bufferSize = 1000;
      int heightPixels = 20;
      currentlyDetectedPlot = new ImGuiEnumPlot(predefinedRigidBodySceneNode.getName(), bufferSize, heightPixels);
   }

   public void update()
   {
      showing = sceneNode.getCurrentlyDetected() || sceneNode.getPoseOverriddenByOperator();

      referenceFrameGraphic.setToReferenceFrame(sceneNode.getNodeFrame());
      ensureGizmoFrameIsSceneNodeFrame();
      if (!showing || !sceneNode.getPoseOverriddenByOperator())
         overridePoseGizmo.getSelected().set(false);

      if (overridePoseGizmo.getPoseGizmo().getGizmoModifiedByUser().poll())
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

      overridePoseWrapper.renderImGuiWidget();
      ImGui.sameLine();
      ImGui.beginDisabled(!showing || !sceneNode.getPoseOverriddenByOperator());
      ImGui.checkbox(labels.get("Use gizmo"), overridePoseGizmo.getSelected());
      ImGui.endDisabled();
      if (sceneNode instanceof StaticRelativeSceneNode staticRelativeNode)
      {
         ImGui.sameLine();
         ImGui.beginDisabled(!staticRelativeNode.getPoseIsStatic());
         if (ImGui.button(labels.get("Unlock pose")))
         {
            staticRelativeNode.invalidatePose();
            ensureGizmoFrameIsSceneNodeFrame();
            sceneNode.markModifiedByOperator();
         }
         ImGui.endDisabled();
      }

      ImGui.separator();
   }

   private void ensureGizmoFrameIsSceneNodeFrame()
   {
      if (overridePoseGizmo.getPoseGizmo().getGizmoFrame() != sceneNode.getNodeFrame())
         overridePoseGizmo.getPoseGizmo().setGizmoFrame(sceneNode.getNodeFrame());
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

   public void setShowing(boolean showing)
   {
      this.showing = showing;
   }

   public RigidBodyTransform getTransformToParent()
   {
      return sceneNode.getNodeToParentFrameTransform();
   }

   public void setTransformToParent(RigidBodyTransform transform)
   {
      sceneNode.getNodeToParentFrameTransform().set(transform);
      sceneNode.getNodeFrame().update();
   }

   public ReferenceFrame getReferenceFrame()
   {
      return sceneNode.getNodeFrame();
   }

   public PredefinedRigidBodySceneNode getSceneNode()
   {
      return sceneNode;
   }
}
