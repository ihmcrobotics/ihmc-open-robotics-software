package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraphNodeMove;
import us.ihmc.perception.sceneGraph.rigidBodies.PredefinedRigidBodySceneNode;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

import java.util.List;
import java.util.Set;

/**
 * A "ghost" colored model.
 *
 * TODO: Add pose "override", via right click context menu, and gizmo.
 *   Possibly do this in a higher level class or class that extends this.
 */
public class RDXPredefinedRigidBodySceneNodeBasics
{
   private static final ColorDefinition GHOST_COLOR = ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.5);
   private final PredefinedRigidBodySceneNode predefinedRigidBodySceneNode;
   private final RDXSceneNodeBasics sceneNodeBasics;
   private final RDXModelInstance modelInstance;
   private final RDXSelectablePose3DGizmo offsetPoseGizmo;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private String initialParentName;
   private final ImBooleanWrapper trackDetectedPoseWrapper;
   private final TypedNotification<Boolean> trackDetectedPoseChanged = new TypedNotification<>();
   private transient final RigidBodyTransform visualModelToWorldTransform = new RigidBodyTransform();
   private transient final FramePose3D nodePose = new FramePose3D();

   public RDXPredefinedRigidBodySceneNodeBasics(PredefinedRigidBodySceneNode predefinedRigidBodySceneNode, RDX3DPanel panel3D)
   {
      this.predefinedRigidBodySceneNode = predefinedRigidBodySceneNode;

      sceneNodeBasics = new RDXSceneNodeBasics(predefinedRigidBodySceneNode);

      modelInstance = new RDXModelInstance(RDXModelLoader.load(predefinedRigidBodySceneNode.getVisualModelFilePath()));
      modelInstance.setColor(GHOST_COLOR);

      offsetPoseGizmo = new RDXSelectablePose3DGizmo(predefinedRigidBodySceneNode.getNodeFrame(),
                                                     predefinedRigidBodySceneNode.getNodeToParentFrameTransform());
      offsetPoseGizmo.createAndSetupDefault(panel3D);
      initialParentName = "Node " + predefinedRigidBodySceneNode.getInitialParentNodeID();
      trackDetectedPoseWrapper = new ImBooleanWrapper(predefinedRigidBodySceneNode::getTrackingInitialParent,
                                                      trackDetectedPoseChanged::set,
                                                      imBoolean -> ImGui.checkbox(labels.get("Track " + initialParentName), imBoolean));
   }

   public void update(List<SceneGraphNodeMove> sceneGraphNodeMoves)
   {
      sceneNodeBasics.update();

      if (trackDetectedPoseChanged.poll())
      {
         boolean trackDetectedPose = trackDetectedPoseChanged.read();
         predefinedRigidBodySceneNode.setTrackInitialParent(trackDetectedPose, sceneGraphNodeMoves);
         ensureGizmoFrameIsSceneNodeFrame();
         predefinedRigidBodySceneNode.markModifiedByOperator();
      }

      ensureGizmoFrameIsSceneNodeFrame();

      if (offsetPoseGizmo.getPoseGizmo().getGizmoModifiedByUser().poll())
      {
         predefinedRigidBodySceneNode.markModifiedByOperator();
      }

      nodePose.setToZero(predefinedRigidBodySceneNode.getNodeFrame());
      nodePose.set(predefinedRigidBodySceneNode.getVisualModelToNodeFrameTransform());
      nodePose.changeFrame(ReferenceFrame.getWorldFrame());
      nodePose.get(visualModelToWorldTransform);
      modelInstance.setTransformToWorldFrame(visualModelToWorldTransform);

      if (predefinedRigidBodySceneNode.getTrackingInitialParent())
         initialParentName = predefinedRigidBodySceneNode.getNodeFrame().getParent().getName();
   }

   public void renderImGuiWidgets()
   {
      sceneNodeBasics.renderImGuiWidgets();
      ImGui.sameLine();
      ImGui.text(" Parent: " + predefinedRigidBodySceneNode.getNodeFrame().getParent().getName());


      trackDetectedPoseWrapper.renderImGuiWidget();
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Show Offset Gizmo"), offsetPoseGizmo.getSelected());
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear Offset")))
      {
         predefinedRigidBodySceneNode.clearOffset();
         ensureGizmoFrameIsSceneNodeFrame();
         predefinedRigidBodySceneNode.markModifiedByOperator();
      }
   }

   private void ensureGizmoFrameIsSceneNodeFrame()
   {
      if (offsetPoseGizmo.getPoseGizmo().getGizmoFrame() != predefinedRigidBodySceneNode.getNodeFrame())
         offsetPoseGizmo.getPoseGizmo().setGizmoFrame(predefinedRigidBodySceneNode.getNodeFrame());
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      sceneNodeBasics.getRenderables(renderables, pool, sceneLevels);

      if (sceneLevels.contains(RDXSceneLevel.MODEL))
         modelInstance.getRenderables(renderables, pool);
   }
}
