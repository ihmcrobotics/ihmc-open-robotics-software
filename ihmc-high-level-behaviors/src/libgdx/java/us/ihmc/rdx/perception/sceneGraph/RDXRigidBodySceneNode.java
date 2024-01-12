package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneNode;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXBallAndArrowPosePlacement;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

import java.util.Set;

public abstract class RDXRigidBodySceneNode extends RDXSceneNode
{
   protected static final ColorDefinition GHOST_COLOR = ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.6);

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final RigidBodySceneNode rigidBodySceneNode;
   private final RigidBodyTransform visualModelToNodeTransform = new RigidBodyTransform();
   private final RDX3DPanel panel3D;

   private final RDXBallAndArrowPosePlacement posePlacement = new RDXBallAndArrowPosePlacement();
   private final RDXSelectablePose3DGizmo offsetPoseGizmo;
   private String initialParentName;
   private final ImBooleanWrapper trackDetectedPoseWrapper;
   private final TypedNotification<Boolean> trackDetectedPoseChanged = new TypedNotification<>();

   protected transient final FramePose3D nodePose = new FramePose3D();
   protected transient final FramePose3D visualModelPose = new FramePose3D();

   private transient final RigidBodyTransform visualModelToWorldTransform = new RigidBodyTransform();

   public RDXRigidBodySceneNode(RigidBodySceneNode rigidBodySceneNode, RigidBodyTransform visualModelToNodeTransform, RDX3DPanel panel3D)
   {
      super(rigidBodySceneNode);

      this.rigidBodySceneNode = rigidBodySceneNode;
      this.visualModelToNodeTransform.set(visualModelToNodeTransform);
      this.panel3D = panel3D;

      posePlacement.create(Color.YELLOW, "Place Node Pose", "Placing Node", "Stop Placement");
      RDXBaseUI.getInstance().getPrimary3DPanel().addImGui3DViewInputProcessor(posePlacement::processImGui3DViewInput);
      posePlacement.setOnEndPositionPlacement(() ->
      {
         if (posePlacement.isPlacingPosition())
            posePlacement.clear();
      });

      offsetPoseGizmo = new RDXSelectablePose3DGizmo(rigidBodySceneNode.getNodeFrame(), rigidBodySceneNode.getNodeToParentFrameTransform());
      offsetPoseGizmo.createAndSetupDefault(panel3D);
      initialParentName = "Node " + rigidBodySceneNode.getInitialParentNodeID();
      trackDetectedPoseWrapper = new ImBooleanWrapper(rigidBodySceneNode::getTrackingInitialParent,
                                                      trackDetectedPoseChanged::set,
                                                      imBoolean -> ImGui.checkbox(labels.get("Track " + initialParentName), imBoolean));
   }

   public void update(SceneGraphModificationQueue modificationQueue)
   {
      if (trackDetectedPoseChanged.poll())
      {
         rigidBodySceneNode.setTrackInitialParent(trackDetectedPoseChanged.read(), modificationQueue);
         // This modification is to get queued after a basic node one
         // in order to subsequently update the UI node after the node
         // has been moved.
         // There can't be another modification added, so we pass null
         // as the modification queue.
         // That would cause an infinite loop.
         modificationQueue.accept(() -> update(null));
      }

      // Ensure gizmo frame is up to date
      if (offsetPoseGizmo.getPoseGizmo().getGizmoFrame() != rigidBodySceneNode.getNodeFrame())
         offsetPoseGizmo.getPoseGizmo().setGizmoFrame(rigidBodySceneNode.getNodeFrame());

      if (offsetPoseGizmo.getPoseGizmo().getGizmoModifiedByUser().poll())
      {
         rigidBodySceneNode.freeze();
      }

      if (posePlacement.isPlacingGoal())
      {
         rigidBodySceneNode.getModifiableNodeFrame().update(transformToParent -> transformToParent.set(posePlacement.getGoalPose()));
      }

      if (posePlacement.getPlacedNotification().poll())
      {
         rigidBodySceneNode.freeze();
         posePlacement.setGoalPoseNoCallbacks(new Pose3D());
         posePlacement.clear();
      }

      nodePose.setToZero(rigidBodySceneNode.getNodeFrame());
      nodePose.changeFrame(ReferenceFrame.getWorldFrame());

      if (rigidBodySceneNode.getTrackingInitialParent())
         initialParentName = rigidBodySceneNode.getNodeFrame().getParent().getName();

      visualModelPose.setIncludingFrame(rigidBodySceneNode.getNodeFrame(), visualModelToNodeTransform);
      visualModelPose.changeFrame(ReferenceFrame.getWorldFrame());

      // Quaternion -> Rotation matrix for LibGDX
      visualModelToWorldTransform.set(visualModelPose);

      getModelInstance().setTransformToWorldFrame(visualModelToWorldTransform);
   }

   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);
      ImGui.sameLine();
      ImGui.text(" Parent: " + rigidBodySceneNode.getNodeFrame().getParent().getName());

      posePlacement.renderPlaceGoalButton();
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear Offset")))
      {
         rigidBodySceneNode.clearOffset();
         rigidBodySceneNode.freeze();
      }
      trackDetectedPoseWrapper.renderImGuiWidget();
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Show Offset Gizmo"), offsetPoseGizmo.getSelected());
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);

      posePlacement.getRenderables(renderables, pool);
   }

   @Override
   public void remove(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.remove(modificationQueue, sceneGraph);
      offsetPoseGizmo.removeRenderables(panel3D);
   }

   public abstract RDXModelInstance getModelInstance();
}
