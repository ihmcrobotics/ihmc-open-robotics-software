package us.ihmc.rdx.perception.sceneGraph;

import imgui.ImGui;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneNode;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;

public class RDXRigidBodySceneNode extends RDXSceneNode
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final RigidBodySceneNode rigidBodySceneNode;
   private final RDX3DPanel panel3D;

   private final RDXSelectablePose3DGizmo offsetPoseGizmo;
   private String initialParentName;
   private final ImBooleanWrapper trackDetectedPoseWrapper;
   private final TypedNotification<Boolean> trackDetectedPoseChanged = new TypedNotification<>();
   private transient final RigidBodyTransform visualModelToWorldTransform = new RigidBodyTransform();
   protected transient final FramePose3D nodePose = new FramePose3D();

   public RDXRigidBodySceneNode(RigidBodySceneNode rigidBodySceneNode, RDX3DPanel panel3D)
   {
      super(rigidBodySceneNode);

      this.rigidBodySceneNode = rigidBodySceneNode;
      this.panel3D = panel3D;

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
         rigidBodySceneNode.freezeFromModification();
      }

      nodePose.setToZero(rigidBodySceneNode.getNodeFrame());
      nodePose.changeFrame(ReferenceFrame.getWorldFrame());
      nodePose.get(visualModelToWorldTransform);

      if (rigidBodySceneNode.getTrackingInitialParent())
         initialParentName = rigidBodySceneNode.getNodeFrame().getParent().getName();
   }

   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);
      ImGui.sameLine();
      ImGui.text(" Parent: " + rigidBodySceneNode.getNodeFrame().getParent().getName());

      trackDetectedPoseWrapper.renderImGuiWidget();
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Show Offset Gizmo"), offsetPoseGizmo.getSelected());
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear Offset")))
      {
         rigidBodySceneNode.clearOffset();
         rigidBodySceneNode.freezeFromModification();
      }
   }

   @Override
   public void remove(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.remove(modificationQueue, sceneGraph);
      offsetPoseGizmo.removeRenderables(panel3D);
   }
}
