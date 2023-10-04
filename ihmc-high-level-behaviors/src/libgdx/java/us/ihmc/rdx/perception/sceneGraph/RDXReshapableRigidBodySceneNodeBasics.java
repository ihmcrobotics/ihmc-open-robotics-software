package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBodies.ReshapableRigidBodySceneNode;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

import java.util.List;
import java.util.Set;

import static us.ihmc.rdx.perception.sceneGraph.RDXPrimitiveRigidBodyShapes.getAvailableShapes;

/**
 * A "ghost" colored model.
 */
public class RDXReshapableRigidBodySceneNodeBasics
{
   private static final ColorDefinition GHOST_COLOR = ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.5);
   private final ReshapableRigidBodySceneNode reshapableRigidBodySceneNode;
   private final RDXSceneNodeBasics sceneNodeBasics;
   private RDXModelInstance modelInstance;
   private final List<RDXPrimitiveRigidBodyShapes> availableShapes;
   private RDXPrimitiveRigidBodyShapes selectedShape;
   private final RDXSelectablePose3DGizmo offsetPoseGizmo;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private String initialParentName;
   private final ImBooleanWrapper trackDetectedPoseWrapper;
   private final TypedNotification<Boolean> trackDetectedPoseChanged = new TypedNotification<>();
   private transient final RigidBodyTransform visualModelToWorldTransform = new RigidBodyTransform();
   private transient final FramePose3D nodePose = new FramePose3D();

   public RDXReshapableRigidBodySceneNodeBasics(ReshapableRigidBodySceneNode resizableRigidBodySceneNode, RDX3DPanel panel3D)
   {
      this.reshapableRigidBodySceneNode = resizableRigidBodySceneNode;

      sceneNodeBasics = new RDXSceneNodeBasics(resizableRigidBodySceneNode);

      offsetPoseGizmo = new RDXSelectablePose3DGizmo(resizableRigidBodySceneNode.getNodeFrame(),
                                                     resizableRigidBodySceneNode.getNodeToParentFrameTransform());
      offsetPoseGizmo.createAndSetupDefault(panel3D);
      initialParentName = "Node " + resizableRigidBodySceneNode.getInitialParentNodeID();
      trackDetectedPoseWrapper = new ImBooleanWrapper(resizableRigidBodySceneNode::getTrackingInitialParent,
                                                      trackDetectedPoseChanged::set,
                                                      imBoolean -> ImGui.checkbox(labels.get("Track " + initialParentName),
                                                                                  imBoolean));
      availableShapes = getAvailableShapes();
   }

   public void update(SceneGraphModificationQueue modificationQueue)
   {
      sceneNodeBasics.update();

      if (trackDetectedPoseChanged.poll())
      {
         reshapableRigidBodySceneNode.setTrackInitialParent(trackDetectedPoseChanged.read(), modificationQueue);
         // This modification is to get queued after a basic node one
         // in order to subsequently update the UI node after the node
         // has been moved.
         // There can't be another modification added, so we pass null
         // as the modification queue.
         // That would cause an infinite loop.
         modificationQueue.accept(() -> update(null));
      }

      // Ensure gizmo frame is up to date
      if (offsetPoseGizmo.getPoseGizmo().getGizmoFrame() != reshapableRigidBodySceneNode.getNodeFrame())
         offsetPoseGizmo.getPoseGizmo().setGizmoFrame(reshapableRigidBodySceneNode.getNodeFrame());

      if (offsetPoseGizmo.getPoseGizmo().getGizmoModifiedByUser().poll())
      {
         reshapableRigidBodySceneNode.freezeFromModification();
      }

      nodePose.setToZero(reshapableRigidBodySceneNode.getNodeFrame());
      nodePose.changeFrame(ReferenceFrame.getWorldFrame());
      nodePose.get(visualModelToWorldTransform);
      modelInstance.setTransformToWorldFrame(visualModelToWorldTransform);

      if (reshapableRigidBodySceneNode.getTrackingInitialParent())
         initialParentName = reshapableRigidBodySceneNode.getNodeFrame().getParent().getName();
   }

   public void renderImGuiWidgets()
   {
      sceneNodeBasics.renderImGuiWidgets();
      ImGui.sameLine();
      ImGui.text(" Parent: " + reshapableRigidBodySceneNode.getNodeFrame().getParent().getName());
      ImGui.text(" Set shape: ");
      for (RDXPrimitiveRigidBodyShapes shape : availableShapes)
      {
         if (ImGui.button(labels.get(shape.toString())))
         {
            selectedShape = shape;
            switch (selectedShape)
            {
               case BOX -> modelInstance = new RDXModelInstance(RDXModelBuilder.createBox((float) 0.1, (float) 0.1, (float) 0.1, Color.WHITE));
               case PYRAMID -> modelInstance = new RDXModelInstance(RDXModelBuilder.createPyramid((float) 0.1, (float) 0.1, (float) 0.1, Color.WHITE));
               case CYLINDER -> modelInstance = new RDXModelInstance(RDXModelBuilder.createCylinder((float) 0.1, (float) 0.1, Color.WHITE));
               case SPHERE -> modelInstance = new RDXModelInstance(RDXModelBuilder.createSphere((float) 0.1, Color.WHITE));
               case ELLIPSOID -> modelInstance = new RDXModelInstance(RDXModelBuilder.createTorus((float) 0.1, (float) 0.1, (float) 0.1, (float) 0.1, Color.WHITE));
               case TORUS -> modelInstance = new RDXModelInstance(RDXModelBuilder.createEllipsoid((float) 0.1, (float) 0.3, (float) 0.1, Color.WHITE));
            }
            modelInstance.setColor(GHOST_COLOR);
         }
      }

      trackDetectedPoseWrapper.renderImGuiWidget();
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Show Offset Gizmo"), offsetPoseGizmo.getSelected());
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear Offset")))
      {
         reshapableRigidBodySceneNode.clearOffset();
         reshapableRigidBodySceneNode.freezeFromModification();
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      sceneNodeBasics.getRenderables(renderables, pool, sceneLevels);

      if (sceneLevels.contains(RDXSceneLevel.MODEL))
         modelInstance.getRenderables(renderables, pool);
   }
}
