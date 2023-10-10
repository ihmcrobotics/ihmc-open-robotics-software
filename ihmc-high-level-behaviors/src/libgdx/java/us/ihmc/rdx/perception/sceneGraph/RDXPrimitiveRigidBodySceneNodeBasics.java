package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImFloat;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBodies.PrimitiveRigidBodySceneNode;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

import java.util.Set;

/**
 * A "ghost" colored model.
 */
public class RDXPrimitiveRigidBodySceneNodeBasics
{
   private static final float DEFAULT_DIMENSION = 0.1F;
   private static final ColorDefinition GHOST_COLOR = ColorDefinitions.parse("0x4B61D1").derive(0.0, 1.0, 1.0, 0.5);
   private final RDX3DPanel panel3D;
   private final PrimitiveRigidBodySceneNode primitiveRigidBodySceneNode;
   private final RDXSceneNodeBasics sceneNodeBasics;
   private RDXModelInstance modelInstance;
   private final RDXSelectablePose3DGizmo offsetPoseGizmo;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private String initialParentName;
   private final ImBooleanWrapper trackDetectedPoseWrapper;
   private final TypedNotification<Boolean> trackDetectedPoseChanged = new TypedNotification<>();
   private transient final RigidBodyTransform visualModelToWorldTransform = new RigidBodyTransform();
   private transient final FramePose3D nodePose = new FramePose3D();
   private final ImFloat xLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat yLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat zLength = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat xRadius = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat yRadius = new ImFloat(DEFAULT_DIMENSION);
   private final ImFloat zRadius = new ImFloat(DEFAULT_DIMENSION);

   public RDXPrimitiveRigidBodySceneNodeBasics(PrimitiveRigidBodySceneNode primitiveRigidBodySceneNode, RDX3DPanel panel3D)
   {
      this.panel3D = panel3D;
      this.primitiveRigidBodySceneNode = primitiveRigidBodySceneNode;

      sceneNodeBasics = new RDXSceneNodeBasics(primitiveRigidBodySceneNode);

      offsetPoseGizmo = new RDXSelectablePose3DGizmo(primitiveRigidBodySceneNode.getNodeFrame(),
                                                     primitiveRigidBodySceneNode.getNodeToParentFrameTransform());
      offsetPoseGizmo.createAndSetupDefault(panel3D);
      initialParentName = "Node " + primitiveRigidBodySceneNode.getInitialParentNodeID();
      trackDetectedPoseWrapper = new ImBooleanWrapper(primitiveRigidBodySceneNode::getTrackingInitialParent,
                                                      trackDetectedPoseChanged::set,
                                                      imBoolean -> ImGui.checkbox(labels.get("Track " + initialParentName),
                                                                                  imBoolean));

      switch (primitiveRigidBodySceneNode.getShape())
      {
         case "BOX" -> modelInstance = new RDXModelInstance(RDXModelBuilder.createBox(DEFAULT_DIMENSION, DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
         case "PRISM" -> modelInstance = new RDXModelInstance(RDXModelBuilder.createPrism(DEFAULT_DIMENSION, DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
         case "CYLINDER" -> modelInstance = new RDXModelInstance(RDXModelBuilder.createCylinder(DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
         case "ELLIPSOID" -> modelInstance = new RDXModelInstance(RDXModelBuilder.createEllipsoid(DEFAULT_DIMENSION, DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
         case "CONE" -> modelInstance = new RDXModelInstance(RDXModelBuilder.createCone(DEFAULT_DIMENSION, DEFAULT_DIMENSION, Color.WHITE));
      }
      modelInstance.setColor(GHOST_COLOR);
   }

   public void update(SceneGraphModificationQueue modificationQueue)
   {
      sceneNodeBasics.update();

      if (trackDetectedPoseChanged.poll())
      {
         primitiveRigidBodySceneNode.setTrackInitialParent(trackDetectedPoseChanged.read(), modificationQueue);
         // This modification is to get queued after a basic node one
         // in order to subsequently update the UI node after the node
         // has been moved.
         // There can't be another modification added, so we pass null
         // as the modification queue.
         // That would cause an infinite loop.
         modificationQueue.accept(() -> update(null));
      }

      // Ensure gizmo frame is up to date
      if (offsetPoseGizmo.getPoseGizmo().getGizmoFrame() != primitiveRigidBodySceneNode.getNodeFrame())
         offsetPoseGizmo.getPoseGizmo().setGizmoFrame(primitiveRigidBodySceneNode.getNodeFrame());

      if (offsetPoseGizmo.getPoseGizmo().getGizmoModifiedByUser().poll())
      {
         primitiveRigidBodySceneNode.freezeFromModification();
      }

      nodePose.setToZero(primitiveRigidBodySceneNode.getNodeFrame());
      nodePose.changeFrame(ReferenceFrame.getWorldFrame());
      nodePose.get(visualModelToWorldTransform);
      modelInstance.setTransformToWorldFrame(visualModelToWorldTransform);

      if (primitiveRigidBodySceneNode.getTrackingInitialParent())
         initialParentName = primitiveRigidBodySceneNode.getNodeFrame().getParent().getName();
   }

   public void renderImGuiWidgets()
   {
      sceneNodeBasics.renderImGuiWidgets();
      ImGui.sameLine();
      ImGui.text(" Parent: " + primitiveRigidBodySceneNode.getNodeFrame().getParent().getName());
      ImGui.text(" Set shape: ");
      switch (primitiveRigidBodySceneNode.getShape())
      {
         case "BOX" ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat("depth", xLength))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat("width", yLength))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat("height", zLength))
               reshaped = true;
            if (reshaped)
            {
               modelInstance = new RDXModelInstance(RDXModelBuilder.createBox(xLength.get(), yLength.get(), zLength.get(), Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
         case "PRISM" ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat("depth", xLength))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat("width", yLength))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat("height", zLength))
               reshaped = true;
            if (reshaped)
            {
               modelInstance = new RDXModelInstance(RDXModelBuilder.createPrism(xLength.get(), yLength.get(), zLength.get(), Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
         case "CYLINDER" ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat("radius", xRadius))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat("height", zLength))
               reshaped = true;
            if (reshaped)
            {
               modelInstance = new RDXModelInstance(RDXModelBuilder.createCylinder(zLength.get(), xRadius.get(), Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
         case "ELLIPSOID" ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat("xRadius", xRadius))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat("yRadius", yRadius))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat("zRadius", zRadius))
               reshaped = true;
            if (reshaped)
            {
               modelInstance = new RDXModelInstance(RDXModelBuilder.createEllipsoid(xRadius.get(), yRadius.get(), zRadius.get(), Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
         case "CONE" ->
         {
            boolean reshaped = false;
            if (ImGuiTools.volatileInputFloat("radius", xRadius))
               reshaped = true;
            if (ImGuiTools.volatileInputFloat("height", zLength))
               reshaped = true;
            if (reshaped)
            {
               modelInstance = new RDXModelInstance(RDXModelBuilder.createCone(zLength.get(), xRadius.get(), Color.WHITE));
               modelInstance.setColor(GHOST_COLOR);
            }
         }
      }

      trackDetectedPoseWrapper.renderImGuiWidget();
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Show Offset Gizmo"), offsetPoseGizmo.getSelected());
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear Offset")))
      {
         primitiveRigidBodySceneNode.clearOffset();
         primitiveRigidBodySceneNode.freezeFromModification();
      }
   }

   public void renderRemove(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      if (sceneNodeBasics.renderRemove(modificationQueue, sceneGraph))
      {
         offsetPoseGizmo.removeRenderables(panel3D);
      }
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      sceneNodeBasics.getRenderables(renderables, pool, sceneLevels);

      if (sceneLevels.contains(RDXSceneLevel.MODEL))
         modelInstance.getRenderables(renderables, pool);
   }
}
