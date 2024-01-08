package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.rigidBody.RigidBodySceneObjectDefinitions;
import us.ihmc.rdx.RDX3DSituatedText;
import us.ihmc.rdx.RDX3DSituatedTextData;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.interactable.RDXInteractableObject;

import javax.annotation.Nullable;
import java.util.Set;

public class RDXCenterposeNode extends RDXDetectableSceneNode
{
   private final CenterposeNode centerposeNode;
   private final Set<RDXSceneLevel> sceneLevels = Set.of(RDXSceneLevel.MODEL);
   private final RDX3DSituatedText text;
   private final FramePose3D textPose = new FramePose3D();
   private RDX3DSituatedTextData previousTextData;
   private ModelInstance boundingBoxModelInstance;
   private final FramePoint3D[] vertices3D = new FramePoint3D[8];
   private final RDX3DPanel panel3D;
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final ImBoolean showBoundingBox = new ImBoolean(false);
   private final ImBoolean enableTracking = new ImBoolean(true);

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   @Nullable
   private RDXInteractableObject interactableObject;

   private boolean wentUndetected;

   public RDXCenterposeNode(CenterposeNode centerposeNode, RDX3DPanel panel3D)
   {
      super(centerposeNode);
      this.panel3D = panel3D;
      this.centerposeNode = centerposeNode;
      this.text = new RDX3DSituatedText(getObjectTooltip(), 0.05f);
   }

   private String getObjectTooltip()
   {
      if (centerposeNode.getCurrentlyDetected())
      {
         return centerposeNode.getObjectType() + " %.1f".formatted(centerposeNode.getConfidence());
      }
      else
      {
         return centerposeNode.getObjectType() + " NOT DETECTED";
      }
   }

   @Override
   public void update(SceneGraphModificationQueue modificationQueue)
   {
      super.update(modificationQueue);

      if (centerposeNode.isEnableTracking() != enableTracking.get())
      {
         centerposeNode.setEnableTracking(enableTracking.get());
      }

      Point3D[] vertices = centerposeNode.getVertices3D();
      for (int i = 0; i < vertices.length; i++)
      {
         if (vertices3D[i] == null)
         {
            vertices3D[i] = new FramePoint3D();
         }
         vertices3D[i].changeFrame(ReferenceFrame.getWorldFrame());
         vertices3D[i].interpolate(vertices[i], 1); // optional interpolation
      }
      Model boundingBoxModel = RDXModelBuilder.buildModel(boxMeshBuilder -> boxMeshBuilder.addMultiLineBox(vertices3D, 0.005, Color.WHITE));

      if (boundingBoxModelInstance != null)
      {
         boundingBoxModelInstance.model.dispose();
      }
      boundingBoxModelInstance = new RDXModelInstance(boundingBoxModel);

      if (previousTextData != null)
         previousTextData.dispose();
      previousTextData = text.setTextWithoutCache(getObjectTooltip());

      textPose.setToZero(panel3D.getCamera3D().getCameraFrame());
      textPose.getOrientation().appendPitchRotation(3.0 / 2.0 * Math.PI);
      textPose.getOrientation().appendYawRotation(-Math.PI / 2.0);

      textPose.changeFrame(ReferenceFrame.getWorldFrame());
      textPose.getPosition().set(centerposeNode.getNodeFrame().getTransformToWorldFrame().getTranslation()); // Not sure if this is correct?

      LibGDXTools.toLibGDX(textPose, tempTransform, text.getModelTransform());

      if (interactableObject == null)
      {
         createInteractableObject();
      }
      else
      {
         if (centerposeNode.getCurrentlyDetected() && centerposeNode.isEnableTracking())
         {
            interactableObject.setPose(centerposeNode.getNodeToParentFrameTransform());
            LibGDXTools.setOpacity(interactableObject.getModelInstance(), (float) centerposeNode.getConfidence());
            LibGDXTools.setDiffuseColor(interactableObject.getModelInstance(), Color.WHITE); // TODO: keep?
         }

         if (centerposeNode.getCurrentlyDetected() && wentUndetected)
         {
            wentUndetected = false;
            // Recreate to remove the red diffuse
            createInteractableObject();
         }
         else if (!centerposeNode.getCurrentlyDetected())
         {
            LibGDXTools.setDiffuseColor(interactableObject.getModelInstance(), Color.RED);
            LibGDXTools.setOpacity(interactableObject.getModelInstance(), 0.2f);
            wentUndetected = true;
         }
      }
   }

   private void createInteractableObject()
   {
      if (interactableObject != null)
      {
         interactableObject.getModelInstance().model.dispose();
         interactableObject.clear();
      }

      if (centerposeNode.getObjectType().equals("SHOE"))
      {
         interactableObject = new RDXInteractableObject(RDXBaseUI.getInstance());
         interactableObject.load(RigidBodySceneObjectDefinitions.SHOE_VISUAL_MODEL_FILE_PATH, RigidBodySceneObjectDefinitions.SHOE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      }
      else if (centerposeNode.getObjectType().equals("LAPTOP"))
      {
         interactableObject = new RDXInteractableObject(RDXBaseUI.getInstance());
         interactableObject.load(RigidBodySceneObjectDefinitions.THINKPAD_VISUAL_MODEL_FILE_PATH, RigidBodySceneObjectDefinitions.THINKPAD_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
      }
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);
      ImGui.checkbox(labels.get("Show bounding box"), showBoundingBox);
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Enable tracking"), enableTracking);
      ImGui.text("ID: %d".formatted(centerposeNode.getObjectID()));
      ImGui.sameLine();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);
      if (sceneLevelCheck(sceneLevels))
      {
         if (boundingBoxModelInstance != null)
         {
            text.getRenderables(renderables, pool);
            if (showBoundingBox.get())
               boundingBoxModelInstance.getRenderables(renderables, pool);
         }

         if (interactableObject != null)
            interactableObject.getRenderables(renderables, pool);
      }
   }

   public boolean sceneLevelCheck(Set<RDXSceneLevel> sceneLevels)
   {
      for (RDXSceneLevel sceneLevel : this.sceneLevels)
         if (sceneLevels.contains(sceneLevel))
            return true;
      return false;
   }
}
