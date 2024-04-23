package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImDouble;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.multiBodies.door.DoorSceneNodeDefinitions;
import us.ihmc.perception.sceneGraph.yolo.YOLOv8Node;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiInputDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiPlot;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.interactable.RDXInteractableObject;

import javax.annotation.Nullable;
import java.util.List;
import java.util.Set;

public class RDXYOLOv8Node extends RDXDetectableSceneNode
{
   private final YOLOv8Node yoloNode;

   private final ImGuiUniqueLabelMap labels;

   /**
    * The mask erosion kernel radius determines how much the YOLO mask is eroded,
    * i.e. how much it is shrunk.
    * A larger radius will result is more shrinkage, and a radius of 0 will not shrink the mask.
    * Erosion is useful when the YOLO mask is slightly larger than the object it detects.
    */
   private final ImInt maskErosionKernelRadius;
   private final ImDouble outlierFilterThreshold;
   private final ImFloat detectionAcceptanceThreshold;
   private final ImGuiInputDoubleWrapper alphaFilterValueSlider;
   private final ImGuiPlot confidencePlot = new ImGuiPlot("Confidence", 1000, 230, 22);

   private final RDXPointCloudRenderer objectPointCloudRenderer = new RDXPointCloudRenderer();
   private final RDXReferenceFrameGraphic objectPoseGraphic = new RDXReferenceFrameGraphic(0.2);

   @Nullable
   private RDXInteractableObject interactableObject;

   private RigidBodyTransform visualModelTransformToWorld = new RigidBodyTransform();

   public RDXYOLOv8Node(YOLOv8Node yoloNode, ImGuiUniqueLabelMap labels)
   {
      super(yoloNode);
      this.yoloNode = yoloNode;
      this.labels = labels;

      maskErosionKernelRadius = new ImInt(yoloNode.getMaskErosionKernelRadius());
      outlierFilterThreshold = new ImDouble(yoloNode.getOutlierFilterThreshold());
      detectionAcceptanceThreshold = new ImFloat(yoloNode.getDetectionAcceptanceThreshold());

      alphaFilterValueSlider = new ImGuiInputDoubleWrapper("Break frequency:",
                                                           "%.2f",
                                                           0.05,
                                                           5.0,
                                                           yoloNode::getAlpha,
                                                           yoloNode::setAlpha,
                                                           yoloNode::freeze);
      alphaFilterValueSlider.setWidgetWidth(100.0f);

      objectPointCloudRenderer.create(5000);
      objectPoseGraphic.setPoseInWorldFrame(yoloNode.getObjectPose());
   }

   private void createInteractableObject()
   {
      switch (yoloNode.getDetectionClass())
      {
         case DOOR_LEVER ->
         {
            interactableObject = new RDXInteractableObject(RDXBaseUI.getInstance());
            interactableObject.load(DoorSceneNodeDefinitions.DOOR_LEVER_HANDLE_VISUAL_MODEL_FILE_PATH,
                                    DoorSceneNodeDefinitions.RIGHT_DOOR_LEVER_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
         }
         case DOOR_PULL_HANDLE ->
         {
            interactableObject = new RDXInteractableObject(RDXBaseUI.getInstance());
            interactableObject.load(DoorSceneNodeDefinitions.DOOR_PULL_HANDLE_VISUAL_MODEL_FILE_PATH,
                                    DoorSceneNodeDefinitions.DOOR_PULL_HANDLE_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
         }
         case DOOR_PUSH_BAR ->
         {
            interactableObject = new RDXInteractableObject(RDXBaseUI.getInstance());
            interactableObject.load(DoorSceneNodeDefinitions.DOOR_EMERGENCY_BAR_VISUAL_MODEL_FILE_PATH,
                                    DoorSceneNodeDefinitions.LEFT_DOOR_EMERGENCY_BAR_VISUAL_MODEL_TO_NODE_FRAME_TRANSFORM);
         }
      }
   }

   @Override
   public void update(SceneGraphModificationQueue modificationQueue)
   {
      super.update(modificationQueue);

      yoloNode.setMaskErosionKernelRadius(maskErosionKernelRadius.get());
      yoloNode.setOutlierFilterThreshold(outlierFilterThreshold.get());
      yoloNode.setDetectionAcceptanceThreshold(detectionAcceptanceThreshold.get());

      if (interactableObject != null)
      {
         visualModelTransformToWorld.getTranslation().set(yoloNode.getFilteredObjectPose().getTranslation());
         visualModelTransformToWorld.getRotation().set(yoloNode.getFilteredObjectPose().getRotation());

         LibGDXTools.setDiffuseColor(interactableObject.getModelInstance(), Color.WHITE); // TODO: keep?
         interactableObject.setPose(visualModelTransformToWorld);
      }
      else
      {
         createInteractableObject();
      }
   }

   @Override
   public void renderImGuiWidgets(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiWidgets(modificationQueue, sceneGraph);

      confidencePlot.setWidth((int) (0.65 * ImGui.getWindowWidth()));
      ImGui.pushStyleColor(ImGuiCol.PlotLines, ImGuiTools.greenRedGradientColor((float) yoloNode.getConfidence(), 1.0f, 0.0f));
      confidencePlot.render(yoloNode.getConfidence());
      ImGui.popStyleColor();

      if (ImGuiTools.volatileInputInt(labels.get("Mask Erosion Kernel Radius"), maskErosionKernelRadius))
         maskErosionKernelRadius.set(MathTools.clamp(maskErosionKernelRadius.get(), 0, 10));
      if (ImGuiTools.volatileInputDouble(labels.get("Outlier Filter Threshold"), outlierFilterThreshold))
         outlierFilterThreshold.set(MathTools.clamp(outlierFilterThreshold.get(), 0.0, 5.0));
      if (ImGuiTools.volatileInputFloat(labels.get("Detection Acceptance Threshold"), detectionAcceptanceThreshold))
         detectionAcceptanceThreshold.set((float) MathTools.clamp(detectionAcceptanceThreshold.get(), 0.0f, 1.0f));
      alphaFilterValueSlider.renderImGuiWidget();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      super.getRenderables(renderables, pool, sceneLevels);

      List<Point3D32> renderablePointCloud = yoloNode.getObjectPointCloud();
      objectPointCloudRenderer.setPointsToRender(renderablePointCloud, Color.GREEN);
      objectPointCloudRenderer.updateMesh();
      objectPointCloudRenderer.getRenderables(renderables, pool);

      objectPoseGraphic.setPoseInWorldFrame(yoloNode.getObjectPose());
      objectPoseGraphic.getRenderables(renderables, pool);

      if (interactableObject != null)
         interactableObject.getRenderables(renderables, pool);
   }

   @Override
   public void destroy()
   {
      super.destroy();

      objectPointCloudRenderer.dispose();
      objectPoseGraphic.dispose();

      if (interactableObject != null)
      {
         interactableObject.getModelInstance().model.dispose();
         interactableObject.clear();
      }
   }
}
