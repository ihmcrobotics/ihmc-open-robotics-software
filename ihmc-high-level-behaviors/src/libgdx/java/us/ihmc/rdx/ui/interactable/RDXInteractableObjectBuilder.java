package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.Color;
import imgui.internal.ImGui;
import imgui.type.ImFloat;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.util.*;

public class RDXInteractableObjectBuilder extends RDXPanel
{
   private final static String WINDOW_NAME = "Object Panel";
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXInteractableObject selectedObject;
   private final SortedMap<String, PredefinedRigidBodySceneNode> nameToNodesMap = new TreeMap<>();
   private final SortedMap<String, PrimitiveRigidBodySceneNode> nameToPrimitiveNodesMap = new TreeMap<>();
   private String selectedObjectName = "";
   private final TypedNotification<String> selectedObjectChanged = new TypedNotification<>();

   public RDXInteractableObjectBuilder(RDXBaseUI baseUI, SceneGraph sceneGraph)
   {
      super(WINDOW_NAME);
      setRenderMethod(this::renderImGuiWidgets);

      selectedObject = new RDXInteractableObject(baseUI);

      registerSceneSubtree(sceneGraph.getRootNode());
   }

   private void registerSceneSubtree(SceneNode sceneNode)
   {
      if (sceneNode instanceof PredefinedRigidBodySceneNode predefinedRigidBodySceneNode)
      {
         nameToNodesMap.put(predefinedRigidBodySceneNode.getName(), predefinedRigidBodySceneNode);
      }

      if (sceneNode instanceof PrimitiveRigidBodySceneNode primitiveRigidBodySceneNode)
      {
         nameToPrimitiveNodesMap.put(primitiveRigidBodySceneNode.getName(), primitiveRigidBodySceneNode);
      }

      for (SceneNode child : sceneNode.getChildren())
      {
         registerSceneSubtree(child);
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Selected: " + (!isAnyObjectSelected() ? "" : (selectedObjectChanged.read())));
      ImGui.text("Predefined object: ");
      for (PredefinedRigidBodySceneNode predefinedRigidBodySceneNode : nameToNodesMap.values())
      {
         if (ImGui.button(labels.get(predefinedRigidBodySceneNode.getName())))
         {
            if (isAnyObjectSelected())
               selectedObject.clear();
            selectedObject.load(predefinedRigidBodySceneNode.getVisualModelFilePath(), predefinedRigidBodySceneNode.getVisualModelToNodeFrameTransform());
            // Notify that the object selection has been updated
            selectedObjectName = predefinedRigidBodySceneNode.getName();
            selectedObjectChanged.set(selectedObjectName);
         }
         ImGui.separator();
      }
      ImGui.text("Resizable Primitive Objects: ");
      for (PrimitiveRigidBodySceneNode primitiveRigidBodySceneNode : nameToPrimitiveNodesMap.values())
      {
         if (ImGui.button(labels.get(primitiveRigidBodySceneNode.getName())))
         {
            if (isAnyObjectSelected())
               selectedObject.clear();
            selectedObject.setVisuals(primitiveRigidBodySceneNode);
            // Notify that the object selection has been updated
            selectedObjectName = primitiveRigidBodySceneNode.getName();
            selectedObjectChanged.set(selectedObjectName);
         }
         ImGui.separator();
      }
      if (isAnyObjectSelected() && (ImGui.button(labels.get("Set Initial Pose") + "##object")))
      {
         selectedObject.setInitialPose();
      }
      ImGui.sameLine();
      if (isAnyObjectSelected() && (ImGui.button(labels.get("Reset To Initial Pose") + "##object")))
      {
         selectedObject.resetToInitialPose();
      }
      ImGui.sameLine();
      if (isAnyObjectSelected() && (ImGui.button(labels.get("Delete") + "##object")))
      {
         reset();
      }
      if (isAnyObjectSelected() && (ImGui.button(labels.get("Hide/Show Gizmo") + "##object")))
      {
         selectedObject.switchGizmoVisualization();
      }
      if (selectedObject.shape != null)
      {
         selectedObject.updateVisuals();
      }
   }

   public void reset()
   {
      selectedObject.clear();
      selectedObjectName = "";
      selectedObjectChanged.set(selectedObjectName);
      selectedObject.resetPose();
   }

   public void resetPose()
   {
      selectedObject.resetPose();
   }

   public TypedNotification<String> getSelectedObjectNotification()
   {
      return selectedObjectChanged;
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }

   public void loadObject(String objectName)
   {
      PredefinedRigidBodySceneNode predefinedRigidBodySceneNode = nameToNodesMap.get(objectName);
      selectedObject.load(predefinedRigidBodySceneNode.getVisualModelFilePath(), predefinedRigidBodySceneNode.getVisualModelToNodeFrameTransform());
      selectedObjectName = objectName;
      selectedObjectChanged.set(selectedObjectName);
   }

   public RDXInteractableObject getSelectedObject()
   {
      return selectedObject;
   }

   public String getSelectedObjectName()
   {
      return selectedObjectName;
   }

   public boolean isAnyObjectSelected()
   {
      return selectedObject.getModelInstance() != null;
   }
}